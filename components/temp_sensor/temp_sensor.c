/*
 * temp_sensor.c
 *
 * DS18B20 1-Wire Temperature Sensor Driver — Implementation
 *
 * Bit-bang 1-Wire protocol on GPIO 32 using esp_rom_delay_us() for
 * microsecond-resolution timing.  Critical sections protect the
 * timing-sensitive bus operations from FreeRTOS preemption.
 *
 * References:
 *   - DS18B20 datasheet (Maxim Integrated)
 *   - David Antliff's esp32-owb / esp32-ds18b20 (Apache 2.0)
 */

#include "temp_sensor.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

static const char *TAG = "temp_sensor";

#define OW_GPIO               GPIO_NUM_32

/* 1-Wire ROM Commands */
#define OW_CMD_SKIP_ROM       0xCC
#define OW_CMD_MATCH_ROM      0x55

/* DS18B20 Function Commands */
#define DS18B20_CMD_CONVERT_T     0x44
#define DS18B20_CMD_READ_SCRATCH  0xBE

/* 1-Wire bus timing (standard speed, in microseconds) */
#define OW_RESET_LOW_US        480
#define OW_RESET_WAIT_US        70
#define OW_RESET_DELAY_US      410

#define OW_WRITE_1_LOW_US       6
#define OW_WRITE_1_HIGH_US      64

#define OW_WRITE_0_LOW_US       60
#define OW_WRITE_0_HIGH_US      10

#define OW_READ_LOW_US          3
#define OW_READ_SAMPLE_US       12
#define OW_READ_HIGH_US         50

/* DS18B20 conversion time (750 ms for 12-bit resolution, per datasheet) */
#define DS18B20_CONV_WAIT_MS    800 /* datasheet maximum 750 ms at 12-bit + margin */

/* -------------------------------------------------------------------------- */
/* 1-Wire Low-Level Bus Operations                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Set 1-Wire GPIO direction: output (open-drain via push-pull + float)
 *
 * The ESP32 does not have true open-drain; we emulate it by switching between
 * output-low (drive 0) and input/high-impedance (release, pulled high by the
 * external 4.7 kΩ resistor).
 */
static inline void ow_set_output(void)
{
    gpio_set_direction(OW_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(OW_GPIO, 0);
}

static inline void ow_set_input(void)
{
    gpio_set_direction(OW_GPIO, GPIO_MODE_INPUT);
}

static inline int ow_read_level(void)
{
    return gpio_get_level(OW_GPIO);
}

/**
 * @brief 1-Wire reset — pull bus low, release, check for presence pulse.
 *
 * Sequence:
 *   1. Pull bus low for 480 µs
 *   2. Release bus, wait 70 µs
 *   3. Sample — if a device is present it will pull low within 60–240 µs
 *   4. Wait remaining slot time (~410 µs)
 *
 * @return true if a device presence pulse was detected.
 */
static bool onewire_reset(void)
{
    portDISABLE_INTERRUPTS();

    /* Drive bus low for 480 µs */
    ow_set_output();
    esp_rom_delay_us(OW_RESET_LOW_US);

    /* Release bus and wait for device to respond */
    ow_set_input();
    esp_rom_delay_us(OW_RESET_WAIT_US);

    /* Sample the bus — device pulls low if present */
    int level = ow_read_level();
    bool present = (level == 0);

    /* Wait for the remainder of the reset time slot */
    esp_rom_delay_us(OW_RESET_DELAY_US);

    portENABLE_INTERRUPTS();

    return present;
}

/**
 * @brief Write a single bit to the 1-Wire bus.
 *
 * Write-1: low for 6 µs, release for 64 µs.
 * Write-0: low for 60 µs, release for 10 µs.
 */
static void onewire_write_bit(bool bit)
{
    portDISABLE_INTERRUPTS();

    ow_set_output();                    /* pull bus low — start of time slot */

    if (bit) {
        esp_rom_delay_us(OW_WRITE_1_LOW_US);
        ow_set_input();                 /* release early for write-1 */
        esp_rom_delay_us(OW_WRITE_1_HIGH_US);
    } else {
        esp_rom_delay_us(OW_WRITE_0_LOW_US);
        ow_set_input();                 /* release late for write-0 */
        esp_rom_delay_us(OW_WRITE_0_HIGH_US);
    }

    portENABLE_INTERRUPTS();
}

/**
 * @brief Read a single bit from the 1-Wire bus.
 *
 * Sequence:
 *   1. Pull bus low for 3 µs (initiate read time slot)
 *   2. Release bus, wait 12 µs, sample
 *   3. Wait remaining 50 µs to complete the slot
 */
static bool onewire_read_bit(void)
{
    int level;

    portDISABLE_INTERRUPTS();

    ow_set_output();                    /* initiate read slot */
    esp_rom_delay_us(OW_READ_LOW_US);

    ow_set_input();                     /* release — device drives line */
    esp_rom_delay_us(OW_READ_SAMPLE_US);

    level = ow_read_level();            /* sample */

    esp_rom_delay_us(OW_READ_HIGH_US);  /* complete slot time */

    portENABLE_INTERRUPTS();

    return (level == 1);
}

static void onewire_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

static uint8_t onewire_read_byte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (onewire_read_bit()) {
            byte |= 0x80;
        }
    }
    return byte;
}

/* -------------------------------------------------------------------------- */
/* DS18B20 High-Level Operations                                              */
/* -------------------------------------------------------------------------- */

bool temp_sensor_init(void)
{
    /* Configure GPIO as open-drain (emulated) with input as default state */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OW_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,   /* external 4.7k pull-up */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Verify device presence on the bus */
    if (!onewire_reset()) {
        ESP_LOGE(TAG, "No DS18B20 detected on GPIO %d — check wiring and pull-up resistor",
                 OW_GPIO);
        return false;
    }

    ESP_LOGI(TAG, "DS18B20 detected on GPIO %d", OW_GPIO);
    return true;
}

bool temp_sensor_start_conversion(void)
{
    if (!onewire_reset()) {
        ESP_LOGE(TAG, "Bus reset failed during start_conversion");
        return false;
    }

    onewire_write_byte(OW_CMD_SKIP_ROM);
    onewire_write_byte(DS18B20_CMD_CONVERT_T);

    return true;
}

float temp_sensor_read_temp(void)
{
    /* Read scratchpad for the last conversion result */
    if (!onewire_reset()) {
        ESP_LOGE(TAG, "Bus reset failed during read_temp");
        return TEMP_SENSOR_NO_DEVICE;
    }

    onewire_write_byte(OW_CMD_SKIP_ROM);
    onewire_write_byte(DS18B20_CMD_READ_SCRATCH);

    /* Read temperature LSB and MSB (scratchpad bytes 0 and 1) */
    uint8_t temp_lsb = onewire_read_byte();
    uint8_t temp_msb = onewire_read_byte();

    /* Reset bus to abort scratchpad read (we only needed the first 2 bytes) */
    onewire_reset();

    /* Reconstruct 16-bit signed temperature value */
    int16_t raw = ((int16_t)temp_msb << 8) | temp_lsb;

    /* DS18B20: 12-bit resolution → LSB = 0.0625 °C */
    float temperature = (float)raw * 0.0625f;

    ESP_LOGI(TAG, "Temperature: %.2f °C (raw: 0x%04X)", temperature, raw);

    return temperature;
}

float temp_sensor_sample(void)
{
    if (!temp_sensor_start_conversion()) {
        return TEMP_SENSOR_NO_DEVICE;
    }

    /* Wait for conversion to complete (750 ms at 12-bit resolution) */
    vTaskDelay(pdMS_TO_TICKS(DS18B20_CONV_WAIT_MS));

    return temp_sensor_read_temp();
}
