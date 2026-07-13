# AQUAHARU — Automated Hydroponics Controller

AQUAHARU is a smart, automated water quality monitoring and closed-loop control system designed for home-scale hydroponics. Built on the **ESP32 DevKit V1** using the official **ESP-IDF framework (v6.0.2)** and **FreeRTOS**, the system stabilizes pH, Electrical Conductivity (EC)/TDS, and water temperature using advanced control algorithms and publishes live telemetry to Firebase.

---

## 1. Key Features

*   **Dual-Core FreeRTOS Multitasking:** 
    *   **Core 1 (Control Core):** Executes high-priority, time-critical tasks like sensor acquisition, Fuzzy Logic inference, and actuator control.
    *   **Core 0 (Comms & Display Core):** Manages lower-priority tasks like Wi-Fi connection, Firebase REST client syncing, and local LCD updates.
*   **Fuzzy Logic Controller (Mamdani MIMO):** A C-language Mamdani-type controller with 9 rules that takes pH and TDS as inputs to calculate exact dosing pump run-times simultaneously, resolving chemical coupling.
*   **Sensor Filtering & Compensation:**
    *   *Trimmed Moving Average:* 30 analog samples are taken, sorted, stripped of the 5 highest and 5 lowest outliers, and averaged to filter high-frequency noise.
    *   *Temperature Compensation:* Compensates pH (Nernst-based) and EC/TDS (2%/°C) readings using real-time DS18B20 temperature measurements.
*   **Independent Peltier Hysteresis Cooling:** Controls a Peltier unit and water circulation pump based on water temperature ($26.0^\circ\text{C}$ to $24.0^\circ\text{C}$ hysteresis) running independently from the main fuzzy loop.
*   **Actuator Safety Interlocks:** Sequential (staggered) pump dosing to prevent calcium precipitation, 5-minute mixing lockout delay, maximum run-time limits, and dry-run float switch protection.
*   **On-Boot Hardware Diagnostics:** Supports an optional full hardware verification and individual component looping tests during development before tasks run.
*   **Firebase Integration:** Uploads live stats (PUT, every 60s), appends historical logs (POST, every 60s), and listens to user setpoints via a Server-Sent Events (SSE) stream.

---

## 2. Directory Structure

```text
ESP32-AQUAHARU/
├── CMakeLists.txt              # Top-level CMake project config
├── sdkconfig                   # Kconfig project settings (4MB Flash, Large Partition)
├── README.md                   # Project documentation
└── main/
│   ├── CMakeLists.txt          # Main component registration
│   └── main.c                  # Application entry point, RTOS tasks, and test loops
└── components/
    ├── temp_sensor/            # DS18B20 1-Wire temperature driver
    ├── ph_sensor/              # pH ADC driver, trimmed average filter, temp compensation
    ├── tds_sensor/             # EC/TDS ADC driver, DFRobot cubic conversion, temp compensation
    ├── actuators/              # Relay controllers, Peltier hysteresis, staggered pump logic
    ├── fuzzy_control/          # Mamdani FLC inference engine (MIMO)
    ├── lcd_display/            # I2C LCD 20x4 display driver (using new ESP-IDF v6.0 I2C master API)
    └── firebase_client/        # Wi-Fi station manager, Firebase Auth, and database REST client
```

---

## 3. Hardware Pin Mapping

| Component | Pin | Protocol | Purpose |
| :--- | :---: | :--- | :--- |
| **pH Sensor (PH-4502C)** | **GPIO 36** | Analog (`ADC1_CH0`) | Measure solution acidity |
| **TDS Sensor (Gravity)** | **GPIO 39** | Analog (`ADC1_CH3`) | Measure Electrical Conductivity |
| **Light Sensor (LDR)** | **GPIO 34** | Analog (`ADC1_CH6`) | Monitor ambient light intensity |
| **Float Switch** | **GPIO 35** | Digital Input | Low water level protection (Active Low) |
| **Temp Sensor (DS18B20)** | **GPIO 32** | Digital (1-Wire) | Measure water temperature |
| **Relay 1 (Pump pH Up)** | **GPIO 19** | Digital Output | Dose base reagent |
| **Relay 2 (Pump pH Down)** | **GPIO 18** | Digital Output | Dose acid reagent |
| **Relay 3 (Pump Nutrient A)**| **GPIO 5** | Digital Output | Dose AB Mix Nutrient A |
| **Relay 4 (Pump Nutrient B)**| **GPIO 17** | Digital Output | Dose AB Mix Nutrient B |
| **Relay 5 (Peltier & Fan)** | **GPIO 16** | Digital Output | Active cooling SSR trigger |
| **Relay 6 (Circulation)** | **GPIO 4** | Digital Output | Circulation pump for Peltier block |
| **LCD Display 20x4** | **GPIO 21, 22**| I2C Master | Local visual screen (Addr: `0x3C`, 400kHz) |

---

## 4. Setup, Build, and Flash

Ensure you have the ESP-IDF command-line toolchain installed. The project is configured for **ESP-IDF v6.0.2**.

### Sourcing ESP-IDF (Windows PowerShell)
Run this command in your PowerShell terminal to export the environment variables:
```powershell
. C:\esp\v6.0.2\esp-idf\export.ps1
```

### Set Target Device
```powershell
idf.py set-target esp32
```

### Configure project (Optional)
Ensure Flash size is set to 4MB, partition table is configured as `Single factory app (large)`, and CPU speed is 240MHz:
```powershell
idf.py menuconfig
```

### Build Project
```powershell
idf.py build
```

### Flash and Open Serial Monitor
```powershell
# Replace COM3 with your actual ESP32 COM port
idf.py -p COM3 flash monitor
```
*(To exit the serial monitor, press `Ctrl + ]`)*

---

## 5. Development Diagnostics & Testing

In `main.c`, there are built-in hardware testing routines to verify your wiring before deploying:

1.  **Isolated Component Testing:**
    If you are assembling and debugging components one-by-one, you can call any of the individual test functions inside a simple loop in `app_main()` while commenting out the RTOS tasks:
    *   `test_sensor_temp()`
    *   `test_sensor_ph()`
    *   `test_sensor_tds()`
    *   `test_relay_pump_nut_a()` / `test_relay_peltier()`, etc.
2.  **Full Diagnostic Check on Boot:**
    Define `#define RUN_DIAGNOSTIC_TESTS` in `main.c` to run a comprehensive sequential diagnostic check of all sensors and actuators upon boot before the FreeRTOS tasks start.
