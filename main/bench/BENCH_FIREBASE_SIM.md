# Bench LCD Wi-Fi Firebase dengan data simulasi

Mode ini hanya untuk bench ESP32 DevKit V1, ESP-IDF 6.0.2. Tidak membaca sensor,
menjalankan fuzzy/control/aktuator, diagnostik relay, atau membuka SSE konfigurasi.
Enam GPIO relay aktif-HIGH (19, 18, 5, 17, 16, 4) ditahan LOW sebelum task bench
dibuat. Tidak ada perintah HIGH. Perubahan `water_low` hanya untuk tampilan.
Jaminan ini berlaku untuk kode setelah boot; keadaan listrik saat reset belum
diverifikasi oleh build. Status pompa di web bukan feedback fisik relay.

## Fakta lokal dan batas verifikasi

- `main.c` dan dashboard menggunakan `ph`, `ec_ms_cm`, `tds_ppm`,
  `temperature_c`, `light_v`, `water_low`, `timestamp_ms`. Live juga mempunyai
  `pump_status`: `ph_up`, `ph_down`, `nutrient_a`, `nutrient_b`, `peltier`,
  `circulation_pump`. Angka dikirim sebagai JSON number, flag sebagai boolean.
- Path firmware adalah `/aquaharu/v1/devices/device_01`. Dashboard memilih
  perangkat dari membership akun, lalu membaca path kanonis yang sama.
- Kartu membaca `live`. Grafik membaca 50 entri terakhir `history`, bukan live.
- Online berasal dari `heartbeat/last_seen_ms` numerik dengan umur <=180 detik.
  Dashboard memeriksa ulang status setiap 30 detik dan saat heartbeat berubah.
- REST mengirim `{".sv":"timestamp"}` untuk `timestamp_ms` dan `last_seen_ms`.
  Firebase menyimpannya sebagai Unix milidetik numerik. Ini bukan uptime ESP32.
- Rules lokal mensyaratkan akun terautentikasi dengan role `owner` pada
  `/aquaharu/v1/user_devices/{uid}/device_01/role` untuk menulis live/history/
  heartbeat. Role operator hanya dapat mengubah config. Dashboard juga
  mensyaratkan email terverifikasi untuk membuka halaman.
- Ekspor JSON lokal di folder Website Maliq tidak memiliki `aquaharu/v1`.
  Ekspor tersebut bukan bukti kondisi database yang terpasang sekarang.
  Dokumen rancangan dan CLAUDE.md masih memuat path/field lama dan rencana fitur.
  Keberadaan tulisan safety/config di dokumen tidak membuktikan implementasinya.
- Tidak dilakukan login ke Firebase, pemeriksaan membership live, deployment,
  import, flash, atau pengujian hardware oleh perubahan ini. Role akun dan rules
  terpasang harus dikonfirmasi saat pengujian pengguna; HTTP PUT 200 diperlukan.

## Memilih mode dan nilai

Di bagian atas `main/main.c`:

```c
/* #define RUN_BENCH_LCD_WIFI */
/* #define RUN_BENCH_FIREBASE_SIM */
#define RUN_BENCH_TEMP_SENSOR
```

Aktifkan maksimal satu define bench. Kombinasi bench dengan diagnostik ditolak
saat kompilasi. Untuk operasi normal, comment semua define bench lalu build/flash
lagi. `app_main()` return setelah membuat task bench; jalur normal tidak berjalan.

## Bench suhu DS18B20

Mode saat ini memilih `RUN_BENCH_TEMP_SENSOR` di `main.c`. Tes dan LCD ada di
`main/bench/bench_temp_sensor.c`; driver DS18B20 yang sama dipakai oleh firmware normal.
Pembacaan diperbarui berkala sekitar 2 detik. Resolusi default 12-bit memakai
waktu konversi 800 ms. Hasil gagal ditampilkan sebagai error, bukan suhu.

| DS18B20 | ESP32 |
|---|---|
| Data | GPIO32 |
| VCC | 3V3 |
| GND | GND |
| Pull-up | 4.7 kΩ antara Data dan 3V3 |

Pastikan sensor Anda memang DS18B20 dan isolasi probe/kabel yang dicelupkan
utuh. Gunakan air suhu ruang terlebih dahulu. Firmware hanya membaca sensor.

**Lepas kabel input relay dari ESP32 selama bench suhu.** Bench ini tidak
menyentuh pin relay. Polaritas trigger dan mapping pin modul 8-channel Anda
belum diverifikasi; jangan anggap LOW berarti OFF. Jangan celupkan ujung kabel
terbuka atau sambungan elektronik ke air.

Setelah build sukses, tutup monitor serial yang lama lalu flash (log sebelumnya
memakai COM3; sesuaikan bila port berubah):

```powershell
Set-Location 'D:\Kuliah\CDP\ESP32-AQUAHARU'
. 'C:\Espressif\tools\Microsoft.v6.0.2.PowerShell_profile.ps1'
idf.py --no-ccache -p COM3 flash monitor
```

LCD diharapkan menunjukkan `DS18B20 GPIO32` dan suhu berkala. Serial diharapkan
menunjukkan `DS18B20 detected` lalu `Sample n: xx.xx C`. Bila tidak terdeteksi,
LCD/log meminta pemeriksaan GPIO32, ground, dan resistor 4.7 kΩ. Jika LCD gagal,
serial tetap mencatat pembacaan sensor.

Bench menggunakan satu selector di `main.c` dan satu file implementasi per
subsistem, misalnya `bench_firebase_sim.c` dan `bench_temp_sensor.c`. Pola ini
menjaga setup dan wiring tiap tes tetap terpisah serta mudah ditinjau. Tambahkan
define baru untuk tes pH, TDS, atau relay dan aktifkan hanya satu mode sekaligus.

## Bench TDS analog

Mode saat ini memilih `RUN_BENCH_TDS_SENSOR` di `main.c`, implementasi ada di
`main/bench/bench_tds_sensor.c`. Driver lokal menyatakan target DFRobot Gravity TDS:
output analog ke GPIO39 / ADC1_CH3. Sambungkan `A/OUT` ke GPIO39, GND bersama,
dan catu module 3V3 untuk tes awal; jangan biarkan output melebihi batas ADC ESP32.
Celupkan hanya probe TDS, bukan board pemancar atau konektor. Probe tetap jauh
dari dinding/bawah wadah, bilas sebelum pindah larutan, dan jangan dipakai di air
lebih dari 55 °C. Lepaskan input modul relay saat uji sensor.

Bench melaporkan tegangan ADC dan TDS hasil rumus dengan faktor kalibrasi awal
1.000 pada referensi suhu 25 °C. TDS Gravity tidak punya sensor suhu internal;
nilai ini adalah asumsi untuk tes terisolasi, bukan pembacaan temperatur air.
Pembacaan memakai median 30 sampel dengan jarak 40 ms, lalu menghitung tegangan
dan ppm dari median ADC yang sama. Setiap batch memakan sekitar 1,16 detik;
LCD diperbarui tiap 1,5 detik. Median membantu menolak spike sesaat, tetapi
tidak mengkalibrasi ADC atau menjamin akurasi ppm.

DS18B20 digital biasanya tidak butuh kalibrasi awal. Bandingkan dengan
termometer acuan bila perlu verifikasi. Sebaliknya, kalibrasi satu titik TDS
disarankan sebelum memakai angka ppm untuk kontrol. Untuk DFRobot SEN0244,
DFRobot merekomendasikan larutan standar 707 ppm (1413 µS/cm) pada 25 °C.
Bilas probe dengan air suling, keringkan tetesan, masukkan probe ke larutan tanpa
menyentuh dinding wadah, aduk perlahan, dan tunggu sampai bacaan stabil.

Driver firmware mempunyai `g_tds_calibration_factor` yang default-nya `1.0`.
Untuk kalibrasi sederhana: `faktor_baru = ppm_label_larutan / ppm_terbaca`.
Contoh rumus ini bukan nilai kalibrasi sebelum ada hasil bacaan nyata. Saat
larutan dan temperatur referensi tersebut, ubah faktor di bagian parameter
driver lalu build ulang. Nilai faktor sekarang tidak disimpan ke NVS; perubahan
akan hilang bila diganti source/default pada build berikutnya. Kita belum
menambahkan input serial atau menu kalibrasi interaktif.

TDS dapat dibaca dulu tanpa larutan standar untuk memeriksa respons, namun
jangan anggap ppm itu terkalibrasi. Sensor suhu yang berhasil diuji dapat
diintegrasikan ke mode pengukuran berikutnya agar kompensasi suhu mengikuti
suhu air; bench ini sengaja memakai referensi tetap 25 °C agar hanya satu sensor
yang diuji. Sumber: [spesifikasi SEN0244](https://wiki.dfrobot.com/sen0244) dan
[panduan kalibrasi DFRobot](https://wiki.dfrobot.com/sen0244/docs/20304).

Edit `SIM_SAMPLES` di bagian atas `main/bench/bench_firebase_sim.c`:

| pH | EC mS/cm | TDS ppm | Suhu C | light_v | water_low |
|---|---|---|---|---|---|
| 6.2 | 1.4 | 700 | 24.0 | 1.8 | false |
| 6.7 | 1.8 | 900 | 25.5 | 2.0 | false |
| 7.1 | 2.6 | 1300 | 27.0 | 2.2 | true |

Pola berulang, target interval awal-ke-awal 3000 ms. Gunakan nilai finite yang
wajar. HTTPS bersifat blocking: bila transaksi melebihi 3 detik, serial mencatat
overrun lalu menunggu satu periode baru tanpa burst. Ini bukan jaminan latensi
internet 3 detik. LCD menampilkan sampel yang dicoba dan hasil PUT terakhir.

## History dan pembersihan

Default `BENCH_SIM_HISTORY_ENABLED` adalah `0`: hanya PUT live dan heartbeat.
Kartu berubah, tetapi grafik history tetap menampilkan data lama/kosong.

Untuk menguji grafik, ubah flag ke `1`, build lalu flash sendiri. Maksimum
`BENCH_SIM_HISTORY_MAX_ATTEMPTS` (default 30) percobaan POST per boot. Sesudah
batas tersebut, live/heartbeat tetap berjalan. Reboot memulai budget baru.
Percobaan gagal tetap dihitung karena server mungkin menyimpan data meskipun
respons tidak sampai. Setiap respons sukses mencetak `History key (for selective
cleanup)` di serial. Bentuk history sama dengan produksi, tanpa field marker.

Simpan key dan waktu mulai/selesai bench. Setelah berhenti, hapus HANYA child key
history yang terkonfirmasi berasal dari bench melalui Firebase Console pada
`/aquaharu/v1/devices/device_01/history`. Bila respons hilang sehingga key tidak
tercatat, cocokkan timestamp dan nilai siklus dalam rentang waktu tes; jangan
menghapus data yang tidak dapat dipastikan. Jangan menghapus seluruh history
atau mengimpor ulang ekspor penuh. Panduan ini tidak menjalankan penghapusan.
Live menimpa nilai terakhir; operasi normal berikutnya akan menggantinya.

Pada cadence ideal: 20 PUT live + 20 PUT heartbeat per menit (2400 PUT/jam).
Ini 20 kali frekuensi interval produksi 60 detik. Jika tanpa pembatas, history
3 detik akan menambah 1200 record/jam; flag di sini membatasi 30 percobaan/boot.
PUT tidak menumbuhkan node live, tetapi respons REST, TLS, dan broadcast listener
tetap memakai bandwidth. Banyak tab dashboard menambah unduhan. Periksa Usage
Firebase; tidak ada klaim kuota proyek saat ini masih tersedia.
Referensi: https://firebase.google.com/docs/database/usage/billing

## Build dan flash oleh pengguna

```powershell
Set-Location 'D:\Kuliah\CDP\ESP32-AQUAHARU'
. 'C:\Espressif\tools\Microsoft.v6.0.2.PowerShell_profile.ps1'
idf.py build
idf.py -p COM3 flash monitor
```

Pada sesi build ini, launcher ccache gagal dengan `CreateProcess failed`.
Build berhasil memakai `idf.py --no-ccache build`. Bila masalah yang sama muncul,
gunakan `idf.py --no-ccache -p COM3 flash monitor`; opsi ini tidak mengubah
sdkconfig atau memerlukan pengaturan Git global.

Verifikasi lokal 7 Oktober 2026: build ESP-IDF 6.0.2 berhasil menghasilkan
`build/Aquaharu_Fuzzy.bin`, ukuran `0xf2840` byte, partisi aplikasi 35% kosong.
Varian history aktif lulus kompilasi terpisah tanpa mengubah binary utama.
Pemeriksaan format melalui snprintf host lulus untuk ketiga sampel live/history:
field, tipe JSON, timestamp server, seluruh pump_status false, dan penolakan
buffer kecil. Log build berada di `build/bench-build.log` (folder build diabaikan
Git). Ada pemberitahuan environment tentang kepemilikan Git SDK, Kconfig SDK,
dan pembuatan gdbinit; build tetap selesai. Ini bukan pengujian serial/hardware
atau bukti izin database live.

Gunakan AP 2.4 GHz dengan sinyal kuat. Jangan membagikan header kredensial atau
log debug HTTP mentah. Pertimbangkan memindahkan kredensial hardcoded ke file
lokal yang di-gitignore dalam pekerjaan terpisah; tidak disalin ke panduan ini.

Yang diharapkan, belum merupakan hasil uji hardware:

- Serial: `SIMULATION`, koneksi Wi-Fi/Auth, `SIM[0]`/`SIM[1]`/`SIM[2]`,
  `live PUT HTTP 200 result=ESP_OK; heartbeat PUT HTTP 200 result=ESP_OK`.
  Tidak ada task sensor, control, actuator atau SSE yang dimulai.
- LCD 20x4, contoh setelah sampel pertama sukses:

```text
SIM pH:6.2 EC:1.40
T:24.0C H2O:OK
WiFi:UP HB:200
FB PUT:200 OK
```

- Dashboard: login dengan akun prototype terverifikasi, pilih `device_01`.
  Kartu berputar mengikuti array; pH 6.7/7.1 dan suhu 27 memberi peringatan,
  water-low muncul pada sampel ketiga; semua pompa OFF; timestamp baru dan
  status Online dari heartbeat. Grafik bertambah hanya bila history diaktifkan.
- Setelah ESP32 berhenti, live tetap berisi data terakhir; badge menjadi Offline
  setelah heartbeat melewati 180 detik, ditambah hingga 30 detik refresh UI.

HTTP 0 berarti belum ada respons HTTP (atau request dilewati saat tidak ready).
HTTP 401/403 mengarah ke masalah autentikasi/otorisasi; cocokkan UID hasil Auth
dengan membership owner dan rules yang terpasang. HTTP 200 dengan FAIL berarti
respons belum dianggap lengkap/JSON valid; jangan mengklaim upload sukses hanya
berdasarkan angka status. Kegagalan init ditampilkan di LCD dan berhenti upload:
perbaiki koneksi/Auth lalu reset, bukan memanggil init berulang.

Port COM3 mengikuti log pengujian terbaru pengguna; pilih port aktual bila berubah.

Helper REST kini menangkap body melalui HTTP_EVENT_ON_DATA selama perform(),
membatasi respons 8192 byte, membebaskan buffer/client pada setiap jalur hasil,
dan menggunakan empat root CA publik Google melalui `cert_pem`. Log tidak mencetak
URL bertoken maupun body Auth. API lama tetap ada untuk upload produksi; logic
aktuator dan safety tidak diubah. Reconnect Wi-Fi lama tetap dibatasi 5 percobaan;
refresh token lama tetap 50 menit. Ketahanan koneksi jangka panjang belum diuji.

## Tindak lanjut kegagalan TLS dari log pengguna

Foto dan log pengguna membuktikan LCD terbaca, Wi-Fi memperoleh IP, dan bench
berhenti saat koneksi TLS ke Firebase Auth. Pesan utamanya adalah
`No matching trusted root certificate found`, `-0x3000`, dan
`REST POST HTTP 0 (ESP_ERR_HTTP_CONNECT)`. Ini belum merupakan penolakan akun
atau rules; tidak ada respons HTTP dari Firebase pada percobaan tersebut.

REST diganti dari callback certificate bundle menjadi verifikasi PEM standar
dengan root GTS R1-R4 yang disertakan di SDK. Rincian sumber sertifikat dan alasan
ada di `components/firebase_client/TLS_CERTIFICATES.md`. Verifikasi nama host
tetap aktif; sdkconfig, safety, dan flag history tidak berubah.

Build perbaikan TLS berhasil: binary `0xe2ca0` byte, partisi aplikasi 40% kosong;
log `build/bench-tls-build.log`. Ukuran ini menggantikan hasil build awal di atas.
Handshake TLS 1.2 dan 1.3 dari komputer dengan hanya empat root tersebut lulus untuk host
Auth dan RTDB, tanpa HTTP atau kredensial. Tetap diperlukan uji ulang ESP32.
Setelah flash, cari Auth `REST POST HTTP 200`, `Firebase client ready`, lalu
live/heartbeat HTTP 200. Log `Certificate validated` dari callback bundle lama
tidak wajib muncul lagi. Jika TLS masih gagal, kirim log mulai Auth sampai error;
jangan mengirim isi header kredensial. Jika Auth sudah HTTP 400 atau PUT 401/403,
itu merupakan tahap kegagalan berbeda yang perlu diperiksa dari bukti baru.

