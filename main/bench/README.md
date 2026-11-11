# AQUAHARU bench modes

Pilih tepat satu `RUN_BENCH_*` di `main/main.c`. Semua source dan header bench ada di folder ini; `main/CMakeLists.txt` mendaftarkan semuanya. Mode bench keluar lebih awal dari `app_main()` sehingga task normal tidak berjalan.

| Macro | Uji |
| --- | --- |
| `RUN_BENCH_LCD_WIFI` | LCD, pemindaian dan koneksi Wi-Fi |
| `RUN_BENCH_FIREBASE_SIM` | LCD, Wi-Fi, Auth, dan data simulasi Firebase |
| `RUN_BENCH_TEMP_SENSOR` | DS18B20 |
| `RUN_BENCH_TDS_SENSOR` | TDS/EC |
| `RUN_BENCH_PH_SENSOR` | pH |
| `RUN_BENCH_ALL_SENSORS` | Suhu, pH, TDS/EC, LDR DO, float switch, LCD, Wi-Fi, Firebase |
| `RUN_BENCH_RELAY` | GPIO16 dan GPIO4 ON bersama selama 3 menit; input relay lain OFF |

## Uji relay

`RUN_BENCH_ALL_SENSORS` adalah mode aktif saat ini; cara pakainya ada di [BENCH_ALL_SENSORS.md](BENCH_ALL_SENSORS.md). Untuk kembali ke tes relay, aktifkan hanya `RUN_BENCH_RELAY` di `main/main.c`.

`RUN_BENCH_RELAY` **tidak** memulai Wi-Fi, Firebase, pembacaan sensor, kontrol, maupun driver aktuator normal. Berdasarkan pengamatan hardware saat fase ALL OFF, modul ini berperilaku aktif-LOW. Setelah `app_main()` dimulai, keenam input relay lama dan GPIO2 diset HIGH (OFF). Hanya GPIO16 (Peltier + fan) dan GPIO4 (pompa sirkulasi) kemudian diset LOW (ON) **bersamaan selama 3 menit**, lalu semuanya kembali HIGH (OFF). GPIO2 tetap HIGH/OFF sepanjang tes. Tes berjalan sekali dan tidak berulang sampai ESP32 di-reset. Kondisi pin selama reset/boot sebelum `app_main()` tidak dapat dijamin oleh firmware.

Mode `RUN_BENCH_ALL_SENSORS` dan `RUN_BENCH_FIREBASE_SIM` juga sudah diubah agar menahan enam input pada HIGH/OFF. **Jangan aktifkan mode operasi normal dengan catu relay terpasang:** `components/actuators/actuators.c` masih menganggap LOW = OFF dan belum disesuaikan/diperiksa untuk hardware ini.

Untuk uji pertama, **lepas suplai 12 V dari terminal COM/beban** dan biarkan hanya modul relay beserta suplai koil/kontrol yang sudah diverifikasi. Jalankan monitor serial (`idf.py -p COM3 monitor`, sesuaikan port). Tidak perlu mengetik perintah. Pastikan hanya dua kanal tersebut menyala saat LCD menunjukkan `GPIO16+4 ON: 3 MIN` dan semua kanal padam setelah `ALL RELAYS OFF`. Mode bench ini tidak membaca float switch dan tidak menjalankan interlock air rendah; sebelum menghubungkan pompa, pastikan pompa berisi air dan aman dijalankan selama 3 menit.

Jumper VCC–JD-VCC yang dilepas memisahkan catu koil dan catu input pada banyak modul. Pastikan dengan multimeter bahwa **JD-VCC dan VCC sisi IN sama-sama mendapat tegangan yang sesuai dengan modul**, dan GND kontrol punya referensi yang benar. Jangan sambungkan beban 12 V sebelum terlihat hanya kanal yang dipilih menyala selama fase ON dan semua kanal yang terhubung padam selama fase OFF. IN7/IN8 tidak dikendalikan firmware; bila LED-nya masih menyala, periksa jalur input fisiknya.

GPIO2 adalah pin strapping ESP32. Karena Anda menyebut GPIO2 juga menuju input relay, sambungan itu dapat memengaruhi mode flash saat reset; bila ESP32 gagal masuk mode download, lepas sementara sambungan input relay dari GPIO2 saat flashing. Periksa LED/klik dan kontinuitas COM–NO/COM–NC dengan multimeter saat suplai beban masih dilepas. GPIO16 menguji **satu relay**; dua Peltier dan satu fan di sisi kontak tidak bisa diuji terpisah dari GPIO ini. Sebelum menyambung beban, periksa arus kerja dan lonjakan awal **jumlah ketiga beban** terhadap rating kontak DC relay, jalur PCB, kabel, dan catu daya.

Catatan bench lain: [semua sensor](BENCH_ALL_SENSORS.md), [simulasi Firebase/sensor tunggal](BENCH_FIREBASE_SIM.md), dan [pH](BENCH_PH_CALIBRATION.md).
