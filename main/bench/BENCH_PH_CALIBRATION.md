# Bench pH PH-4502C (ESP32 DevKit V1)

Mode `RUN_BENCH_PH_SENSOR` di `main/main.c` hanya menjalankan LCD 20x4 dan pembacaan ADC1_CH0/GPIO36. Wi-Fi, Firebase, sensor lain, task kontrol, serta driver relay tidak dijalankan. Lepaskan sambungan relay selama bench. Jangan aktifkan mode operasi normal sebelum rangkaian dan kalibrasi pH diperiksa.

## Rentang ADC dan pilihan tanpa mengubah PCB utama

Board pH hijau diberi 5 V. Pemilik sudah mengukur bahwa PO pada varian ini dibatasi maksimum 3,3 V. Namun pengujian pertama menunjukkan sekitar 2,85 V di buffer 6,86 dan 3,13 V di buffer 4,01. Datasheet ESP32 menyatakan akurasi ADC pada attenuasi tertinggi memburuk di atas raw sekitar 3000 (sekitar 2,45 V). Kedua titik pengujian melebihi batas itu, jadi jangan masukkan hasil lama sebagai kalibrasi final.

**Periksa tegangan fisik sebelum mengubah rangkaian.** Pada PH-4502C dengan dua trimpot, trimpot dekat BNC adalah offset PO; prosedur short pusat BNC ke selubungnya (tanpa probe) lalu menyetel PO ke 2,5 V adalah penyetelan offset standar. Pemilik sudah melakukannya, sehingga jangan memutar trimpot lagi untuk mengatasi angka ADC. Itu bukan kalibrasi dua titik probe. Jika 2,5 V tadi ditentukan dari pembacaan ADC ESP32, ulangi verifikasi dengan multimeter pada PO terhadap GND modul; ADC ESP32 bisa tidak akurat pada tegangan tinggi. Ukur juga PO dengan multimeter saat probe stabil dalam buffer 6,86 dan 4,01, lalu bandingkan dengan serial GPIO36. Jangan menyentuh trimpot ambang DO. Putuskan catu saat melepas/memasang probe atau short BNC, dan jangan pernah men-short PO/VCC/GPIO36.

Jika multimeter menunjukkan PO memang di atas sekitar 2,45 V pada kedua buffer, masalah rentang akurasi ADC nyata dan penurunan tegangan sebelum GPIO36 diperlukan untuk pembacaan yang lebih andal. Jika PO pada multimeter lebih rendah tetapi serial GPIO36 tetap tinggi, periksa sambungan, ground, dan pengukuran ADC sebelum mengubah hardware. Menggeser offset PO jauh dari 2,5 V bisa dipakai sebagai eksperimen terbatas, tetapi mengubah seluruh kurva dan bukan langkah pertama setelah penyetelan BNC yang benar.

**Hasil bench terbaru:** setelah trimpot offset diputar hingga batasnya, PO pada buffer 4,01 tetap sekitar 3,13 V menurut multimeter; pembacaan ESP32 pada buffer 6,86 sekitar 2,75 V. Jadi trimpot tidak mampu membawa kedua titik ke rentang akurasi ADC. Jangan masukkan dua nilai ini sebagai kalibrasi final dan jangan aktifkan dosing otomatis berdasarkan pH tersebut. Filter yang sudah ada mengambil 30 sampel per pembacaan, membuang 5 tertinggi dan 5 terendah, lalu merata-ratakan 20 sisanya. Filter ini dapat mengurangi derau, tetapi tidak dapat memulihkan informasi yang hilang akibat ADC mendekati jenuh atau memperbaiki akurasi di luar rentangnya. Untuk bench sementara, amati tegangan dan perubahan relatifnya saja; simpan log raw ADC dan tegangan dari kedua buffer setelah stabil. Jika kelak akurasi pH diperlukan untuk kontrol, perlu perubahan jalur analog atau ADC eksternal dan kalibrasi ulang.

**Bila tegangan PO terkonfirmasi terlalu tinggi:** buat adaptor kecil di jalur PO menuju konektor PCB, tanpa membuat ulang PCB utama. Dua resistor 10 kΩ dapat membagi sinyal menjadi separuh:

```text
PO modul -- 10 kΩ --+-- GPIO36 (VP, ADC1_CH0)
                   |
                 10 kΩ
                   |
GND modul ---------+-- GND ESP32
VCC modul ------------ 5 V sesuai catu board
```

Matikan catu sebelum mengubah kabel. Pastikan titik tengah kedua resistor masuk ke GPIO36. Dari log sebelumnya, tegangan baru diperkirakan sekitar 1,43 V dan 1,57 V, tetapi harus dibuktikan lewat bacaan baru. Pin DO/TO tidak digunakan. Pembagi pada kabel/adaptor perlu tetap terpasang setelah kalibrasi; kalau modul dan PCB terhubung langsung tanpa kabel yang bisa disisipi, perubahan ini memerlukan akses ke jalur PO, dan letak perubahan harus ditentukan dari layout/foto PCB.

**Firmware bench yang baru perlu di-build dan di-flash sekali** untuk menampilkan perkiraan pH pada LCD dan serial. Tidak ada perubahan mode operasi normal. Bila kelak adaptor pembagi dipasang, kedua konstanta tegangan bench harus diganti memakai hasil pengukuran GPIO36 setelah pembagi.

## Pengambilan dua titik

1. Siapkan dua wadah kecil bersih untuk larutan pH 6,86 dan 4,01. Ikuti petunjuk pada kemasan jika larutan dibuat dari serbuk. Jangan mencelupkan probe langsung ke botol stok. Usahakan kedua larutan mendekati 25 °C karena angka pada kemasan berlaku pada 25 °C.
2. Bilas ujung probe dengan air demineral/RO, lalu tepuk perlahan dengan tisu bebas serat; jangan digosok. Masukkan probe ke larutan **pH 6,86** hingga bagian sensitif terendam. Hindari menyentuh dasar/dinding wadah. Tunggu sekitar 1–2 menit atau sampai nilai PO pada serial stabil.
3. Catat 5–10 baris terakhir `GPIO36=... V` beserta `ADC=...` dan `pH~=`. Gunakan rata-rata tegangan **GPIO36** yang stabil sebagai `PH_BENCH_VOLTAGE_686_V`.
4. Bilas dan tepuk probe lagi. Pindahkan ke larutan **pH 4,01** dalam wadah terpisah. Tunggu stabil, catat 5–10 baris dan rata-ratakan tegangan GPIO36 untuk `PH_BENCH_VOLTAGE_401_V`.
5. Bilas dan cek kembali larutan 6,86. Bila tegangan jauh berubah atau ADC mendekati 0/4095, periksa kabel, ground, probe, dan kesegaran larutan sebelum menghitung kalibrasi.

Masukkan kedua tegangan **GPIO36 yang benar-benar terukur dengan posisi trimpot yang sama** ke dua definisi di bagian atas `main/bench/bench_ph_sensor.c`:

```c
#define PH_BENCH_VOLTAGE_686_V    /* hasil buffer 6.86 */
#define PH_BENCH_VOLTAGE_401_V    /* hasil buffer 4.01 */
```

Mode bench menghitung `pH~ = 6.86 + (V - V_6.86) * (4.01 - 6.86) / (V_4.01 - V_6.86)`. Build sementara memakai `2.736 V` untuk buffer 6,86 (rata-rata pembacaan ESP32 terbaru) dan `3.130 V` untuk buffer 4,01 (bacaan multimeter, belum diverifikasi lagi sebagai bacaan ESP32 setelah perubahan trimpot). Karena kedua titik di luar rentang akurasi ADC dan titik 4,01 mendekati jenuh, tanda `~` berarti **perkiraan bench**, bukan pH terkalibrasi atau layak untuk dosing otomatis. Jangan pindahkan angka ini ke operasi normal.

## Tanda yang diharapkan

- LCD: `AQUAHARU pH BENCH`, ADC raw, tegangan GPIO36, serta `pH~... bench`.
- Serial: `bench_ph ready` dan satu `Sample` sekitar tiap 2 detik, masing-masing berisi `ADC`, `GPIO36`, dan `pH~` dari satu batch filter yang sama.
- Setiap batch terdiri dari 30 bacaan ADC berjarak 30 ms. Lima nilai tertinggi dan lima terendah dibuang; 20 nilai tengah dirata-ratakan. Satu bacaan ADC gagal membatalkan seluruh batch.

Walau bench tidak lagi menampilkan peringatan tiap sampel, raw ADC di atas 3000 masih berada di luar rentang akurasi yang ditentukan Espressif. Jika raw mendekati 4095, nilai `pH~` dapat terdistorsi oleh kejenuhan ADC. Dua titik 4,01 dan 6,86 terutama memvalidasi rentang di antaranya; pembacaan jauh di luar rentang itu perlu buffer tambahan. Tidak ada angka pH air keran yang dianggap benar hanya dari perkiraan bench.

Setelah selesai, bilas probe dan simpan ujung elektroda dalam larutan penyimpanan yang sesuai untuk probe pH, mengikuti petunjuk produsen probe. Jangan menyimpan elektroda kaca dalam air demineral.

