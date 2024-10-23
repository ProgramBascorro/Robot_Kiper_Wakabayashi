# Robot Kiper dengan Robotis Bioloid GP
Repository ini berisi kode program untuk mengoperasikan robot kiper berbasis Robotis Bioloid GP. Robot ini dirancang untuk mendeteksi bola menggunakan visi komputer dan melakukan gerakan responsif untuk menghalangi bola. Selain itu, robot ini juga dilengkapi dengan komunikasi sensor gyro untuk meningkatkan stabilitas dan akurasi gerakan.

---

# Deskripsi
Robot kiper ini menggunakan dua program utama:
1. Program Vision dan Motion
   Bertugas untuk mendeteksi keberadaan bola menggunakan kamera dan menentukan gerakan yang harus dilakukan oleh robot untuk menghalangi bola tersebut.

2. Program Komunikasi Sensor Gyro
   Berfungsi untuk membaca data sensor gyro yang dipasang pada robot, memberikan informasi tambahan tentang orientasi dan keseimbangan robot selama gerakan berlangsung.

---

# Persyaratan
Sebelum menjalankan program, pastikan Anda telah menginstal pustaka berikut:
- Python 3.x
- OpenCV (untuk vision processing)
- PySerial (untuk komunikasi serial)

---

# Konfigurasi
Anda mungkin perlu mengonfigurasi port serial dan parameter komunikasi yang digunakan oleh sensor gyro dan robot. Pastikan pengaturan tersebut sesuai dengan konfigurasi perangkat keras yang Anda gunakan.
