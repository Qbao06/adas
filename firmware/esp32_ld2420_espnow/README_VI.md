# Firmware Arduino IDE: 2 ESP32 + 4 HLK-LD2420 V2

ESP32 A (`ROLE_GATEWAY`) doc hai radar ben trai, nhan hai radar ben phai qua ESP-NOW va xuat JSON 115200 sang Jetson/PC. ESP32 B (`ROLE_REMOTE`) doc hai radar ben phai.

## Noi day

| LD2420 | ESP32 |
|---|---|
| ESP32 A: Radar 1 `OT1`, `RX` | GPIO17, GPIO16 |
| ESP32 A: Radar 2 `OT1`, `RX` | GPIO19, GPIO18 |
| ESP32 B: Radar 1 `OT1`, `RX` | GPIO16, GPIO17 |
| ESP32 B: Radar 2 `OT1`, `RX` | GPIO18, GPIO19 |

Tren ca hai board, `3V3` radar noi nguon 3.3 V va `GND` radar noi chung `GND` ESP32.

`OT1` la UART TX cua radar nen phai vao RX ESP32. `OT2` de trong. Khong cap 5 V vao chan `3V3` cua LD2420. Neu dung nguon 3.3 V rieng, bat buoc noi chung GND.

## Nap bang Arduino IDE

1. Cai **esp32 by Espressif Systems**, mo `esp32_ld2420_espnow.ino`, chon `ESP32 Dev Module`.
2. De `DEVICE_ROLE ROLE_GATEWAY`, nap board A; mo Serial Monitor 115200 va ghi `LOCAL_MAC`.
3. Doi thanh `ROLE_REMOTE`, nap tam board B va ghi `LOCAL_MAC`.
4. Dien hai MAC vao `GATEWAY_MAC`, `REMOTE_MAC`; hai board dung cung `ESPNOW_CHANNEL`.
5. Nap lai board A voi `ROLE_GATEWAY`, board B voi `ROLE_REMOTE`.
6. Cam USB board A vao Jetson/PC. `remote_ok` phai thanh 1; dua nguoi/tay truoc moi radar de kiem tra dung vi tri.

Firmware nhan ca Simple text mode (`ON/OFF`, `Range`) va Energy binary mode cua LD2420. Rut radar thi sau 500 ms radar do thanh `valid:false, stale:true`; tat board B thi sau 800 ms `remote_ok:0`.
