# Firmware Arduino Nano — Robot TRR

Firmware cho Nano (ATmega328), giao tiếp với app digital twin (`../digital_twin/app.py`) qua Serial 115200.

## Nạp

```bash
pio run -t upload
```

Mở cửa sổ theo dõi Serial:

```bash
pio device monitor
```

## Đấu dây (shield CNC V4 cho Nano, driver A4988, vi bước 1/16)

| Trục | Động cơ | STEP | DIR | Quy đổi |
|---|---|---|---|---|
| Z | stepper 4237 + vít me 8 mm/vòng | D7 | D4 | 400 bước/mm |
| J1 | stepper 4238, đai 20→60 | D5 | D2 | 26.67 bước/độ |
| J2 | stepper 4239, đai 20→36 | D6 | D3 | 16 bước/độ |
| EN chung | | D8 (LOW = bật) | | |
| J3 | servo MG995 | D10 | | servo = 90° + j3 |
| Kẹp | servo SG90 | D11 | | servo = 120° − g |

Hai servo nên cấp nguồn 5–6 V riêng (MG995 kéo dòng lớn), nối chung GND với Nano.

## Giao thức

Mỗi lệnh là một dòng text. Đơn vị mm và độ, tính so với home.

**PC → Nano**

| Lệnh | Ý nghĩa |
|---|---|
| `J z j1 j2 j3 g` | Setpoint khớp. Twin gửi khoảng 20 Hz. Nano không trả lời, chỉ báo `ERR` khi lệnh sai. |
| `H` | Về home |
| `S` | Dừng êm |
| `?` | Báo vị trí ngay |
| `O` | Lấy vị trí hiện tại làm home |
| `E 0` / `E 1` | Tắt / bật driver |
| `P` | Ping, Nano trả `OK` |

**Nano → PC:** `P z j1 j2 j3 g`, khoảng 20 Hz.
- z, j1, j2 đếm từ số bước đã phát.
- j3 và g là góc đang ra lệnh, vì servo không có cảm biến phản hồi.

## Cách chạy

- **Phát xung:** ngắt Timer2 chạy 25 kHz. Mỗi trục có bộ tích luỹ pha nên phát tối đa khoảng 12 000 bước/s. Servo dùng Timer1 nên không đụng nhau.
- **Bám vị trí:** vòng điều khiển 500 Hz, có giới hạn vận tốc và gia tốc. Càng gần đích càng chậm, theo `v = √(2·a·d)`, nên không bị giật.
- **Giới hạn mềm** khớp với `robot_config.json`: Z −12…150 mm, J1 ±135°, J2 ±150°, J3 ±90°, kẹp 0…135°.
- **Tốc độ tối đa:** Z 25 mm/s (= 10 000 bước/s), J1 90°/s, J2 120°/s.

## Lưu ý khi chạy lần đầu

1. **Chưa có công tắc hành trình.** Vị trí lúc cấp nguồn hoặc reset được coi là home. Trước khi cấp nguồn, đưa robot về tư thế home: cánh tay duỗi thẳng theo −Y, Z ở vị trí home. Nếu quên, đưa tay về home rồi gửi `O`.
2. **Kiểm tra chiều quay:** trong tab "Khớp" của app, kéo nhẹ từng khớp. Trục nào quay ngược thì đổi `invert` của trục đó trong mảng `CFG` (đầu `src/main.cpp`).
3. **Tay kẹp:** chỉnh `GRIP_SERVO_OPEN` sao cho g = 0 thì ngón thẳng, cước vừa căng. Nếu servo phải quay ngược chiều để cuộn cước thì đổi dấu `GRIP_SIGN`.
4. Đổi vi bước hoặc tỉ số truyền thì sửa đồng thời `STEPS_PER_REV`/`CFG` ở đây và mục `drive` trong `robot_config.json`.
