# Digital Twin — Robot TRR

App PyQt5 + PyVista mô phỏng robot SCARA trong `robot.FCStd`.

## Cài đặt & chạy

```bash
pip install -r requirements.txt
python app.py            # mở app
python app.py --demo     # mở app và chạy luôn chương trình mẫu gắp–đặt vật
```

## Cấu trúc

| File | Vai trò |
|---|---|
| `export_meshes.py` | Chạy bằng `freecadcmd export_meshes.py` để xuất lại mesh khi sửa CAD (toạ độ global, tư thế home) |
| `meshes/` | STL từng chi tiết + `parts.json` |
| `robot_config.json` | Kích thước, giới hạn và vận tốc khớp, tỉ số truyền, nhóm chi tiết, màu |
| `kinematics.py` | Động học thuận/ngược, quy đổi sang số bước động cơ |
| `motion.py` | Quỹ đạo: jog, PTP (quintic), LIN (đường thẳng Descartes) |
| `device_link.py` | Serial tới robot thật và robot ảo (SIM) |
| `gripper_sim.py` | Mô phỏng tay kẹp: ngón cong theo góc servo SG90, tiếp xúc và kẹp/nhả vật hình trụ |
| `collision.py` | Kiểm tra va chạm giữa tay kẹp với thân robot và sàn (bật/tắt bằng nút **Chặn va chạm**) |
| `app.py` | Giao diện |

## Mô hình động học (suy ra từ assembly)

- **Z**: carriage `link_1` chạy trên 2 trục `truc8`, kéo bởi vít me (`vitme`) qua stepper 4237.
- **J1**: trục quay tại (0, −35). Stepper 4238 kéo `puly_20` → `puly_60` (tỉ số 3:1).
- **J2**: trục quay tại (0, −135). Stepper 4239 kéo `puly_20` → `puly_36` (tỉ số 1.8:1).
- **J3**: trục ra của MG995 tại (0.1, −213.2), hướng xuống.
- **Kẹp**: tay kẹp mềm `soft_gripper/SoftGripper_SG90.FCStd` (3 ngón, dẫn bằng dây cước, servo SG90) gắn lên horn tròn 25T của MG995 qua đế `Mount_MG995`. TCP nằm ở mặt phẳng đầu ngón (z ≈ 12.7 mm ở home), nên Z ≥ −12 mm để ngón không chạm sàn.

Nếu sửa CAD trong lúc FreeCAD đang mở `robot`, có thể chạy `export_meshes.py` ngay trong FreeCAD (Macro hoặc Python console) mà không cần lưu file trước.

Mọi joint trong `robot.FCStd` đều là Fixed. Các khớp chuyển động ở trên được suy ra từ hình học và cấu hình trong `robot_config.json`.

## Giao thức Serial

Giao thức dạng text, mỗi lệnh một dòng. Đơn vị là mm và độ, tính so với home.

- **PC → robot**:
  - `J z j1 j2 j3 g`: setpoint, gửi khoảng 20 Hz khi twin chạy
  - `H`: về home
  - `S`: dừng
  - `?`: hỏi vị trí
- **robot → PC**:
  - `P z j1 j2 j3 g`: vị trí thực tế. Hiển thị dưới dạng bóng xanh, kèm bảng sai lệch.

Số bước tương ứng = góc/độ dịch × tỉ số truyền × (200 × 16) / (360° hoặc bước vít me). App hiển thị sẵn giá trị này.
