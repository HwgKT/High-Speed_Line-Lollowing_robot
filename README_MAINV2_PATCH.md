# BLE debug telemetry patch cho HwgKT/doline17s

Gói này sinh ra các thay đổi trên checkout local. Nó không push GitHub, không commit và bắt buộc đang ở nhánh `mainv2` trước khi sửa.

## 1. Tạo nhánh mới từ main

Trong terminal ở thư mục repo:

```bash
git fetch origin
git switch -c mainv2 origin/main
```

Nếu `mainv2` đã tồn tại ở local, dùng `git switch mainv2` thay vì tạo lại.

## 2. Chạy patch

Giải nén `apply_mainv2_patch.py` vào thư mục gốc repo (cùng cấp `index.html`), sau đó:

```bash
python apply_mainv2_patch.py
pio run
git diff --check
git diff --stat
```

Script xác nhận branch là `mainv2` và xác nhận tất cả vị trí thay đổi tồn tại chính xác một lần trước khi ghi bất kỳ file nào. Nếu anchor không khớp, script dừng mà không sửa file.

## 3. Những file được sửa

- `index.html`: thêm ADC raw / ADC calibrated ngay dưới từng mắt logic 0/1; thêm PWM trái/phải và encoder trái/phải; ghép dữ liệu BLE nhị phân trước khi cập nhật UI; tắt polling `P` khi đang kết nối BLE vì ESP chủ động gửi debug.
- `src/ble_control.cpp`: gói telemetry 56 byte, chia thành bốn BLE notification.
- `include/ble_control.h`: khai báo hàm gửi telemetry mới.
- `src/main.cpp`: khởi tạo bộ đếm encoder và gửi snapshot debug định kỳ.

## 4. Gói debug

- PWM trái/phải: signed 16-bit little-endian, phạm vi -255…255.
- Encoder trái/phải: unsigned 32-bit little-endian, bộ đếm tích lũy.
- 16 ADC raw: 12-bit mỗi mắt (0–4095), tổng 24 byte.
- 16 ADC sau calibration: 10-bit mỗi mắt (0–1000), tổng 20 byte.

Payload logic có 56 byte. Mỗi notification có header 3 byte `[0xA7, sequence, fragmentIndex]` cùng tối đa 16 byte payload, tức tối đa 19 byte/notification phù hợp MTU mặc định 23. Trang web chỉ cập nhật khi nhận đủ bốn mảnh thuộc cùng sequence.

ESP gửi snapshot 150 ms/lần, phù hợp chu kỳ debug trong `src/main.cpp`. Không còn phải polling 16 trạng thái qua lệnh `P` trên BLE.

## 5. Lưu ý encoder

Patch mặc định dùng GPIO32 làm xung bánh trái và GPIO33 làm xung bánh phải, đếm cạnh lên. Bộ đếm này đo số xung tích lũy, không xác định chiều quay. Nếu encoder của bạn dùng hai kênh A/B cho từng bánh, cần khai báo thêm hai GPIO và giải mã quadrature.

Không nối đầu ra encoder 5 V trực tiếp vào ESP32; tín hiệu phải tương thích logic 3,3 V hoặc dùng chuyển mức.

## 6. Push đúng nhánh

Sau khi `pio run` thành công và đã kiểm tra diff:

```bash
git add index.html src/main.cpp src/ble_control.cpp include/ble_control.h
git commit -m "Add compact BLE sensor and encoder debug telemetry"
git push -u origin mainv2
```

Không commit hoặc push thay đổi này lên `main`.
