# High-Speed Line Following Robot

Firmware ESP32 và giao diện BLE để hiệu chuẩn, kiểm tra động cơ, đọc cảm biến line và theo dõi encoder GA25-370.

## Mở giao diện BLE

**[Mở LineLab BLE trên GitHub Pages](https://phanyennhi1412-blip.github.io/Line-following-robot/)**

Giao diện tĩnh nằm trong [`web/index.html`](web/index.html), được tự động xuất bản lên GitHub Pages bằng workflow khi có thay đổi trong `web/` hoặc workflow deploy trên nhánh `main`. Nếu trang chưa mở được sau lần đầu, vào **Settings → Pages**, chọn **GitHub Actions** ở mục *Build and deployment*, sau đó chờ workflow `Deploy BLE web to GitHub Pages` chạy thành công trong tab **Actions**.

Web Bluetooth cần ngữ cảnh bảo mật **HTTPS** và trình duyệt hỗ trợ Web Bluetooth:

- Dùng Chrome hoặc Edge trên máy tính; Chrome trên Android cũng được hỗ trợ.
- Không mở `index.html` trực tiếp bằng `file://`, không dùng địa chỉ HTTP thường hoặc bản xem trước Markdown của GitHub.
- iOS Safari và Chrome trên iOS hiện không hỗ trợ Web Bluetooth.
- Bật robot, bật Bluetooth, chọn **Kết nối ESP32**, rồi chọn thiết bị `ESP32_Robot`.

## Phần cứng được đối chiếu từ schematic

Schematic [SCH_Following_line_ga25-370_2026-10-14 (1).json](<hardware/SCH_Following_line_ga25-370_2026-10-14 (1).json>) có:

- **16 mắt cảm biến quang**, nối các kênh `S1`–`S16` vào mux analog 16 kênh `U24 (HP4067)`. Đây là 16 cảm biến riêng; không phải 8 mắt được nhân đôi.
- Hai động cơ GA25-370, mỗi động cơ có encoder hai kênh (A/B), tổng cộng **4 tín hiệu encoder**.

Các chân ESP32 dùng trong firmware:

| Chức năng | Chân ESP32 |
| --- | --- |
| Tín hiệu analog mux | GPIO36 |
| Chọn kênh mux S0–S3 | GPIO17, GPIO18, GPIO19, GPIO23 |
| Điều khiển cầu H: IN1–IN4 | GPIO26, GPIO25, GPIO27, GPIO13 |
| Encoder trái A/B | GPIO21, GPIO22 |
| Encoder phải A/B | GPIO32, GPIO33 |

Firmware phát giá trị analog 12-bit (0–4095) và chuỗi bit phát hiện line của đủ 16 kênh. Số encoder là bộ đếm xung quadrature có dấu, không phải RPM; muốn đổi sang RPM hoặc quãng đường cần xác nhận số xung/vòng và tỉ số truyền của đúng phiên bản GA25-370 đang dùng.

## Hướng dẫn nút trên web

| Nút / điều khiển | Tác dụng |
| --- | --- |
| **Kết nối ESP32 / Ngắt kết nối** | Mở hộp chọn thiết bị Bluetooth, kết nối hoặc ngắt kết nối. Khi kết nối, web gửi `SYNC` để yêu cầu trạng thái hiện tại. |
| **▲ Tiến** | Chạy hai động cơ tiến trong lúc nhấn giữ; nhả nút để dừng hướng này. |
| **▼ Lùi** | Chạy hai động cơ lùi trong lúc nhấn giữ; nhả nút để dừng hướng này. |
| **◀ Xoay trái / ▶ Xoay phải** | Điều khiển hai động cơ ngược chiều để xoay tại chỗ trong lúc nhấn giữ; nhả nút để dừng. |
| **DỪNG ĐỘNG CƠ** | Nhả mọi hướng đang giữ và gửi lệnh dừng ngay. Mất kết nối BLE cũng làm firmware dừng motor. |
| **Calib nền trắng** | Lấy mẫu trung bình 16 cảm biến trong 1 giây khi đặt dãy mắt trên nền trắng. |
| **Calib vạch đen** | Lấy mẫu trung bình 16 cảm biến trong 1 giây khi đặt dãy mắt trên vạch đen. Cần calib cả trắng và đen để xác định line chính xác. |
| **Calib di chuyển** | Trong 6 giây, di chuyển dãy cảm biến qua cả nền trắng và vạch đen; firmware tự ghi nhận min/max làm mốc. |
| **Đọc lại trạng thái** | Yêu cầu ESP32 gửi lại cảm biến, encoder và trạng thái calib. |
| **Đặt xung về 0** | Xóa bộ đếm encoder trái/phải để bắt đầu phép đo mới. |
| **Lưu vào ESP32** | Gửi P, I, D, tốc độ cơ bản và PWM tối đa vào RAM ESP32; các giá trị có hiệu lực ngay và mất khi tắt nguồn. |
| **Chạy bám line / Dừng bám line** | Chạy bộ điều khiển PID với cấu hình hiện tại. Cần hiệu chuẩn cảm biến trước khi bắt đầu; nếu mất line, firmware dừng động cơ. |
| **Xóa nhật ký** | Chỉ xóa nội dung nhật ký BLE đang hiển thị trên trang. |

Khung cảm biến hiển thị **MẮT 01–16** theo thứ tự mux `S1`–`S16`. Giá trị analog dùng để kiểm tra tín hiệu; ô `LINE` được xác định từ bit cảm biến firmware gửi. Khung encoder hiển thị xung tích lũy riêng cho động cơ trái và phải.

P/I/D mặc định lần lượt là `18`, `0` và `0.25`; tốc độ cơ bản mặc định là `150` và PWM tối đa là `255` (thang PWM 0–255). Tốc độ cơ bản là mức PWM chạy thẳng, còn PWM tối đa giới hạn đầu ra mỗi động cơ. Trang đọc lại cấu hình hiện tại từ ESP32 khi kết nối hoặc bấm **Đọc lại trạng thái**. Cấu hình chỉ được giữ trong RAM, không lưu qua lần khởi động lại.

## Cài đặt và nạp firmware

1. Cài PlatformIO và mở thư mục dự án.
2. Nạp firmware cho `esp32doit-devkit-v1`.
3. Cấp nguồn đúng điện áp cho ESP32 và driver động cơ. Khi thử hướng motor lần đầu, kê bánh xe khỏi mặt đất và nhấn **DỪNG ĐỘNG CƠ** nếu chiều quay hoặc đấu dây không đúng.
4. Mở web BLE qua link HTTPS ở trên, kết nối rồi thử từng chức năng.

Lệnh BLE sử dụng Nordic UART Service (NUS):

| Thành phần | UUID | Hướng |
| --- | --- | --- |
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | — |
| RX | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | Web → ESP32 |
| TX | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | ESP32 → Web |

Các nút motor dùng lệnh nhấn/nhả `!B51`/`!B50` (tiến), `!B61`/`!B60` (lùi), `!B71`/`!B70` (trái), `!B81`/`!B80` (phải). Những lệnh chức năng khác: `STOP`, `FOLLOW:1`/`FOLLOW:0`, `P:<giá_trị>`, `I:<giá_trị>`, `D:<giá_trị>`, `V:<tốc_độ>`, `W:<PWM_tối_đa>`, `CAL_WHITE`, `CAL_BLACK`, `CAL_MOVE`, `ENC_RESET`, `SYNC`. Telemetry gồm các gói analog `A<chỉ_số>:` (được chia nhỏ để tương thích giới hạn BLE), `D:`, `POS:`, `ERR:`, `ENC_L:`, `ENC_R:`, `CAL:`, trạng thái bám line `FOLLOW:` và cấu hình `P:`, `I:`, `D:`, `V:`, `W:`.
