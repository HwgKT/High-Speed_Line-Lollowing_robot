# HIGH-SPEED BLE | Line Following Robot

Bảng điều khiển và firmware ESP32 dành cho robot dò line tốc độ cao. Giao diện mang phong cách bàn kỹ thuật điện tử với nền bo mạch tối, đường mạch PCB kiểu Altium được làm dịu để không gây chói, đèn trạng thái nhiều màu và logo vi mạch.

## Mở bảng điều khiển

### [⚡ Mở HIGH-SPEED BLE — phiên bản web mới](https://phanyennhi1412-blip.github.io/Line-following-robot/)

Giao diện được lưu tại [`web/index.html`](web/index.html) và tự động xuất bản lên GitHub Pages khi cập nhật thư mục `web/` trên nhánh `main`. Sau khi push thay đổi, theo dõi workflow **Deploy BLE web to GitHub Pages** trong tab **Actions**; cần workflow chạy thành công để phiên bản mới xuất hiện tại link phía trên. Nếu Pages chưa được cấu hình, vào **Settings → Pages**, chọn **GitHub Actions** tại *Build and deployment*.

Link Pages giữ nguyên khi chỉ cập nhật thiết kế; sau khi workflow deploy thành công, chính link trên sẽ mở giao diện mới.

Dùng Chrome hoặc Edge trên máy tính, mở trang qua **HTTPS** (GitHub Pages) hoặc `localhost`; không mở bằng `file://`. Trang hỗ trợ kết nối **Bluetooth LE** và **USB Serial**.

## Chức năng chính

- **Kết nối BLE hoặc USB** với ESP32; USB Serial chạy ở `115200 baud`.
- **Điều khiển thử động cơ** tiến, lùi, xoay trái/phải bằng nút trên trang hoặc bàn phím; có nút dừng khẩn cấp.
- **Hiển thị trực quan đủ 16 mắt cảm biến** theo dãy một hàng, cùng giá trị analog, trạng thái line, vị trí và sai lệch.
- **Hiệu chuẩn cảm biến** trên nền trắng, vạch đen hoặc bằng cách di chuyển cảm biến qua hai bề mặt.
- **Theo dõi encoder trái/phải** và đặt bộ đếm xung về 0 ngay trong khung Encoder.
- **Chỉnh P/I/D, tốc độ cơ bản và PWM tối đa**, lưu vào RAM ESP32 và bật/tắt chế độ bám line.
- **Nhật ký giao tiếp và gửi lệnh thủ công** để hỗ trợ kiểm tra, chẩn đoán.

### Lưu ý kết nối

- Chọn **Kết nối BLE** rồi chọn thiết bị `ESP32_Robot`, hoặc cắm cáp USB dữ liệu và chọn **Kết nối cáp USB**.
- Với USB Serial, chọn đúng cổng COM; đóng Serial Monitor/PlatformIO Monitor trước khi kết nối. USB cần firmware hỗ trợ Serial ở `115200 baud`.
- Khi dùng USB, trang gửi heartbeat; firmware tự dừng motor nếu mất heartbeat quá 1,5 giây.
- Web Bluetooth trên iOS Safari và Chrome iOS không được hỗ trợ.

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
| **DỪNG ĐỘNG CƠ** | Nhả mọi hướng đang giữ và gửi lệnh dừng ngay. Mất kết nối BLE làm firmware dừng motor. |
| **Calib nền trắng** | Lấy mẫu trung bình 16 cảm biến trong 1 giây khi đặt dãy mắt trên nền trắng. |
| **Calib vạch đen** | Lấy mẫu trung bình 16 cảm biến trong 1 giây khi đặt dãy mắt trên vạch đen. Cần calib cả trắng và đen để xác định line chính xác. |
| **Calib di chuyển** | Trong 6 giây, di chuyển dãy cảm biến qua cả nền trắng và vạch đen; firmware tự ghi nhận min/max làm mốc. |
| **Đọc lại trạng thái** | Yêu cầu ESP32 gửi lại cảm biến, encoder và trạng thái calib. |
| **Đặt xung về 0** | Nút trong khung **Encoder GA25-370**, xóa bộ đếm trái/phải để bắt đầu phép đo mới; không phải hiệu chuẩn tốc độ động cơ. |
| **Lưu vào ESP32** | Gửi P, I, D, tốc độ cơ bản và PWM tối đa vào RAM ESP32; các giá trị có hiệu lực ngay và mất khi tắt nguồn. |
| **Chạy bám line / Dừng bám line** | Chạy bộ điều khiển PID với cấu hình hiện tại. Cần hiệu chuẩn cảm biến trước khi bắt đầu; nếu mất line, firmware dừng động cơ. |
| **Xóa nhật ký** | Chỉ xóa nội dung nhật ký kết nối đang hiển thị trên trang. |

Khung cảm biến hiển thị **MẮT 01–16** theo thứ tự mux `S1`–`S16`. Giá trị analog dùng để kiểm tra tín hiệu; ô `LINE` được xác định từ bit cảm biến firmware gửi. Các nút calib cảm biến nằm trong hộp **Test động cơ**; nhật ký kết nối nằm dưới dãy cảm biến ở cột bên phải. Trong khung **Encoder GA25-370** có nút đặt xung về 0 và số xung tích lũy riêng của động cơ trái/phải.

P/I/D mặc định lần lượt là `18`, `0` và `0.25`; tốc độ cơ bản mặc định là `150` và PWM tối đa là `255` (thang PWM 0–255). Tốc độ cơ bản là mức PWM chạy thẳng, còn PWM tối đa giới hạn đầu ra mỗi động cơ. Trang đọc lại cấu hình hiện tại từ ESP32 khi kết nối hoặc bấm **Đọc lại trạng thái**. Cấu hình chỉ được giữ trong RAM, không lưu qua lần khởi động lại.

## Cấu trúc firmware

`src/main.cpp` chỉ khởi tạo và điều phối các module trong `lib/`. Mỗi module có file `.h` để khai báo giao diện và file `.cpp` chứa phần cài đặt:

- `BLEcontrol`: BLE, xử lý lệnh từ trang web và gửi telemetry.
- `Button`: trạng thái nhấn/nhả và điều khiển thủ công.
- `Encoder`: ngắt đọc xung, đọc và đặt lại bộ đếm.
- `Motor`: thiết lập chân và điều khiển hai động cơ.
- `PID`: thông số PID và điều khiển bám line.
- `Sensor`: đọc 16 cảm biến, hiệu chuẩn và tính vị trí line.

Trang web là nội dung tĩnh trong `web/index.html`, được deploy riêng bằng GitHub Pages; firmware không tạo HTML. Module BLE xử lý BLE UART, đồng thời firmware nhận lệnh và phát cùng giao thức telemetry qua USB Serial ở `115200 baud`.

## Cài đặt và nạp firmware

1. Cài PlatformIO và mở thư mục dự án.
2. Nạp firmware cho `esp32doit-devkit-v1`.
3. Cấp nguồn đúng điện áp cho ESP32 và driver động cơ. Khi thử hướng motor lần đầu, kê bánh xe khỏi mặt đất và nhấn **DỪNG ĐỘNG CƠ** nếu chiều quay hoặc đấu dây không đúng.
4. Mở HIGH-SPEED BLE qua link HTTPS ở trên, kết nối bằng BLE hoặc USB Serial rồi thử từng chức năng.

Lệnh BLE sử dụng Nordic UART Service (NUS):

| Thành phần | UUID | Hướng |
| --- | --- | --- |
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | — |
| RX | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | Web → ESP32 |
| TX | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | ESP32 → Web |

Các nút motor dùng lệnh nhấn/nhả `!B51`/`!B50` (tiến), `!B61`/`!B60` (lùi), `!B71`/`!B70` (trái), `!B81`/`!B80` (phải). Những lệnh chức năng khác: `STOP`, `FOLLOW:1`/`FOLLOW:0`, `P:<giá_trị>`, `I:<giá_trị>`, `D:<giá_trị>`, `V:<tốc_độ>`, `W:<PWM_tối_đa>`, `CAL_WHITE`, `CAL_BLACK`, `CAL_MOVE`, `ENC_RESET`, `SYNC`. Telemetry gồm các gói analog `A<chỉ_số>:` (được chia nhỏ để tương thích giới hạn BLE), `D:`, `POS:`, `ERR:`, `ENC_L:`, `ENC_R:`, `CAL:`, trạng thái bám line `FOLLOW:` và cấu hình `P:`, `I:`, `D:`, `V:`, `W:`.
