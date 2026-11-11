# 🏎️ XE DÒ LINE 16 MẮT 2026 (CAR CHALLENGE 2026)

Dự án xe đua dò line tốc độ cao 16 cảm biến sử dụng vi điều khiển ESP32, động cơ GA25-370 12V 1360RPM có Encoder hồi tiếp, kiến trúc Dual-Core FreeRTOS kết hợp giao diện điều khiển không dây **Web Bluetooth (Web BLE)** trên GitHub Pages.

---

## ⚡ TÍNH NĂNG NỔI BẬT

1. **Kiến trúc Dual-Core FreeRTOS**:
   - **Core 1 (500Hz / 2ms)**: Chuyên trách điều khiển thời gian thực: quét 16 mắt ADC qua MUX CD74HC4067, thuật toán Trọng tâm liên tục (Smooth Analog Centroid), Non-linear PID bám vạch, đọc xung Encoder phần cứng PCNT X4 và xuất PWM 20kHz ra 2 động cơ.
   - **Core 0**: Chạy ngầm máy chủ Bluetooth Low Energy (BLE) chuẩn Nordic UART Service (NUS), đồng bộ dữ liệu Telemetry về Web và quản lý bộ nhớ Flash NVS.
2. **Chiến lược Quản lý 3 Bản đồ Sa bàn (Map 1, 2, 3)**:
   - Hỗ trợ đổi làn thi đấu đối xứng: **Làn Trái** vs **Làn Phải (Mirror)** chỉ bằng một nút bấm.
   - Cơ chế Phân đoạn Sa bàn (Sector/Waypoint Mapping bằng số xung Encoder `uint32_t ticks`).
   - Tự động ép xe chọn **nhánh Lượn Sóng** thay vì nhánh Răng Cưa góc nhọn, khóa lái phóng qua Vạch Nét Đứt, và phanh dừng chính xác tại rương kho báu đích.
   - Toàn bộ mốc sa bàn được lưu vào bộ nhớ Flash NVS (`Preferences`), thay pin không bao giờ mất dữ liệu.
3. **Giao diện Web BLE Mobile (Single-Page App `index.html`)**:
   - Chạy trực tiếp từ GitHub Pages: Không cần cài App, mở bằng trình duyệt hỗ trợ Web Bluetooth (Chrome trên Android/PC, Bluefy trên iOS).
   - Nút bấm lớn: `CALIB 4S`, `START RACE`, `STOP PHANH GẤP`.
   - Giám sát vị trí vạch, xung encoder, tốc độ PWM theo thời gian thực.
   - Bảng chỉnh sửa mốc xung trực tiếp tại sân thi đấu (Live Tuning) và nút chụp mốc xung tức thời.

---

## 🛠️ SƠ ĐỒ CHÂN PHẦN CỨNG (PINOUT)

| Ngoại vi | Chân ESP32 | Chức năng |
| :--- | :--- | :--- |
| **Động cơ Trái** | GPIO 26, 25 | Cầu H IN_L1, IN_L2 (PWM 20kHz) |
| **Động cơ Phải** | GPIO 13, 27 | Cầu H IN_R1, IN_R2 (PWM 20kHz) |
| **Encoder Trái** | GPIO 21, 22 | PCNT X4 Pha A, B (Bánh Trái) |
| **Encoder Phải** | GPIO 32, 33 | PCNT X4 Pha A, B (Bánh Phải) |
| **Tín hiệu MUX** | GPIO 36 | Chân đọc Analog ADC1_CH0 |
| **Chọn kênh MUX**| GPIO 17, 18, 19, 23 | S0, S1, S2, S3 |
| **Nút bấm trên xe**| GPIO 35 | Nút bấm vật lý đa năng (Calib / Start / Stop) |
| **Đèn LED báo** | GPIO 2 | LED trạng thái tích hợp trên board |

---

## 🚀 HƯỚNG DẪN SỬ DỤNG

### 1. Nạp Firmware vào ESP32
```bash
pio run -t upload
```

### 2. Sử dụng Web BLE trên điện thoại
1. Đẩy file `index.html` lên GitHub Pages của repository:
   `https://<username>.github.io/<repo-name>/`
2. Mở trình duyệt Chrome trên điện thoại, truy cập vào đường link trên.
3. Bấm nút **[⚡ KẾT NỐI BLE]**, chọn thiết bị tên **`RACECAR_16SEN`**.
4. Chọn sa bàn thi đấu: `MAP 1`, `MAP 2`, hoặc `MAP 3` và chọn làn (Trái / Phải).
5. Bấm **[🌀 CALIB 4S]**, lia đầu xe qua vạch đen trong 4 giây.
6. Đặt xe vào vạch xuất phát, bấm **[🚀 XUẤT PHÁT]**!
