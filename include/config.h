#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ================================================================
// 1. CẤU HÌNH ĐỘNG CƠ (MOTOR GA25-370 12V 1360RPM)
// ================================================================
#define IN_L1                  26     // Động cơ Trái - Chiều 1
#define IN_L2                  25     // Động cơ Trái - Chiều 2
#define IN_R1                  13     // Động cơ Phải - Chiều 1
#define IN_R2                  27     // Động cơ Phải - Chiều 2

// Cấu hình PWM 20kHz (êm ái, lực kéo mạnh, triệt tiêu tiếng rít)
#define MOTOR_PWM_FREQ         20000 
#define MOTOR_PWM_RESOLUTION  8      // 8-bit: 0 - 255

// Chiều quay động cơ (-1: đã hiệu chuẩn chuẩn xác theo xe để lệnh dương là xe chạy TIẾN)
#define DEFAULT_DIR_MOTOR_L    -1
#define DEFAULT_DIR_MOTOR_R    -1

// ================================================================
// 2. CẤU HÌNH ENCODER PCNT X4 PHẦN CỨNG
// ================================================================
// Chân Encoder Bánh Trái
#define ENC_L_A                21
#define ENC_L_B                22

// Chân Encoder Bánh Phải
#define ENC_R_A                32
#define ENC_R_B                33

// Chiều đếm xung (Đã hiệu chuẩn để khi bánh xe lăn TIẾN thì xung và vận tốc TĂNG DƯƠNG)
#define DEFAULT_DIR_ENC_L      1
#define DEFAULT_DIR_ENC_R      -1

// ================================================================
// 3. THÔNG SỐ CƠ KHÍ & HIỆU CHUẨN ODOMETRY SÀN THỰC TẾ
// ================================================================
// Đường kính lăn hiệu dụng thực tế (đã đo chuẩn xác 50cm trên sàn gạch): 57.6 mm
#define WHEEL_DIAMETER_MM      57.6f
#define WHEEL_CIRCUMFERENCE_MM (PI * WHEEL_DIAMETER_MM) // ~180.956 mm

// Số xung trên 1 vòng bánh xe (Đo thực nghiệm 10 vòng: Trái 1854 xung, Phải 1858 xung)
#define DEFAULT_PPR_WHEEL_L    185.4f
#define DEFAULT_PPR_WHEEL_R    185.8f

// ================================================================
// 4. NÚT BẤM VÀ ĐÈN BÁO TRÊN BOARD
// ================================================================
#define PIN_BUTTON_34          34     // Nút nhấn chân 34 (Active-LOW)
#define PIN_BUTTON_35          35     // Nút nhấn chân 35 (Active-LOW)
#define PIN_LED_BOARD          2      // Đèn LED tích hợp trên board ESP32

// ================================================================
// 5. CẤU HÌNH THANH 16 CẢM BIẾN (MULTIPLEXER CD74HC4067)
// ================================================================
#define PIN_LINE_SIG           36     // ADC1_CH0 (Chân đọc tín hiệu Analog từ MUX)
#define PIN_MUX_S0             17     // Chân chọn kênh S0
#define PIN_MUX_S1             18     // Chân chọn kênh S1
#define PIN_MUX_S2             19     // Chân chọn kênh S2
#define PIN_MUX_S3             23     // Chân chọn kênh S3

#endif // CONFIG_H
