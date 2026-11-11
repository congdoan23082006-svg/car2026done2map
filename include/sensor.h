#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =================== CẤU HÌNH CHÂN (PIN CONFIG) ===================
#define PIN_LINE_SIG          36      // Chân đọc Analog (ADC1_CH0)
#define PIN_MUX_A             17      // S0 (Bit 0)
#define PIN_MUX_B             18      // S1 (Bit 1)
#define PIN_MUX_C             19      // S2 (Bit 2)
#define PIN_MUX_D             23      // S3 (Bit 3)
#define PIN_CALIB             34      // Nút nhấn hiệu chuẩn (Calibration)

// =================== THÔNG SỐ CẢM BIẾN ===================
#define N_SENSORS             16      // Tổng số mắt cảm biến
#define SENSOR_START_IDX      0       // Bắt đầu từ mắt 0
#define SENSOR_END_IDX        15      // Kết thúc ở mắt 15
#define ADC_SAMPLES           4       // Số lần lấy mẫu ADC trung bình để lọc nhiễu
#define MIN_CALIBRATION_DELTA 150     // Chênh lệch tối thiểu giữa Min và Max (tránh nhiễu)
#define BUTTON_DEBOUNCE_MS    50      // Thời gian chống rung phím (ms)

#define NO_LINE               -1      // Mã trả về khi cả 16 mắt đều không nhìn thấy vạch đen

// =================== CÁC BIẾN TOÀN CỤC ===================
extern uint16_t sensorRaw[N_SENSORS];
extern uint16_t minValues[N_SENSORS];
extern uint16_t maxValues[N_SENSORS];
extern uint16_t thresholds[N_SENSORS];
extern bool blackIsHigher[N_SENSORS];
extern bool isCalibrating;
extern bool calibrationReady;

// =================== CÁC HÀM XỬ LÝ (FUNCTIONS) ===================

void sensor_init(void);
void sensor_mux_select(uint8_t channel);
uint16_t sensor_read_mux(uint8_t channel);
void sensor_read_all(void);

void sensor_start_sweep_calibration(uint32_t duration_ms);
void sensor_update_calibration_loop(void);
void sensor_finish_sweep_calibration(void);
bool sensor_is_calibrating(void);

bool sensor_is_button_pressed(void);
void sensor_handle_calibration_step(void);

uint16_t sensor_get_calibrated(uint8_t index);
bool sensor_is_black(uint8_t index);
uint16_t sensor_get_line_bits(void);

void sensor_set_polarity(bool black_is_higher);
bool sensor_get_polarity(void);

/**
 * @brief Tính vị trí tim vạch bằng thuật toán ANALOG CENTROID (0..15000, Setpoint: 7500)
 * @return 0 đến 15000 nếu thấy line, hoặc NO_LINE (-1) nếu mất line hoàn toàn
 */
int16_t sensor_get_line_position_analog(void);
int16_t sensor_get_line_position_binary(void);
int16_t sensor_get_line_position(void);

// =================== CÁC HÀM NHẬN DIỆN ĐỊA HÌNH VẠCH ĐẶC BIỆT ===================

uint8_t sensor_get_black_count(void);
bool sensor_is_crossline(void);
bool sensor_is_out_of_line(void);
bool sensor_is_90_left(void);
bool sensor_is_90_right(void);

void sensor_print_states(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_H
