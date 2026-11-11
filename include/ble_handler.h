#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAR_MODE_STOP  = 0,
    CAR_MODE_SCAN  = 1,  // Lượt 1: Bò chậm lấy mốc sa bàn (baseSpeed 70)
    CAR_MODE_RACE  = 2,  // Lượt 2: Bung tốc độ thi đấu theo bảng mốc xung
    CAR_MODE_ALIGN = 3   // Tự căn tâm khóa vạch 0 độ tại chỗ sau khi Calib (Zero-Base Speed)
} CarRunMode_t;

// Khởi tạo BLE Server trên Core 0
void ble_init(const char *device_name);

// Quản lý chế độ chạy
void car_set_run_mode(CarRunMode_t mode);
CarRunMode_t car_get_run_mode(void);

void car_set_scan_speed(int16_t speed);
int16_t car_get_scan_speed(void);

// Kiểm tra trạng thái kết nối
bool ble_is_connected(void);

// Bắn dữ liệu Telemetry về Web BLE
void ble_send_telemetry(int32_t ticks_l, int32_t ticks_r, float line_pos, 
                        int16_t pwm_l, int16_t pwm_r, 
                        uint8_t map_idx, uint8_t wp_idx, uint8_t car_state,
                        uint16_t sensor_mask);

// Bắn chuỗi thông báo chung hoặc danh sách Map về Web
void ble_send_string(const char *str);
void ble_send_map_data(uint8_t map_idx);

#ifdef __cplusplus
}
#endif

#endif // BLE_HANDLER_H
