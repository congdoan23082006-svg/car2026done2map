#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_MAPS          3
#define MAX_WAYPOINTS     16

// Các hành vi cưỡng bức tại từng mốc sa bàn
typedef enum {
    ACTION_NORMAL        = 0, // Bám line PID bình thường
    ACTION_FORCE_LEFT    = 1, // Bỏ mắt phải, ép rẽ ôm nhánh Trái (Lượn Sóng)
    ACTION_FORCE_RIGHT   = 2, // Bỏ mắt trái, ép rẽ ôm nhánh Phải (Lượn Sóng)
    ACTION_LOCK_STRAIGHT = 3, // Khóa cân bằng 2 bánh chạy thẳng (Vượt vạch đứt)
    ACTION_BRAKE_STOP    = 4  // Phanh dừng chốt đích
} WaypointAction_t;

// Cấu trúc 1 Mốc phân đoạn (Waypoint)
typedef struct {
    uint32_t start_ticks;    // Mốc xung encoder BẮT ĐẦU phân đoạn
    int16_t  target_speed;   // Tốc độ nền mong muốn (baseSpeed: 0 - 255)
    uint8_t  action;         // WaypointAction_t
} Waypoint_t;

// Cấu trúc 1 Bản đồ sa bàn
typedef struct {
    uint8_t count;
    Waypoint_t wps[MAX_WAYPOINTS];
} MapProfile_t;

// =================== CÁC HÀM QUẢN LÝ (API) ===================

void map_manager_init(void);
void map_load_defaults(void);
void map_save_to_flash(void);
bool map_load_from_flash(void);
void map_sort_waypoints(uint8_t map_idx);

// Lựa chọn map và làn thi đấu
void map_select(uint8_t map_idx); // 0: Map 1, 1: Map 2, 2: Map 3
uint8_t map_get_selected(void);

void map_set_right_lane(bool is_right); // true: Làn Phải (Mirror), false: Làn Trái
bool map_is_right_lane(void);

// Thao tác sửa mốc trực tiếp từ Web BLE
bool map_set_waypoint(uint8_t map_idx, uint8_t wp_idx, uint32_t ticks, int16_t speed, uint8_t action);
void map_clear_waypoints(uint8_t map_idx);
uint8_t map_get_waypoint_count(uint8_t map_idx);
const Waypoint_t* map_get_waypoint(uint8_t map_idx, uint8_t wp_idx);

// Hàm truy vấn thời gian thực cho bộ điều khiển PID (Core 1)
void map_get_current_action(uint32_t current_ticks, int16_t *out_speed, uint8_t *out_action, uint8_t *out_wp_idx);

#ifdef __cplusplus
}
#endif

#endif // MAP_MANAGER_H
