#ifndef PS2_BUS_H
#define PS2_BUS_H
#include <stdbool.h>
#include <stdint.h>
#include "robot_control.h"
typedef struct {
    uint8_t stage;
    uint16_t fail_count;   /* 连续读不到有效帧的次数，用于自愈判断 */
    uint32_t due_ms, next_config_ms;
} Ps2Service;
extern volatile uint8_t g_ps2_last_bytes[21];
extern volatile uint8_t g_ps2_last_length;
void ps2_service_init(Ps2Service *service, uint32_t now_ms);
bool ps2_service_poll(Ps2Service *service, bool allow_config,
                      uint32_t now_ms, RobotInput *out);
#endif