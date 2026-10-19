#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 按键统一为 1=按下；PS2 在线上传输的是低电平有效。 */
enum {
    PS2_SELECT = 1u << 0, PS2_START = 1u << 3,
    PS2_UP = 1u << 4, PS2_RIGHT = 1u << 5,
    PS2_DOWN = 1u << 6, PS2_LEFT = 1u << 7,
    PS2_L2 = 1u << 8, PS2_R2 = 1u << 9,
    PS2_L1 = 1u << 10, PS2_R1 = 1u << 11,
    PS2_TRIANGLE = 1u << 12, PS2_CIRCLE = 1u << 13,
    PS2_CROSS = 1u << 14, PS2_SQUARE = 1u << 15
};
#define ROBOT_LINK_TIMEOUT_MS 100u
#define ROBOT_NEUTRAL_FRAMES 5u
/* 2026-10-19 实测：原值 350/180（=35%/18% 占空比）在 7.4V 上只有 ≈2.6V/1.3V，
 * TT 马达带麦轮几乎带不动整车，表现就是"推杆了但车不走"。
 * 改满 1000=100%（7.4V）、慢速 450=45%，先把车跑起来，手感之后再收敛。 */
#define ROBOT_NORMAL_LIMIT 1000
#define ROBOT_SLOW_LIMIT 450
#define ROBOT_SLEW_PER_SECOND 1000u
#define ROBOT_AXIS_DEADZONE 12

typedef struct {
    bool valid;
    uint8_t mode, lx, ly, rx, ry;
    uint16_t buttons;
} RobotInput;

typedef struct {
    bool connected, armed, emergency_latched, initialized;
    uint8_t neutral_frames;
    uint16_t previous_buttons;
    uint32_t last_good_ms, last_step_ms;
    RobotInput latest;
    /* 顺序固定为左前、右前、左后、右后。范围 -1000..1000。 */
    int16_t wheel[4];
} RobotControl;

bool robot_ps2_decode(const uint8_t *bytes, size_t length, RobotInput *out);
int16_t robot_axis(uint8_t raw);
void robot_mix(int16_t forward, int16_t left, int16_t ccw,
               int16_t limit, int16_t wheel[4]);
void robot_init(RobotControl *state);
/* 新帧才传 input；没有新帧传 NULL。now_ms 为单调递增、可回绕的毫秒计数。
 * local_stop 为本地急停输入。整个控制器没有 5 秒阻塞等延时。
 */
void robot_step(RobotControl *state, const RobotInput *input,
                uint32_t now_ms, bool local_stop);
#endif