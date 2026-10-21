#include "board.h"
#include "board_config.h"
#include "robot_control.h"
#include "ps2_bus.h"
#include "servo_lift.h"
#include "stm32f103_min.h"

/* ST-Link 调试时可在 Watch 观察这两个变量，无需额外调试串口。 */
RobotControl g_robot;
typedef struct {
    uint32_t valid_frames, invalid_frames, last_loop_ms;
    uint8_t ps2_mode;
    bool connected, armed, emergency;
    int16_t wheel[4];
    uint16_t servo_us;
} RobotDebug;
volatile RobotDebug g_debug;

int main(void)
{
    Ps2Service ps2;
    uint32_t last_run;
    robot_init(&g_robot);
    if (!board_init()) {
        /* 初始化失败绝不进入运动循环；若已有看门狗则等待其复位。 */
        for (;;) { }
    }
    ps2_service_init(&ps2,board_millis());
    servo_lift_init(board_millis());
    last_run = board_millis();
    for (;;) {
        uint32_t now = board_millis();
        if ((uint32_t)(now-last_run) >= BOARD_CONTROL_MS) {
            RobotInput input;
            bool valid;
            unsigned i;
            last_run = now;
            valid = ps2_service_poll(&ps2,!g_robot.armed,now,&input);
            if (valid) {
                ++g_debug.valid_frames;
                g_debug.ps2_mode = input.mode;
            } else {
                ++g_debug.invalid_frames;
            }
            /* 事务耗时后重新取时间，不把“开始读手柄”的时间当成接收时间。 */
            now = board_millis();
            robot_step(&g_robot,valid ? &input : NULL,now,board_local_stop());
            board_motors_apply(g_robot.wheel);
            g_debug.last_loop_ms = now;
            g_debug.connected = g_robot.connected;
            g_debug.armed = g_robot.armed;
            g_debug.emergency = g_robot.emergency_latched;
            for (i = 0u; i < 4u; ++i) g_debug.wheel[i] = g_robot.wheel[i];
            /* 挑杆舵机：只有收到有效帧、已启用且未急停时才动；否则保持角度不动。 */
            servo_lift_step(g_robot.connected && g_robot.armed &&
                            !g_robot.emergency_latched &&
                            (g_robot.latest.buttons & PS2_R1) != 0u,
                            g_robot.connected && g_robot.armed &&
                            !g_robot.emergency_latched &&
                            (g_robot.latest.buttons & PS2_R2) != 0u,
                            now);
            g_debug.servo_us = servo_lift_get_us();
            if (g_robot.emergency_latched) board_led(((now/100u)&1u) != 0u);
            else if (g_robot.armed) board_led(true);
            else if (g_robot.connected) board_led(((now/500u)&1u) != 0u);
            else board_led(((now/250u)&1u) != 0u);
            board_watchdog_feed();
        }
        cpu_wait(); /* 每1ms被SysTick唤醒；不使用阻塞式HAL_Delay。 */
    }
}