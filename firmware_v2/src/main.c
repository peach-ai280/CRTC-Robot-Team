#include "board.h"
#include "board_config.h"
#include "robot_control.h"
#include "ps2_bus.h"
#include "servo_lift.h"
#include "stm32f103_min.h"

/* ST-Link 调试时可在 Watch 观察这两个变量，无需额外调试串口。 */
RobotControl g_robot;
/* 提成全局是为了能用 OpenOCD 直接读 stage / fail_count：
 * PS2 不通时先看这两个数，就能分清是"卡在配置事务里"还是"轮询了但没有回帧"。 */
Ps2Service g_ps2;
typedef struct {
    uint32_t valid_frames, invalid_frames, last_loop_ms;
    uint8_t ps2_mode;
    bool connected, armed, emergency;
    int16_t wheel[4];
} RobotDebug;
volatile RobotDebug g_debug;

int main(void)
{
    uint32_t last_run;
    robot_init(&g_robot);
    if (!board_init()) {
        /* 初始化失败绝不进入运动循环；若已有看门狗则等待其复位。 */
        for (;;) { }
    }
    servo_lift_init(board_millis()); /* BOARD_SERVO_ENABLE=0 时是空函数 */
    ps2_service_init(&g_ps2,board_millis());
    last_run = board_millis();
    for (;;) {
        uint32_t now = board_millis();
        if ((uint32_t)(now-last_run) >= BOARD_CONTROL_MS) {
            RobotInput input;
            bool valid;
            unsigned i;
            last_run = now;
            valid = ps2_service_poll(&g_ps2,!g_robot.armed,now,&input);
            if (valid) {
                ++g_debug.valid_frames;
                g_debug.ps2_mode = input.mode;
            } else {
                ++g_debug.invalid_frames;
            }
            /* 事务耗时后重新取时间，不把“开始读手柄”的时间当成接收时间。 */
            now = board_millis();
            robot_step(&g_robot,valid ? &input : NULL,now,board_local_stop());
            /* 舵机：按住 R1 升起、按住 R2 下降，松开保持当前角度。
             * 只在收到有效帧时动；BOARD_SERVO_ENABLE=0（车一默认）时是空函数，
             * PA3 一个电平都不输出，不影响原有行为。 */
            servo_lift_step(valid && (input.buttons & PS2_R1) != 0u,
                            valid && (input.buttons & PS2_R2) != 0u,
                            now);
            board_motors_apply(g_robot.wheel);
            g_debug.last_loop_ms = now;
            g_debug.connected = g_robot.connected;
            g_debug.armed = g_robot.armed;
            g_debug.emergency = g_robot.emergency_latched;
            for (i = 0u; i < 4u; ++i) g_debug.wheel[i] = g_robot.wheel[i];
            if (g_robot.emergency_latched) board_led(((now/100u)&1u) != 0u);
            else if (g_robot.armed) board_led(true);
            else if (g_robot.connected) board_led(((now/500u)&1u) != 0u);
            else board_led(true); /* 未连手柄时常亮：烧完就有反应，一眼看出程序在跑 */
            board_watchdog_feed();
        }
        cpu_wait(); /* 每1ms被SysTick唤醒；不使用阻塞式HAL_Delay。 */
    }
}