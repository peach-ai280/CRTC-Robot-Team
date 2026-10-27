/* servo_test_main.c —— 只动舵机，四个轮子一律不动
 *
 * 用途：手柄/PS2 还不通的时候，单独确认"舵机这一路到底好不好"。
 *
 * 它在 1500 / 1800 / 1200 三个位置之间来回走，每个位置停 2 秒。
 * 看现象就能分出三种情况：
 *   ① 铲子跟着走三个位置、走完停住    → 舵机好的，之前"一直复原"是因为
 *                                        没有手柄指令，固件让它保持初始位
 *   ② 铲子走到某个位置顶死、嗡嗡响    → 那个角度超出机械行程，要把
 *                                        BOARD_SERVO_MIN_US / MAX_US 收窄
 *   ③ 铲子一直朝一个方向转不停        → 这是连续旋转舵机（360° 版），
 *                                        不是 180° 位置舵机，得换货或改用法
 *
 * 灯语：舵机在这一段要动 = 灯亮；回中段 = 灯灭。
 *
 * ★ 轮子全程 0，车不会跑，可以放在桌上测。
 */
#include "board.h"
#include "board_config.h"
#include "servo_lift.h"
#include "stm32f103_min.h"

volatile uint32_t g_stage;
volatile int16_t  g_wheel[4];
volatile uint16_t g_servo_now;

typedef struct {
    uint16_t us;
    uint16_t ms;
} Step;

static const Step STEPS[] = {
    { 1500, 2000 },   /* 1 上电初始位（固件默认就停在这里） */
    { 1800, 2000 },   /* 2 向一端 */
    { 1500, 1000 },   /* 3 回中 */
    { 1200, 2000 },   /* 4 向另一端 */
    { 1500, 1000 },   /* 5 回中 */
};
#define NSTEP (sizeof(STEPS) / sizeof(STEPS[0]))

static const int16_t ZERO[4] = { 0, 0, 0, 0 };

int main(void)
{
    unsigned i;

    if (!board_init()) {
        board_led(true);
        for (;;) { }
    }
    servo_lift_init(board_millis());
    board_motors_apply(ZERO);
    board_led(false);

    for (;;) {
        for (i = 0u; i < (unsigned)NSTEP; ++i) {
            uint32_t t0;

            g_stage = (uint32_t)(i + 1u);
            servo_lift_set_us(STEPS[i].us);
            g_servo_now = servo_lift_get_us();
            /* 1800 / 1200 这两段算"在动"，1500 的两段算"停着" */
            board_led(STEPS[i].us != 1500u);

            t0 = board_millis();
            while ((uint32_t)(board_millis() - t0) < STEPS[i].ms) {
                board_motors_apply(ZERO);   /* 轮子一直保持停 */
                board_watchdog_feed();
            }
        }
        board_led(false);
    }
}
