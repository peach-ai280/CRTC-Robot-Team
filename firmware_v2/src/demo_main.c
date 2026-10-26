/* demo_main.c —— 整车自动演示（完全不需要 PS2 手柄）
 *
 * 用途：手边没有能用的手柄时，用来验证"整台车能不能正常动"。
 *       四个轮子按麦轮混控跑各种动作，舵机同步来回摆。
 *
 * 段表驱动：想增删动作，改 STAGES 一行即可。
 *
 * 灯语：这段要动（有电机或舵机输出）= 亮；纯停顿 = 灭。
 *
 * 轮序与 robot_mix 一致：wheel[0]=左前 wheel[1]=右前 wheel[2]=左后 wheel[3]=右后
 * ⚠ 车一的 MOTOR_LB/RB_SIGN 都是 -1，后两轮物理上会朝反方向转，属正常。
 */
#include "board.h"
#include "board_config.h"
#include "robot_control.h"
#include "servo_lift.h"
#include "stm32f103_min.h"

volatile uint32_t g_stage;
volatile int16_t  g_wheel[4];

/* 速度上限，-1000..1000。500 = 50%，7.4V 上约 3.7V 出力。 */
#define DEMO_LIMIT 500

typedef struct {
    int16_t  f;         /* 前后   -1000..1000 */
    int16_t  l;         /* 平移   -1000..1000 */
    int16_t  w;         /* 转向   -1000..1000 */
    uint16_t servo_us;  /* 舵机脉宽，0 = 这一拍不动舵机 */
    uint16_t ms;
} Stage;

static const Stage STAGES[] = {
    /*  f      l      w     servo   ms   */
    {  500,    0,    0,      0,  3000 },   /* 1 前进，四轮同向 */
    {    0,    0,    0,      0,   600 },   /*   停 */
    { -500,    0,    0,      0,  3000 },   /* 2 后退 */
    {    0,    0,    0,      0,   600 },   /*   停 */
    {    0,  500,    0,      0,  2500 },   /* 3 向左平移 */
    {    0,    0,    0,      0,   600 },   /*   停 */
    {    0, -500,    0,      0,  2500 },   /* 4 向右平移 */
    {    0,    0,    0,      0,   600 },   /*   停 */
    {    0,    0,  500,      0,  2500 },   /* 5 原地转（左） */
    {    0,    0,    0,      0,   600 },   /*   停 */
    {    0,    0, -500,      0,  2500 },   /* 6 原地转（右） */
    {    0,    0,    0,      0,   600 },   /*   停 */
    {    0,    0,    0,   1800,  1500 },   /* 7 舵机转到一端 */
    {    0,    0,    0,   1200,  1500 },   /* 8 舵机转到另一端 */
    {    0,    0,    0,   1500,  1500 },   /* 9 舵机回中 */
};
#define NSTAGE (sizeof(STAGES) / sizeof(STAGES[0]))

static const int16_t ZERO[4] = { 0, 0, 0, 0 };

int main(void)
{
    unsigned i;

    if (!board_init()) {
        board_led(true);
        for (;;) { }
    }
    servo_lift_init(board_millis());   /* BOARD_SERVO_ENABLE=1 时才有脉冲 */
    board_motors_apply(ZERO);
    board_led(false);

    for (;;) {
        for (i = 0u; i < (unsigned)NSTAGE; ++i) {
            int16_t wheel[4];
            uint32_t t0;
            bool active = (STAGES[i].f != 0) || (STAGES[i].l != 0) ||
                          (STAGES[i].w != 0) || (STAGES[i].servo_us != 0);

            g_stage = (uint32_t)(i + 1u);

            robot_mix(STAGES[i].f, STAGES[i].l, STAGES[i].w, 1000, wheel);
            if (STAGES[i].servo_us != 0u) servo_lift_set_us(STAGES[i].servo_us);

            g_wheel[0] = wheel[0];
            g_wheel[1] = wheel[1];
            g_wheel[2] = wheel[2];
            g_wheel[3] = wheel[3];

            board_led(active);

            t0 = board_millis();
            while ((uint32_t)(board_millis() - t0) < STAGES[i].ms) {
                board_motors_apply(wheel);
                board_watchdog_feed();
            }
            board_motors_apply(ZERO);
            board_led(false);
            board_watchdog_feed();
        }
    }
}
