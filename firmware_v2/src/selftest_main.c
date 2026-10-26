/* selftest_main.c —— 电机自检固件（不需要 PS2 手柄）
 *
 * 目的：完全绕开遥控链路，直接驱动四个轮子。
 *
 * ★ 当前模式：**一次只驱动一个轮子**（每轮 5 秒、间隔 2 秒、循环）
 *   灯亮期间，只有对应的那一个轮子应该转；灯灭期间全部停。
 *   用途：查"哪个轮子根本没接到驱动板 / 接到了错的通道"。
 *
 * 轮序与 robot_mix 一致：wheel[0]=左前 wheel[1]=右前 wheel[2]=左后 wheel[3]=右后
 * 正数 = 前进方向。数值为占空比 ×10（400 = 40%）。
 *
 * 灯：前进中 = 亮；停止时 = 灭。
 * 调试变量 g_step / g_out 可在 ST-Link 的 Watch 里看。
 */
#include "board.h"
#include "board_config.h"
#include "stm32f103_min.h"

volatile uint32_t g_step;
volatile int16_t  g_out[4];

#define STEP_MS 4000u
#define GAP_MS  1500u

/* 一次只驱动一个轮子，每轮 4 秒、间隔 1.5 秒、循环。
 * 2026-10-25 调慢：速度 500→300（30%），单轮 5s→4s，间隔 2s→1.5s。
 * 慢速是为了能看清到底是哪个轮子在转、往哪个方向转。
 *
 * 看点：灯亮的 4 秒里，只有下表对应的那一个轮子该转。
 *   第 1 段 灯亮 → 只在左前转
 *   第 2 段 灯亮 → 只在右前转
 *   第 3 段 灯亮 → 只在左后转
 *   第 4 段 灯亮 → 只在右后转
 * 如果四段转的都是同一个轮子，说明其余三个马达的线接错了通道；
 * 如果某段完全没动静，说明那一路没接到马达。
 *
 * ⚠ 车一的 MOTOR_LB_SIGN / MOTOR_RB_SIGN 都是 -1（后两轮旧线序反），
 *   所以第 3、4 段后轮会朝"后退"方向转，这是正常的，不代表接错。 */
static const int16_t SEQ[][4] = {
    { 500,    0,    0,    0},   /* 1 只驱动左前 LF */
    {   0,  500,    0,    0},   /* 2 只驱动右前 RF */
    {   0,    0,  500,    0},   /* 3 只驱动左后 LB */
    {   0,    0,    0,  500},   /* 4 只驱动右后 RB */
};
#define NSEQ (sizeof(SEQ) / sizeof(SEQ[0]))

static const int16_t STOP4[4] = {0, 0, 0, 0};

/* 在这段时间内持续刷新电机输出。
 * 必须每个控制周期都写寄存器：PWM 通道会保持上次的 CCR，
 * 停写就等于保持上一个速度。 */
static void hold(const int16_t w[4], uint32_t ms)
{
    uint32_t t0 = board_millis();
    for (;;) {
        board_motors_apply(w);
        board_watchdog_feed();
        if ((uint32_t)(board_millis() - t0) >= ms) break;
    }
    board_motors_apply(STOP4);
    board_watchdog_feed();
}

int main(void)
{
    unsigned i;

    if (!board_init()) {
        /* 初始化失败：灯常亮，不进入运动循环。 */
        board_led(true);
        for (;;) { }
    }

    board_motors_apply(STOP4);
    board_led(false);

    for (;;) {
        for (i = 0u; i < (unsigned)NSEQ; ++i) {
            g_step = (uint32_t)(i + 1u);
            g_out[0] = SEQ[i][0];
            g_out[1] = SEQ[i][1];
            g_out[2] = SEQ[i][2];
            g_out[3] = SEQ[i][3];

            board_led(true);                /* 前进中：灯亮 */
            hold(SEQ[i], STEP_MS);

            board_led(false);               /* 停：灯灭 */
            hold(STOP4, GAP_MS);
        }
    }
}
