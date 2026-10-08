/* ============================================================================
 * sim_kinematics.c —— 运动学离线验证（在 ARM 模拟器里跑真实代码）
 *
 * 它编译进的是**真实的 mecanum.c / motor.c**，不是重写一遍公式。
 * 跑完把四轮解算值、真实写进 TIM4->CCRx 的 PWM 比较值、以及用正解反算
 * 回来的速度方向存在一个数组里，由 tools/sim/run_sim.py 读出来做判定。
 *
 * 判定的核心是"归一化限幅有没有保住速度矢量的方向"：
 *   给一个斜向指令 -> 四轮解算 -> 归一化 -> 用正解 Mecanum_Fwd() 反算速度
 *   -> 反算出来的方向角必须等于指令方向角。
 *   如果用的是"逐轮截断"而不是"整体等比缩放"，这个角度就会偏，
 *   表现就是你说的"明明推 45°，车却走成 30°"。
 * ==========================================================================*/

#include "main.h"
#include "board.h"
#include "config.h"
#include "mecanum.h"
#include "motor.h"
#include <math.h>

/* bsp.c 里定义的句柄太多了（还有串口、ADC），离线验证只需要电机这一个，
   所以这里自己定义一份 htim4，不去牵扯整套 BSP。 */
TIM_HandleTypeDef htim4;

/* 结果区：[用例][0..2]=指令 / [3..6]=四轮解算 / [7]=指令方向角(度)
 *         [8]=加增益后的期望角 / [9]=正解反算角 / [10..13]=四个轮的实际 PWM 比较值 */
#define NCASE   8
#define NW      14
volatile float    g_out[NCASE][NW];
volatile uint32_t g_done = 0;

static float deg(float r) { return r * 57.29577951f; }

int main(void)
{
    /* --- 复刻 BSP_TIM4_Init：10kHz，PSC=0 ARR=7199 --- */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 7199;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim4);

    Chassis_Init();                       /* 里面会调 Motor_Init -> 启动 4 路 PWM */

    const float cases[NCASE][3] = {
        { 1.0f,  0.0f,  0.0f},            /* 0 纯前进 */
        { 0.0f,  1.0f,  0.0f},            /* 1 纯左移 */
        { 0.0f,  0.0f,  1.0f},            /* 2 纯逆时针 */
        { 0.80f, 0.0f,  0.80f},           /* 3 前进 + 自转（测归一化） */
        { 0.80f,-0.80f, 0.0f},            /* 4 斜走 45° */
        { 0.50f, 0.50f, 0.50f},           /* 5 三轴全给 */
        { 0.60f, 0.0f, -0.60f},           /* 6 前进 + 反向自转 */
        {-1.0f,  0.0f,  0.0f},            /* 7 纯后退 */
    };

    for (int c = 0; c < NCASE; c++)
    {
        float vx = cases[c][0], vy = cases[c][1], wz = cases[c][2];

        /* 先归零，让斜坡从上一个用例回到 0 */
        Chassis_SetTarget(0, 0, 0);
        for (int i = 0; i < 400; i++) { HAL_IncTick(); Chassis_Update(0.005f); }

        /* 给指令，跑到稳态（ACC_UP=3.0 -> 0.33s 到满，跑 1.5s 足够） */
        Chassis_SetTarget(vx, vy, wz);
        for (int i = 0; i < 300; i++) { HAL_IncTick(); Chassis_Update(0.005f); }

        float w[4];
        Chassis_GetWheel(w);

        /* 正解反算速度 */
        float rvx, rvy, rwz;
        Mecanum_Fwd(w, &rvx, &rvy, &rwz);

        g_out[c][0] = vx;
        g_out[c][1] = vy;
        g_out[c][2] = wz;
        g_out[c][3] = w[0];
        g_out[c][4] = w[1];
        g_out[c][5] = w[2];
        g_out[c][6] = w[3];
        g_out[c][7] = (fabsf(vx) < 1e-6f && fabsf(vy) < 1e-6f)
                        ? 0.0f : deg(atan2f(vy, vx));
        /* 期望角：注意 Chassis_SetTarget 会给每个轴乘上各自的 SPEED_GAIN，
           所以严格来说期望方向要按增益后的矢量算。这里两个都存，由脚本对比。 */
        g_out[c][8] = (fabsf(vx) < 1e-6f && fabsf(vy) < 1e-6f)
                        ? 0.0f
                        : deg(atan2f(vy * (float)STRAFE_SIGN  * SPEED_GAIN_STRAFE,
                                     vx * (float)FORWARD_SIGN * SPEED_GAIN_FORWARD));
        g_out[c][9] = (fabsf(rvx) < 1e-6f && fabsf(rvy) < 1e-6f)
                        ? 0.0f : deg(atan2f(rvy, rvx));

        /* 真实写进定时器的比较值（模拟器的内存里能读出来） */
        g_out[c][10] = (float)TIM4->CCR1;
        g_out[c][11] = (float)TIM4->CCR2;
        g_out[c][12] = (float)TIM4->CCR3;
        g_out[c][13] = (float)TIM4->CCR4;
    }

    g_done = 1;
    while (1) { }
    return 0;
}
