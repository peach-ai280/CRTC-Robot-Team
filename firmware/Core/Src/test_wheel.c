/* ============================================================================
 * test_wheel.c —— 逐轮慢速测试（一次只转一个轮子，速度很慢）
 *
 * 【为什么单独做一个】
 *   motionauto 是四个轮子一起按运动学跑，出问题时不好分辨"哪个轮子没接"。
 *   这个固件一次只让一个轮子转，速度只有 15%，所以：
 *     · 哪个轮子不转 → 那一路的 PWM 或方向线没接
 *     · 转的方向反了   → 那一路的马达两根线对调
 *
 * 【时序】
 *   上电 → LED 快闪 3 下 → 等 3 秒（倒计时，LED 2Hz）→
 *     左前 正转 2s → 停 1s
 *     右前 正转 2s → 停 1s
 *     左后 正转 2s → 停 1s
 *     右后 正转 2s → 停 1s
 *     左前 反转 2s → 停 1s
 *     右前 反转 2s → 停 1s
 *     左后 反转 2s → 停 1s
 *     右后 反转 2s → 停 1s
 *   → 全部刹车停住，LED 常亮。总共约 27 秒，跑完不再动。
 *
 * 【⚠ 安全】
 *   ① 四轮必须离地（架起来）
 *   ② 手放在电源旁边
 *   ③ 跑完自动停，想再测就拔插 USB 重新上电
 *   ④ 15% 速度很慢，但 TT 马达有减速箱，扭矩不小，手指别伸进轮子里
 * ==========================================================================*/

#include "main.h"
#include "bsp.h"
#include "board.h"
#include "config.h"
#include "motor.h"
#include "sensor.h"

#define AUTO_START_DELAY_MS   3000
#define STEP_MS               2000      /* 每个轮子转多久 */
#define GAP_MS                 800      /* 轮子之间停多久 */
#define WHEEL_SPEED           0.15f     /* ★ 很慢：15% */
#define TOTAL_STEPS              8      /* 4 个正转 + 4 个反转 */

/* 步号 → (轮子下标, 方向) */
static const uint8_t k_wheel[TOTAL_STEPS] = {
    MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB,
    MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB,
};
static const float k_sign[TOTAL_STEPS] = {
    +1.0f, +1.0f, +1.0f, +1.0f,
    -1.0f, -1.0f, -1.0f, -1.0f,
};

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_Init();
    BSP_TIM4_Init();
    BSP_ADC1_Init();

    Sensor_Init();
    Motor_Init();

    /* 开机指示：快闪 3 下 */
    for (int i = 0; i < 6; i++)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);      /* 灭 */

    uint32_t t0 = HAL_GetTick();
    uint8_t  done = 0;
    uint8_t  last_active = 0xFE;      /* 0xFE = 不可能的值，保证第一轮一定刷新 */

    while (1)
    {
        uint32_t now     = HAL_GetTick();
        uint32_t elapsed = now - t0;

        /* 本周期「唯一应该转」的那一路；0xFF = 都不转 */
        uint8_t active = 0xFF;
        float   spd    = 0.0f;

        if (!done && elapsed >= AUTO_START_DELAY_MS)
        {
            uint32_t k     = elapsed - AUTO_START_DELAY_MS;
            uint32_t cycle = STEP_MS + GAP_MS;
            uint8_t  s     = (uint8_t)(k / cycle);

            if (s >= TOTAL_STEPS)          done = 1;
            else if ((k % cycle) < STEP_MS)
            {
                active = k_wheel[s];
                spd    = k_sign[s] * WHEEL_SPEED;
            }
        }

        /* ★★★ 关键：切换轮子时，先把四路全部设成「滑行」，再单独给活动那一路速度。
         * 绝不能对非活动通道调用 Motor_Set(i, 0) —— 那会走「刹车」分支
         * （DIR 高 + PWM 100%），只要该路方向线没接，驱动芯片就会当成
         * 「PWM 高 + 方向低」= 全速正转，于是出现"控制一个、另外几路跟着狂奔"。
         * 滑行是 DIR 低 + PWM 0，方向线断了也只是不转。
         *
         * ⚠ 两个坑，都踩过（2026-10-19 寄存器实测发现）：
         *   ① Motor_CoastAll() 不能每周期调 —— 它把 g_was_idle 置 1，
         *      会让「起步助力」每周期重新触发，kick 恒为最大值。
         *   ② Motor_Set() 也不能只在切换时调一次 —— PWM 寄存器会保持上次的值，
         *      起步助力那 0.12 就永远不衰减（实测 CCR 恒为 3045，应为 2182）。
         *   ⇒ 正确写法：CoastAll 只在切换时调，Motor_Set 每周期都调。 */
        if (active != last_active)
        {
            Motor_CoastAll();
            last_active = active;
        }
        if (active != 0xFF)
            Motor_Set(active, spd);

        /* LED：跑完常亮 / 倒计时快闪 / 动作时常亮 */
        if (done)
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        else if (elapsed < AUTO_START_DELAY_MS)
            HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                              ((now / 250u) & 1u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                              (active != 0xFF) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}
