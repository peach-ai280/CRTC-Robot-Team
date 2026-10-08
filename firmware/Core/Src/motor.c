/* ============================================================================
 * motor.c —— 电机输出层：占空比合成 + 死区补偿 + 起步助力 + 电压补偿
 *
 * 【这一层负责什么】
 *   只负责把一个 -1..1 的“目标速度”变成 PWM 占空比和方向电平。
 *   加速度斜坡、麦轮解算不在这里（分别在 chassis / mecanum），职责分开，
 *   出问题好定位。
 *
 * 【常见现象 -> 改哪里】
 *   小油门不动，到一半突然蹿   -> config.h 的 MOTOR_DEADZONE 调大
 *   起步“咯噔”一下、一顿       -> MOTOR_KICK_DUTY 调大
 *   电池快没电时明显变慢        -> VBAT_COMP_ENABLE 打开
 *   车跑直线往一边偏            -> MOTOR_LEFT_TRIM / MOTOR_RIGHT_TRIM
 * ==========================================================================*/

#include "motor.h"
#include "board.h"
#include "config.h"

/* TIM4 计数周期（CubeMX 里 PSC=0, ARR=7199 -> 10kHz）。改了 CubeMX 要同步改这里 */
#ifndef MOTOR_PWM_PERIOD
#define MOTOR_PWM_PERIOD   7199U
#endif

static float g_duty[4]   = {0, 0, 0, 0};   /* 实际输出占空比 0..1 */
static float g_vcomp     = 1.0f;           /* 电压补偿系数 */

/* 起步助力状态 */
static uint32_t g_kick_start[4] = {0, 0, 0, 0};
static uint8_t  g_was_idle[4]   = {1, 1, 1, 1};

static const uint32_t ch_table[4] = {
    MOTOR_TIM_CH_LF, MOTOR_TIM_CH_RF, MOTOR_TIM_CH_LB, MOTOR_TIM_CH_RB
};
static const uint16_t dir_pin_table[4] = {
    MOTOR_LF_DIR_PIN, MOTOR_RF_DIR_PIN, MOTOR_LB_DIR_PIN, MOTOR_RB_DIR_PIN
};
/* 轮转向修正表：装反了改 config.h 里 MOTOR_x_DIR 的符号，别改线 */
static const int8_t dir_sign_table[4] = {
    MOTOR_LF_DIR, MOTOR_RF_DIR, MOTOR_LB_DIR, MOTOR_RB_DIR
};
/* 左右轮一致性微调 */
static const float trim_table[4] = {
    MOTOR_LEFT_TRIM, MOTOR_RIGHT_TRIM, MOTOR_LEFT_TRIM, MOTOR_RIGHT_TRIM
};

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_TIM_CH_LF);
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_TIM_CH_RF);
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_TIM_CH_LB);
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_TIM_CH_RB);

    for (uint8_t i = 0; i < 4; i++)
    {
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, ch_table[i], 0);
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_table[i], GPIO_PIN_RESET);
        g_duty[i] = 0;
        g_was_idle[i] = 1;
    }
}

void Motor_SetVoltageComp(float vbat)
{
#if VBAT_COMP_ENABLE
    if (vbat < 3.0f) { g_vcomp = 1.0f; return; }   /* 没接电池（USB 供电）时不补偿 */
    g_vcomp = VBAT_NOMINAL / vbat;
    if (g_vcomp > VBAT_COMP_MAX) g_vcomp = VBAT_COMP_MAX;
    if (g_vcomp < 0.8f)          g_vcomp = 0.8f;
#else
    (void)vbat;
    g_vcomp = 1.0f;
#endif
}

/* ---------------------------------------------------------------------------
 * 单轮输出
 * -------------------------------------------------------------------------*/
void Motor_Set(uint8_t idx, float speed)
{
    if (idx > 3) return;

    /* 1) 限幅 */
    if (speed >  1.0f) speed =  1.0f;
    if (speed < -1.0f) speed = -1.0f;

    /* 2) 转向修正 + 左右微调 */
    float s = speed * (float)dir_sign_table[idx] * trim_table[idx];
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    float mag = (s >= 0) ? s : -s;

    /* 3) 起步助力：从静止跨过阈值时叠一个短时 boost，然后线性衰减 */
    uint32_t now = HAL_GetTick();
    float kick = 0.0f;
    if (mag > 0.03f)
    {
        if (g_was_idle[idx])
        {
            g_kick_start[idx] = now;
            g_was_idle[idx]   = 0;
        }
        uint32_t dt = now - g_kick_start[idx];
        if (dt < MOTOR_KICK_MS)
            kick = MOTOR_KICK_DUTY * (1.0f - (float)dt / (float)MOTOR_KICK_MS);
    }
    else
    {
        g_was_idle[idx] = 1;
    }

    /* 4) 死区补偿：把 0..1 的速度映射到 DEADZONE..1 的占空比。
     *    这是开环系统里唯一能对付静摩擦的手段。 */
    float duty;
    if (mag < 0.001f)
        duty = 0.0f;
    else
        duty = MOTOR_DEADZONE + mag * (1.0f - MOTOR_DEADZONE);

    duty += kick;

    /* 5) 电压补偿（电池掉压时把占空比补回来，保证两局之间速度一致） */
    duty *= g_vcomp;

    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;

    /* 6) 拆成 PWM + DIR */
    uint16_t cmp;
    if (s > 0.001f)
    {
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_table[idx], GPIO_PIN_RESET);
        cmp = (uint16_t)(duty * (float)(MOTOR_PWM_PERIOD + 1U));
    }
    else if (s < -0.001f)
    {
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_table[idx], GPIO_PIN_SET);
        cmp = (uint16_t)((1.0f - duty) * (float)(MOTOR_PWM_PERIOD + 1U));
    }
    else
    {
        /* 静止：按配置选择刹车还是滑行 */
        if (MOTOR_IDLE_BRAKE)
        {
            HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_table[idx], GPIO_PIN_SET);
            cmp = MOTOR_PWM_PERIOD + 1U;          /* duty=1, DIR=1 -> 双高刹车 */
        }
        else
        {
            HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_table[idx], GPIO_PIN_RESET);
            cmp = 0;                              /* coast */
        }
        duty = 0.0f;
    }

    if (cmp > MOTOR_PWM_PERIOD + 1U) cmp = MOTOR_PWM_PERIOD + 1U;
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, ch_table[idx], cmp);
    g_duty[idx] = duty;
}

void Motor_OutputAll(float s[4])
{
    for (uint8_t i = 0; i < 4; i++) Motor_Set(i, s[i]);
}

void Motor_BrakeAll(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_table[i], GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, ch_table[i], MOTOR_PWM_PERIOD + 1U);
        g_duty[i] = 1.0f;
        g_was_idle[i] = 1;
    }
}

void Motor_CoastAll(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_table[i], GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, ch_table[i], 0);
        g_duty[i] = 0;
        g_was_idle[i] = 1;
    }
}

float Motor_GetDuty(uint8_t idx) { return (idx < 4) ? g_duty[idx] : 0.0f; }
