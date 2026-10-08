/* ============================================================================
 * servo.c —— 舵机 PWM 输出 + 角度缓动
 * TIM2 配置：PSC=71 -> 1MHz 计数，ARR=19999 -> 50Hz（20ms 周期）
 *            比较寄存器直接填微秒数（500~2500），非常直观
 * ==========================================================================*/

#include "servo.h"
#include "board.h"
#include "config.h"
#include <math.h>

static const uint32_t ch[4] = {
    SERVO_TIM_CH_GRIP, SERVO_TIM_CH_ARM, SERVO_TIM_CH_SCOOP, SERVO_TIM_CH_EXTRA
};

static float g_now[4];      /* 当前角度 */
static float g_tgt[4];      /* 目标角度 */
static uint8_t g_en[4];     /* 是否输出脉冲 */

static uint16_t angle_to_us(float a)
{
    if (a < 0.0f)   a = 0.0f;
    if (a > 180.0f) a = 180.0f;
    float us = SERVO_PWM_MIN_US +
               (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US) * (a / 180.0f);
    return (uint16_t)(us + 0.5f);
}

static void apply(ServoId_t id, float a)
{
    if (!g_en[id]) return;
    __HAL_TIM_SET_COMPARE(&SERVO_TIM, ch[id], angle_to_us(a));
}

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&SERVO_TIM, SERVO_TIM_CH_GRIP);
    HAL_TIM_PWM_Start(&SERVO_TIM, SERVO_TIM_CH_ARM);
    HAL_TIM_PWM_Start(&SERVO_TIM, SERVO_TIM_CH_SCOOP);
    HAL_TIM_PWM_Start(&SERVO_TIM, SERVO_TIM_CH_EXTRA);

    for (uint8_t i = 0; i < 4; i++)
    {
        g_en[i]  = 1;
        g_now[i] = 90.0f;
        g_tgt[i] = 90.0f;
    }

    /* 上电就收到“检录姿态”：机构全部收起，别一上电乱甩 */
    Servo_SetNow(SERVO_GRIP,  SERVO_GRIP_OPEN);
    Servo_SetNow(SERVO_ARM,   SERVO_ARM_STOW);
    Servo_SetNow(SERVO_SCOOP, SERVO_SCOOP_TRAVEL);
    Servo_SetNow(SERVO_EXTRA, SERVO_EXTRA_IN);
}

void Servo_SetNow(ServoId_t id, float angle)
{
    if ((int)id >= SERVO_NUM) return;
    g_now[id] = angle;
    g_tgt[id] = angle;
    apply(id, angle);
}

void Servo_SetAngle(ServoId_t id, float angle)
{
    if ((int)id >= SERVO_NUM) return;
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    g_tgt[id] = angle;
}

void Servo_Update(float dt)
{
    float step = SERVO_SPEED_DPS * dt;
    for (uint8_t i = 0; i < 4; i++)
    {
        float d = g_tgt[i] - g_now[i];
        if (fabsf(d) <= step)
        {
            if (g_now[i] != g_tgt[i])
            {
                g_now[i] = g_tgt[i];
                apply((ServoId_t)i, g_now[i]);
            }
        }
        else
        {
            g_now[i] += (d > 0 ? step : -step);
            apply((ServoId_t)i, g_now[i]);
        }
    }
}

float   Servo_GetAngle(ServoId_t id) { return (int)id < SERVO_NUM ? g_now[id] : 0.0f; }
uint8_t Servo_IsMoving(ServoId_t id)
{
    if ((int)id >= SERVO_NUM) return 0;
    return (fabsf(g_tgt[id] - g_now[id]) > 0.5f) ? 1 : 0;
}
uint8_t Servo_AnyMoving(void)
{
    for (uint8_t i = 0; i < 4; i++) if (Servo_IsMoving((ServoId_t)i)) return 1;
    return 0;
}

void Servo_Disable(ServoId_t id)
{
    if ((int)id >= SERVO_NUM) return;
    g_en[id] = 0;
    __HAL_TIM_SET_COMPARE(&SERVO_TIM, ch[id], 0);
}
