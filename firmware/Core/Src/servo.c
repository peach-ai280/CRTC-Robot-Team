/* ============================================================================
 * servo.c —— 舵机 PWM 输出 + 角度缓动
 * TIM2 配置：PSC=71 -> 1MHz 计数，ARR=19999 -> 50Hz（20ms 周期）
 *            比较寄存器直接填微秒数（500~2500），非常直观
 * ★ TIM1_CH4（PA11，臂回转）参数完全一样，只是挂在另一个定时器上，
 *   所以下面用「每路一个定时器句柄」的对照表，而不是单一句柄。
 * ==========================================================================*/

#include "servo.h"
#include "board.h"
#include "config.h"
#include <math.h>

/* 每路舵机自己的定时器句柄与通道。★ 第 5 路在 TIM1 上，别写成同一个。 */
static TIM_HandleTypeDef *const g_tim[SERVO_NUM] = {
    &SERVO_TIM, &SERVO_TIM, &SERVO_TIM, &SERVO_TIM, &SERVO_TIM_YAW
};
static const uint32_t ch[SERVO_NUM] = {
    SERVO_TIM_CH_GRIP, SERVO_TIM_CH_ARM, SERVO_TIM_CH_SCOOP,
    SERVO_TIM_CH_EXTRA, SERVO_TIM_CH_YAW
};

static float   g_now[SERVO_NUM];    /* 当前角度 */
static float   g_tgt[SERVO_NUM];    /* 目标角度 */
static uint8_t g_en[SERVO_NUM];     /* 是否输出脉冲 */

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
    if ((int)id >= SERVO_NUM || !g_en[id]) return;
    __HAL_TIM_SET_COMPARE(g_tim[id], ch[id], angle_to_us(a));
}

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&SERVO_TIM,     SERVO_TIM_CH_GRIP);
    HAL_TIM_PWM_Start(&SERVO_TIM,     SERVO_TIM_CH_ARM);
    HAL_TIM_PWM_Start(&SERVO_TIM,     SERVO_TIM_CH_SCOOP);
    HAL_TIM_PWM_Start(&SERVO_TIM,     SERVO_TIM_CH_EXTRA);
    HAL_TIM_PWM_Start(&SERVO_TIM_YAW, SERVO_TIM_CH_YAW);   /* ★ 第 5 路 */

    for (uint8_t i = 0; i < SERVO_NUM; i++)
    {
        g_en[i]  = 1;
        g_now[i] = 90.0f;
        g_tgt[i] = 90.0f;
    }

    /* 上电就收到“检录姿态”：机构全部收起，别一上电乱甩。
     * ★ 顺序很重要：先把臂摆平（ARM_STOW）再回转到正后方，
     *   不能带着大角度差硬转 —— 那是扫齿的标准姿势。 */
    Servo_SetNow(SERVO_GRIP,  SERVO_GRIP_OPEN);
    Servo_SetNow(SERVO_ARM,   SERVO_ARM_STOW);
    Servo_SetNow(SERVO_YAW,   SERVO_YAW_STOW);
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
    for (uint8_t i = 0; i < SERVO_NUM; i++)
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
    for (uint8_t i = 0; i < SERVO_NUM; i++) if (Servo_IsMoving((ServoId_t)i)) return 1;
    return 0;
}

void Servo_Disable(ServoId_t id)
{
    if ((int)id >= SERVO_NUM) return;
    g_en[id] = 0;
    __HAL_TIM_SET_COMPARE(g_tim[id], ch[id], 0);
}
