#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/* ============================================================================
 * servo.h —— 4 路舵机（TIM2，50Hz）
 *   SERVO_GRIP  夹爪   SG90   （PA0 / TIM2_CH1）
 *   SERVO_ARM   大臂   MG995  （PA1 / TIM2_CH2）
 *   SERVO_SCOOP 前铲   SG90   （PA2 / TIM2_CH3）
 *   SERVO_EXTRA 备用   SG90   （PA3 / TIM2_CH4）
 *
 * 【新手最容易踩的坑】
 *   1. 四个舵机同时动作会瞬间拉垮 5V，单片机复位 / PS2 掉线。
 *      本驱动对每个舵机做了独立限速（SERVO_SPEED_DPS），而且 arm.c 里
 *      严格保证同一时刻只有一个舵机在动。不要绕过限速直接写比较寄存器。
 *   2. MG995 堵转会到 1A 以上，5V 降压模块（Mini 降压）带不动两个 MG995。
 *      所以大臂只用 1 个 MG995，夹爪/前铲用 SG90。
 *   3. 舵机“吱吱”抖 = 脉宽超行程，收 SERVO_PWM_MIN_US / MAX_US。
 * ==========================================================================*/

typedef enum {
    SERVO_GRIP  = 0,
    SERVO_ARM   = 1,
    SERVO_SCOOP = 2,
    SERVO_EXTRA = 3,
    SERVO_NUM   = 4
} ServoId_t;

void    Servo_Init(void);
void    Servo_SetAngle(ServoId_t id, float angle);   /* 设定目标，带缓动 */
void    Servo_SetNow(ServoId_t id, float angle);      /* 立即到位（仅初始化用） */
void    Servo_Update(float dt);                       /* 每 SERVO_PERIOD_MS 调一次 */
float   Servo_GetAngle(ServoId_t id);                 /* 当前实际角度 */
uint8_t Servo_IsMoving(ServoId_t id);
uint8_t Servo_AnyMoving(void);
void    Servo_Disable(ServoId_t id);                  /* 停发脉冲，省电也防抖 */

#endif /* __SERVO_H */
