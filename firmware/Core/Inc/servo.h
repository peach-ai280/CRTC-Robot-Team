#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/* ============================================================================
 * servo.h —— 5 路舵机（TIM2 四路 + TIM1_CH4 一路，都是 50Hz）
 *   SERVO_GRIP  夹爪开合        SG90   （PA0 / TIM2_CH1）
 *   SERVO_ARM   ★臂升降 = Z 轴   SG90   （PA1 / TIM2_CH2）  —— 别名 SERVO_LIFT
 *   SERVO_SCOOP 前铲            SG90   （PA2 / TIM2_CH3）
 *   SERVO_EXTRA 挑杆/洞窟探杆    SG90   （PA3 / TIM2_CH4）
 *   SERVO_YAW   ★臂回转 = θ 轴   SG90   （PA11 / TIM1_CH4）
 *
 *   ⚠ 2026-10-10 机械臂改成**柱坐标**（回转 + 升降 + 夹爪）之后，
 *     `SERVO_ARM` 这一路的语义从"大臂俯仰摆角"变成了"**升降（Z 轴）**"，
 *     **引脚和枚举值都没动**（接线冻结），只是含义变了。
 *     新代码请用别名 `SERVO_LIFT`，一眼就知道是柱坐标的 Z 轴。
 *
 * 【新手最容易踩的坑】
 *   1. 几个舵机同时动作会瞬间拉垮 5V，单片机复位 / PS2 掉线。
 *      本驱动对每个舵机做了独立限速（SERVO_SPEED_DPS），而且 arm.c 里
 *      严格保证同一时刻只有一个舵机在动。不要绕过限速直接写比较寄存器。
 *   2. 5V 降压模块带不动两个大舵机同时动。所以三轴臂全用 SG90。
 *   3. 舵机"吱吱"抖 = 脉宽超行程，收 SERVO_PWM_MIN_US / MAX_US。
 *   4. ★ 第 5 路（回转）走的是**另一个定时器**（TIM1，PA11），
 *      所以 servo.c 里是一个「舵机编号 → 定时器句柄」的对照表，不是单一句柄。
 *      加第 6 路的话，两个数组都要改，别忘了 Servo_Init 里也要 Start 一次。
 *   5. ★ 两个定时器都是 72MHz / (71+1) = 1MHz 计数 —— 如果哪天改了其中一个的
 *      预分频，两个数组里填的「微秒数」就不一样了，症状是那一路舵机行程全乱。
 * ==========================================================================*/

typedef enum {
    SERVO_GRIP  = 0,    /* 夹爪（SG90） */
    SERVO_ARM   = 1,    /* ★臂升降 = 柱坐标 Z 轴（SG90）。新代码请写 SERVO_LIFT */
    SERVO_SCOOP = 2,    /* 前铲 / 喇叭口翻板（SG90） */
    SERVO_EXTRA = 3,    /* 挑杆 / 洞窟探杆（SG90） */
    SERVO_YAW   = 4,    /* ★臂回转 = 柱坐标 θ 轴（SG90） */
    SERVO_NUM   = 5
} ServoId_t;

/* ★ 语义别名：柱坐标下这一路就是"升降"，用这个名字读起来才不别扭 */
#define SERVO_LIFT   SERVO_ARM

void    Servo_Init(void);
void    Servo_SetAngle(ServoId_t id, float angle);   /* 设定目标，带缓动 */
void    Servo_SetNow(ServoId_t id, float angle);      /* 立即到位（仅初始化用） */
void    Servo_Update(float dt);                       /* 每 SERVO_PERIOD_MS 调一次 */
float   Servo_GetAngle(ServoId_t id);                 /* 当前实际角度 */
uint8_t Servo_IsMoving(ServoId_t id);
uint8_t Servo_AnyMoving(void);
void    Servo_Disable(ServoId_t id);                  /* 停发脉冲，省电也防抖 */

#endif /* __SERVO_H */
