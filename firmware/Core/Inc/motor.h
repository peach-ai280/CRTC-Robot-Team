#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* ============================================================================
 * motor.h —— DRV8833 四路直流电机驱动（PHASE / ENABLE 接法）
 *
 * 接线：DRV8833 每个通道两个输入 IN1 / IN2
 *       我们把 IN1 接 MCU 的 PWM，IN2 接 MCU 的普通 GPIO（方向）
 *       真值表（DRV8833）：
 *          IN1  IN2   OUT1  OUT2   效果
 *           0    0     Z     Z     自由滑行 coast
 *           PWM  0     H/L   L     正转，平均电压 = duty * Vm
 *           PWM  1     H     H/L   反转，平均电压 = (1-duty) * Vm
 *           1    1     L     L     刹车 brake
 * 所以：正转 duty = |v|；反转 duty = 1 - |v|。这不是凑合，是手册给的精确关系。
 * ==========================================================================*/

void Motor_Init(void);
/* speed: -1.0 ~ +1.0（正 = 该轮“车体前进”方向，方向修正由 dir_sign 表完成） */
void Motor_Set(uint8_t idx, float speed);
void Motor_OutputAll(float s[4]);
void Motor_BrakeAll(void);
void Motor_CoastAll(void);
float Motor_GetDuty(uint8_t idx);      /* 调试用：实际送出去的占空比 */
void  Motor_SetVoltageComp(float vbat);

#endif /* __MOTOR_H */
