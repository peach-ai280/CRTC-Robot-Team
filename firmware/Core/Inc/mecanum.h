#ifndef __MECANUM_H
#define __MECANUM_H

#include "main.h"

/* ============================================================================
 * mecanum.h —— 麦轮运动学 + 底盘防滑输出
 *
 * 坐标约定（写死在这里，全工程统一）：
 *   x = 车头方向（前为正）
 *   y = 车体左侧方向（左为正）
 *   ω = 逆时针为正（从上往下看）
 * 轮序：0=LF 左前  1=RF 右前  2=LB 左后  3=RB 右后
 *
 * 逆解（X 型麦轮，四轮辊子在俯视图上呈 X）：
 *   w_LF = ( vx - vy - (a+b)·ω ) / R
 *   w_RF = ( vx + vy + (a+b)·ω ) / R
 *   w_LB = ( vx + vy - (a+b)·ω ) / R
 *   w_RB = ( vx - vy + (a+b)·ω ) / R
 * 自检：纯前进 -> 四轮同号同值；纯左移 -> LF/RB 负、RF/LB 正；
 *       纯逆时针 -> 左轮负、右轮正。
 * ==========================================================================*/

void Chassis_Init(void);

/* 设置目标速度（归一化 -1..1）。vx=前后 vy=左右 wz=自转 */
void Chassis_SetTarget(float vx, float vy, float wz);

/* 每 CTRL_PERIOD_MS 调一次，dt 单位秒。内部完成：
 *   斜坡限幅 -> 自转限速 -> 麦轮逆解 -> 矢量归一化 -> 下发电机 */
void Chassis_Update(float dt);

/* 保持当前目标不变，只把输出停掉（急停用，斜坡不生效） */
void Chassis_Stop(void);
void Chassis_EmergencyStop(void);

/* 调试读值 */
void  Chassis_GetCur(float *vx, float *vy, float *wz);
void  Chassis_GetWheel(float w[4]);
void  Mecanum_Fwd(float w[4], float *vx, float *vy, float *wz);  /* 正解，里程计用 */

#endif /* __MECANUM_H */
