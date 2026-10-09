#ifndef __CHASSIS_DIFF_H
#define __CHASSIS_DIFF_H

#include "main.h"

/* ============================================================================
 * chassis_diff.h —— B 车（橡胶轮）差速转向
 *
 * 接口与 mecanum.h 的 Chassis_* 完全同形（连 vy 参数都保留），
 * 目的是让 app.c 可以按"当前开的是哪台车"直接切换，上层逻辑一个字不改。
 * ==========================================================================*/

void Diff_Init(void);
void Diff_SetTarget(float vx, float vy, float wz);   /* vy 会被忽略 */
void Diff_Update(float dt);
void Diff_Stop(void);
void Diff_EmergencyStop(void);
void Diff_GetWheel(float w[4]);

#endif /* __CHASSIS_DIFF_H */
