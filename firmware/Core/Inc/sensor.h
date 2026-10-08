#ifndef __SENSOR_H
#define __SENSOR_H

#include "main.h"

/* ============================================================================
 * sensor.h —— 限位开关 + 电池电压
 *
 * 【对中原理（半自动的核心，不用视觉也能做）】
 *   车头是一个 V 形喇叭口，两侧各装一个微动开关（KW11 / SS-5GL 之类的小行程
 *   开关，5 元 10 个）。方块被铲进去之后会同时压到左右两个摆杆，两个开关都
 *   闭合 => 说明方块已经居中 => 触发夹紧。
 *   这比视觉鲁棒得多：不受光照影响，不需要调阈值，新手半小时能调通。
 *   视觉是"锦上添花"，对中是"保底方案"，两者在代码里可以并存切换。
 * ==========================================================================*/

void    Sensor_Init(void);
void    Sensor_Update(uint32_t now_ms);   /* 每 1~5ms 调一次，做去抖 */
uint8_t Limit_Left(void);                 /* 1 = 左摆杆被压 */
uint8_t Limit_Right(void);
uint8_t Limit_Centered(uint32_t now_ms);  /* 1 = 左右都在窗口期内闭合过 => 已对中 */
float   Sensor_GetVbat(void);             /* 伏特 */
uint8_t Sensor_KeyPressed(void);

#endif /* __SENSOR_H */
