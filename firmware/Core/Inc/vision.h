#ifndef __VISION_H
#define __VISION_H

#include "main.h"

/* ============================================================================
 * vision.h —— 视觉模块串口协议（K210 / OpenMV / ESP32-CAM 通用）
 *
 * 帧格式（14 字节，小端）：
 *   [0] 0xAA  [1] 0x55
 *   [2] cls      1=白色能量单元  2=黄色能量核心  0=没找到
 *   [3..4]  x    目标中心横坐标（像素，int16）
 *   [5..6]  y    目标中心纵坐标
 *   [7..8]  w    色块宽度
 *   [9..10] h    色块高度
 *   [11..12] area 像素面积
 *   [13] chk      = (cls ^ x ^ y ^ w ^ h ^ area) & 0xFF，低 8 位异或校验
 *
 * 视觉模块侧的代码见 vision/k210_find_cube.py（MaixPy）。
 *
 * 【为什么帧尾不用 \r\n】
 *   \r = 0x0D、\n = 0x0A 都可能出现在数据里，状态机会被带跑。
 *   定长帧 + 校验和比分隔符帧稳，新手调试时不容易出现"数据错位但还在收"的怪事。
 * ==========================================================================*/

typedef struct {
    uint8_t  cls;
    int16_t  x, y;
    uint16_t w, h;
    uint16_t area;
    uint8_t  fresh;       /* 本周期是否收到过新帧 */
    uint32_t last_ms;
} Vision_t;

extern Vision_t vision;

void    Vision_Init(void);
void    Vision_RxByte(uint8_t b);      /* 在 UART 中断回调里逐字节喂进来 */
void    Vision_Update(uint32_t now_ms);/* 超时则标记失效 */
uint8_t Vision_Valid(void);

#endif /* __VISION_H */
