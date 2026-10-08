#ifndef __PS2_H
#define __PS2_H

#include "main.h"
#include <stdint.h>

/* PS2 按键位（低电平有效：数据位为 0 表示按下） */
#define PSB_SELECT      0x0001
#define PSB_L3          0x0002
#define PSB_R3          0x0004
#define PSB_START       0x0008
#define PSB_PAD_UP      0x0010
#define PSB_PAD_RIGHT   0x0020
#define PSB_PAD_DOWN    0x0040
#define PSB_PAD_LEFT    0x0080
#define PSB_L2          0x0100
#define PSB_R2          0x0200
#define PSB_L1          0x0400
#define PSB_R1          0x0800
#define PSB_TRIANGLE    0x1000
#define PSB_CIRCLE      0x2000
#define PSB_CROSS       0x4000
#define PSB_SQUARE      0x8000

/* 摇杆原始值范围 0~255，中位 128 */
typedef struct
{
    uint16_t btn;        /* 当前按键位图 */
    uint16_t btn_prev;   /* 上一次按键位图（用于边沿检测） */
    uint16_t btn_click;  /* 本周期“刚按下”的位图 */
    int16_t  lx, ly;     /* 左摇杆：已减 128，范围 -128..127 */
    int16_t  rx, ry;     /* 右摇杆 */
    uint8_t  mode;       /* 0x41=数字模式 0x73=模拟绿灯 0x79=模拟红灯 */
    uint8_t  connected;  /* 1=在线 */
    uint16_t lost_cnt;   /* 连续失联计数 */
    uint8_t  raw[9];     /* 最近一帧的原始 9 字节，调试用（全 FF = 对面没回应） */
} PS2_t;

extern PS2_t ps2;

void     PS2_Init(void);
uint8_t  PS2_Poll(void);                 /* 返回 1 表示本帧数据有效 */
uint8_t  PS2_Key(uint16_t key);          /* 是否按住 */
uint8_t  PS2_Clicked(uint16_t key);      /* 是否刚按下（上升沿） */
void     PS2_Scan(void);                 /* 放到 5ms 任务里：轮询 + 掉线保护 */

#endif /* __PS2_H */
