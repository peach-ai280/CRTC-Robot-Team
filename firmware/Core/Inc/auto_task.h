#ifndef __AUTO_TASK_H
#define __AUTO_TASK_H

#include "main.h"

/* ============================================================================
 * auto_task.h —— 半自动「一键宏」
 *
 * 【为什么这么做就是半自动，而不是全自动】
 *   手册定义：全自动 = 无需配备操作手 + 经组委会审查确定为自动运行；
 *   只满足一部分的叫半自动。我们让操作手按一个键触发一个**有时间上限、
 *   中途可被摇杆打断**的动作序列 —— 车本身不自主决策去哪、抢什么，
 *   决策权在操作手。这是最稳妥的半自动认定方式，也最不容易在审查时翻车。
 *
 * 【安全设计】
 *   1. 每个宏都有硬超时，卡住不会一直顶着电机（顶着 = 堵转 = 烧驱动/电池）
 *   2. 宏运行期间操作手一动摇杆就立刻中止，控制权交回人工
 *   3. 宏只在"车体静止或低速"时允许启动，避免高速撞墙
 * ==========================================================================*/

typedef enum {
    MACRO_NONE = 0,
    MACRO_GROUND,   /* ○ 键：地面方块自动收集（对中 -> 夹 -> 抬 -> 入仓） */
    MACRO_RACK,     /* △ 键：挑物资架上的黄色能量核心 */
    MACRO_CAVE,     /* □ 键：洞窟探杆取块 */
    MACRO_EDGE,     /* × 键：一键贴边横移 */
    MACRO_VISION    /* L2 键：视觉辅助自动对位并抓取（需 USE_VISION=1） */
} Macro_t;

void     Auto_Init(void);
void     Auto_Start(Macro_t m);
void     Auto_Abort(void);
uint8_t  Auto_Running(void);
Macro_t  Auto_Current(void);
void     Auto_Update(uint32_t now_ms);            /* 5ms 任务 */
void     Auto_GetCmd(float *vx, float *vy, float *wz);
uint8_t  Auto_LastResult(void);                   /* 1=成功 0=超时失败 */

#endif /* __AUTO_TASK_H */
