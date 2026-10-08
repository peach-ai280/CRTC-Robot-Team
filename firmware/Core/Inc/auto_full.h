#ifndef __AUTO_FULL_H
#define __AUTO_FULL_H

#include "main.h"

/* ============================================================================
 * auto_full.h —— 全自动模式（在半自动宏之上再叠一层「自动调度」）
 *
 * 【和半自动的关系：不是重写，是编排】
 *   半自动（auto_task.c）= 操作手按一个键，车完成一串动作。决策权在人。
 *   全自动（本文件）    = 车自己巡游找块，找到就**自动触发那个宏**，
 *                        抓完自己换地方继续找。决策权在车。
 *
 *   所以全自动并没有重新实现一遍"对中→夹→抬→入仓"，它只是自动按下了
 *   那个键。这样写有两个好处：
 *     1. 宏调好的参数，全自动直接复用，不用再调第二遍
 *     2. 半自动跑稳了，全自动就稳了一半——你是在验证过的代码上叠加
 *
 * 【这算不算"全自动"？】
 *   手册：全自动 = 无需配备操作手 + 经组委会审查确定为自动运行。
 *   本模式进入后无人干预即可连续作业，符合定义。但**仍需组委会现场认定**，
 *   所以别把宝押在这上面，半自动必须能独立跑通。
 *
 * 【三道保险，缺一不可】
 *   1. 总时长硬上限 AUTO_TOTAL_LIMIT_MS，到点无条件停车（一局只有 5 分钟）
 *   2. 连续失败 AUTO_FAIL_LIMIT 次自动退出，回到半自动等人接管
 *   3. 操作手动一下摇杆 / 按 START，立刻退出全自动（在 app.c 里实现）
 * ==========================================================================*/

typedef enum {
    FA_OFF = 0,
    FA_ARM,        /* 预备：机构收起，给人 AUTO_FULL_ARM_DELAY_MS 时间走开 */
    FA_SEARCH,     /* 搜索：原地慢转扫视（有视觉等目标，没视觉转够时间就巡游） */
    FA_CRUISE,     /* 巡游：蛇形盲扫，靠对中限位触发 */
    FA_APPROACH,   /* 靠拢：视觉 P 控制 或 直推等对中 */
    FA_MACRO,      /* 交给半自动宏执行抓取，本状态只负责等它结束 */
    FA_RECOVER,    /* 收尾：机构收起 + 后退一步，防止块卡在铲口 */
    FA_DONE        /* 结束：仓满 / 超时 / 连败，停车等人 */
} FullState_t;

void        FullAuto_Init(void);
void        FullAuto_Start(void);   /* 进入全自动 */
void        FullAuto_Stop(void);    /* 退出全自动（会顺带中止正在跑的宏） */
uint8_t     FullAuto_Running(void);
FullState_t FullAuto_State(void);
void        FullAuto_Update(uint32_t now_ms);          /* CTRL 周期调用（5ms） */
void        FullAuto_GetCmd(float *vx, float *vy, float *wz);
uint8_t     FullAuto_Picked(void);                     /* 本轮已取到几个 */
uint8_t     FullAuto_Failed(void);                     /* 连续失败次数 */
const char *FullAuto_StateName(void);                  /* 串口打印用 */

#endif /* __AUTO_FULL_H */
