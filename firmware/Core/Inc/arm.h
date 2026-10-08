#ifndef __ARM_H
#define __ARM_H

#include "main.h"
#include "servo.h"

/* ============================================================================
 * arm.h —— 机构动作序列器
 *
 * 【设计原则：同一时刻只允许一个舵机在动】
 *   两个 SG90 同时动大约 0.5A，MG995 堵转能到 1.2A。官方配套的 Mini 降压
 *   模块标称 1A 左右，极限也就 1.5A。四个一起动必掉压，掉压 -> 单片机复位 ->
 *   比赛里就是"车突然原地死机"。所以这里用一个串行序列器：
 *   一个舵机到位后，等 wait 毫秒，再动下一个。慢是慢了点（一个完整取块
 *   动作约 1.2 秒），但**不掉电**，这在比赛里比快重要得多。
 *
 * 【"舵机运动（多项）"考核点怎么满足】
 *   规则要求：同一台机器人至少三个舵机转动，且其可动机构能在**任意非初始
 *   状态下保持相对静止**。SG90/MG995 都是自带减速箱的舵机，断电保持力矩足够，
 *   演示时把三个舵机停在中间角度、用手掰不动即可。不要选那种"弹簧回中"的
 *   数字舵机模式。
 * ==========================================================================*/

typedef enum {
    POSE_NONE = 0,
    POSE_STOW,          /* 全部收起（检录初始态，最低矮） */
    POSE_READY,         /* 前铲放下的待命姿态 */
    POSE_PICK_GROUND,   /* 地面取块：放铲 -> 下降 -> 夹紧 -> 抬起 -> 兜入仓 */
    POSE_PICK_STEP,     /* 台阶/焦点平台高度取块 */
    POSE_STORE,         /* 把举着的块推进储仓并复位夹爪 */
    POSE_RACK_HOOK,     /* 挑物资架上的黄色能量核心 */
    POSE_CAVE_PROBE,    /* 洞窟探杆取块 */
    POSE_SCOOP_TOGGLE   /* 前铲放/收（手动 R1） */
} ArmPose_t;

void     Arm_Init(void);
void     Arm_Start(ArmPose_t pose);
void     Arm_Update(uint32_t now_ms);
uint8_t  Arm_IsBusy(void);
ArmPose_t Arm_Current(void);
void     Arm_Grip(float angle);        /* 直接控夹爪，不走序列 */
void     Arm_ScoopDown(uint8_t down);  /* 直接控前铲 */

#endif /* __ARM_H */
