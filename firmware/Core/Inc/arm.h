#ifndef __ARM_H
#define __ARM_H

#include "main.h"
#include "servo.h"

/* ============================================================================
 * arm.h —— 机构动作序列器（5 路舵机：大臂 / 回转 / 爪 / 铲 / 探杆）
 *
 * 【设计原则：同一时刻只允许一个舵机在动】
 *   两个 SG90 同时动大约 0.5A，MG995 堵转能到 1.2A。官方配套的 Mini 降压
 *   模块标称 1A 左右，极限也就 1.5A。几个一起动必掉压，掉压 -> 单片机复位 ->
 *   比赛里就是"车突然原地死机"。所以这里用一个串行序列器：
 *   一个舵机到位后，等 wait 毫秒，再动下一个。
 *
 * 【★★ 回转铁律】抬平 → 回转 → 再摆臂。理由见 arm.c 顶部注释。
 *
 * 【"舵机运动（多项）"考核点怎么满足】
 *   规则要求：同一台机器人至少三个舵机转动，且其可动机构能在**任意非初始
 *   状态下保持相对静止**。SG90/MG995 都是自带减速箱的舵机，断电保持力矩足够，
 *   演示时把舵机停在中间角度、用手掰不动即可。**本车有 5 个舵机**，富余。
 * ==========================================================================*/

typedef enum {
    POSE_NONE = 0,
    POSE_STOW,          /* 全部收起：抬平 → 转正后 → 检录初始态 */
    POSE_READY,         /* 待命：臂转到正前 + 前铲放下 */
    POSE_PICK_GROUND,   /* 地面取块：★ 只用前铲兜（机械臂不参与） */
    POSE_PICK_STEP,     /* 台阶 / 高台面取块（臂，φ=+19°） */
    POSE_STORE,         /* ★ 入仓：抬平 → 转正后 → 松爪（落差 22mm） */
    POSE_RACK_HOOK,     /* 挑物资架黄块（φ=+90° 举到最高，拿不下就放弃） */
    POSE_CAVE_PROBE,    /* 洞窟探杆取块（爪进不去 70mm 的洞） */
    POSE_SCOOP_TOGGLE,  /* 前铲放/收（手动 R1） */
    POSE_PICK_FOCUS     /* 中央焦点黄块（臂，φ=−26°） */
} ArmPose_t;

void     Arm_Init(void);
void     Arm_Start(ArmPose_t pose);
void     Arm_Update(uint32_t now_ms);
uint8_t  Arm_IsBusy(void);
ArmPose_t Arm_Current(void);
void     Arm_Grip(float angle);        /* 直接控夹爪，不走序列 */
void     Arm_ScoopDown(uint8_t down);  /* 直接控前铲 */
void     Arm_SetPitch(float phi_deg);  /* ★ 手动控大臂摆角（0=水平，+ 抬头，限 −50~+90） */
void     Arm_SetYaw(float yaw_deg);    /* ★ 手动控臂朝向（0=正前，180=正后） */

#endif /* __ARM_H */
