#ifndef __ARM_H
#define __ARM_H

#include "main.h"
#include "servo.h"

/* ============================================================================
 * arm.h —— 机械臂动作序列器（柱坐标构型 / Cylindrical Robot）
 *
 * 【构型】三个**互相独立、互不干涉**的自由度，各一个舵机、各一条指令：
 *      ① θ 回转  SERVO_YAW  (PA11)  0°=正前 / 90°=正左 / 180°=正后
 *      ② Z 升降  SERVO_ARM  (PA1)   滑座沿立柱上下滑  ← ★本文件里叫 SERVO_LIFT
 *      ③ 夹爪    SERVO_GRIP (PA0)
 *   ★ 没有逆运动学、没有联动。上层只需要说"爪尖要升到离地多少 mm"和"朝哪边"，
 *     舵机角由 arm_cal.h 的两张标定表算出来。
 *
 * 【设计原则：同一时刻只允许一个舵机在动】
 *   两个 SG90 同时动约 0.5A，舵机一堵转就是 1A 级。车上那一片 5V 降压模块
 *   根本带不动两个同时动 —— 现象是"一动舵机 STM32 就复位 / PS2 掉线"。
 *   所以这里用串行序列器：一个舵机到位后等 wait 毫秒，再动下一个。
 *
 * 【★★ 回转铁律（柱坐标版，改任何序列都不能违反）】
 *     ① 先把滑座升到最高档（爪尖 95，高于甲板面 88.5）
 *     ② 再回转（SERVO_YAW_*）
 *     ③ 转到位之后，才允许降下来去取块
 *   爪偏心 52mm，回转时爪会扫过车侧；爪尖最低只有 64.5，比甲板面还低 24mm，
 *   低位硬转 = 爪插进甲板/储仓 = 扫齿或别断爪架。
 *   Arm_SetYaw() 里另有一道互锁兜底，但序列本身就该按上面顺序写。
 *
 * 【"舵机运动（多项）"考核点怎么满足】
 *   规则要求：同一台机器人至少三个舵机转动，且其可动机构能在**任意非初始
 *   状态下保持相对静止**。SG90/MG995 自带减速箱，断电保持力矩足够 ——
 *   演示时把舵机停在中间角度、用手掰不动即可。**本车有 5 个舵机**，富余。
 * ==========================================================================*/

typedef enum {
    POSE_NONE = 0,
    POSE_STOW,              /* 全部收起 = 检录初始态（升最高 → 转正后 → 松爪） */
    POSE_READY,             /* 待命：转正前 + 前铲放下 */
    POSE_PICK_GROUND,       /* 地面取块：★ 只用前铲兜（臂够不到地面） */
    POSE_PICK_FOCUS,        /* 中央焦点黄块（爪尖 ~85） */
    POSE_PICK_PLATFORM,     /* 中央高台面白块（爪尖 ~87.5） */
    POSE_PICK_STEP,         /* 台阶面白块（只够下半截，爪尖最高 95） */
    POSE_STORE,             /* ★ 入仓：升最高 → 转正后 → 松爪 */
    POSE_RACK_HOOK,         /* 挑物资架黄块：★ 臂够不到，改用挑杆 SERVO_EXTRA */
    POSE_CAVE_PROBE,        /* 洞窟探杆取块：★ 爪进不去 70mm 的洞，用探杆 */
    POSE_SCOOP_TOGGLE,      /* 前铲放/收（手动 R1） */
    POSE_NUM
} ArmPose_t;

void     Arm_Init(void);
void     Arm_Start(ArmPose_t pose);
void     Arm_Update(uint32_t now_ms);      /* 每 1~5ms 调一次，内部自己计时 */
uint8_t  Arm_IsBusy(void);
ArmPose_t Arm_Current(void);

/* ---- 直接控单个执行器（不走序列，用于手动模式 / 宏里的单步） ---- */
void     Arm_Grip(float angle);            /* 夹爪角度 */
void     Arm_ScoopDown(uint8_t down);      /* 前铲 放下/行驶 */

/* ---- ★ 柱坐标三轴的手动接口 ---- */
/* 升降：给"爪尖要落到的**离地高度 mm**"（比如 85 表示爪尖离地 85mm）。
 *       内部查 ARM_LIFT 标定表 → 舵机角，并做硬限位 + 与回转的互锁。 */
void     Arm_SetLift(float tip_mm);
void     Arm_SetLiftStep(uint8_t idx);     /* 直接给标定表档位 0..ARM_LIFT_N-1 */
float    Arm_LiftMmNow(void);              /* 当前爪尖高度（由舵机角反查） */
float    Arm_LiftDegNow(void);             /* 当前升降舵机角 */

/* 回转：0 = 正前，180 = 正后。★ 带互锁：若当前升降低了而目标朝向落在车侧
 *       危险扇区，本函数会**先把升降抬到安全高度并返回**（这一拍不转），
 *       下一拍再转 —— 于是天然变成"先抬后转"，绝不会低位硬转。 */
void     Arm_SetYaw(float yaw_deg);
uint8_t  Arm_YawLocked(void);              /* 上一次 SetYaw 是否被互锁拦下（1=拦下了） */

/* ---- 换算工具（标定/调试用） ---- */
float    Arm_LiftDegFromMm(float tip_mm);  /* 爪尖高度 mm → 升降舵机角 */

#endif /* __ARM_H */
