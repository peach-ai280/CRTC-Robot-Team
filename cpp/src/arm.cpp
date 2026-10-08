/* ============================================================================
 * arm.cpp —— 机构动作序列器（C++ 版）
 *
 * 【核心设计：一次只动一个舵机】
 *   这是整个机构层最重要的一条。四个舵机同时甩会拉塌 5V，单片机直接复位。
 *   序列器保证：下达当前这一步 -> 等它走完 -> 再等 waitMs -> 才下一步。
 *   代价是动作慢一点（一个取块动作约 2 秒），换来的是不会重启。值得。
 *
 * 【失败怎么办】
 *   每个序列都是"尽力而为"，没有反馈（开环）。夹不住块时不会报错，
 *   只能靠操作手看。所以宏动作都设了超时（见 auto_task.c），超时就收机构。
 *
 * 【角度表怎么来的】
 *   下面这些数是按机构几何推的初值，**必须上电在线标定**。
 *   标定方法：烧 test_arm 固件，用串口发 i/k 微调，听到"吱吱"就往回收，
 *   调好后把角度抄回这里（对应 C 版是 config.h 的 SERVO_*）。
 * ==========================================================================*/

#include "crtc.hpp"

namespace crtc {

/* --- 角度常量（= config.h 的 SERVO_*，改这里等价于改那边） -------------- */
static const float kGripOpen  = 30.0f;    /* 夹爪张开 */
/* V1.4 把方块改成 30mm，这两个角度必须重标；见 firmware/Core/Inc/config.h 的说明 */
static const float kGripCube  = 86.0f;    /* 夹 30mm 白块（能量单元）【待标定】 */
static const float kGripCore  = 84.0f;    /* 夹 30mm 黄块（能量核心）【待标定】 */

/* ⚠ 角度常量必须与 firmware/Core/Inc/config.h 的 SERVO_* 保持一致。
   下面是 2026-09-30 逐项核对过的结果，改 config.h 时这里要同步改。 */
static const float kArmStow   = 25.0f;    /* 大臂收起（检录态）⚠ 按数值序列是偏高的一端，高度需实测 */
static const float kArmGround = 120.0f;   /* 地面取块位 */
static const float kArmStep   = 78.0f;    /* 台阶 / 焦点平台取块位 */
static const float kArmRack   = 15.0f;    /* 挑物资架（抬到最高） */
static const float kArmLift   = 95.0f;    /* 举到储仓口高度 */

static const float kScoopDown   = 150.0f; /* 放铲贴地 */
static const float kScoopTravel = 60.0f;  /* 行驶位（离地约 15mm） */
static const float kScoopUp     = 20.0f;  /* 兜起往储仓送 */

static const float kExtraIn  = 0.0f;
static const float kExtraOut = 180.0f;

/* --- 时间常量 ------------------------------------------------------------ */
static const uint16_t kGripSettle  = 220;  /* 夹紧后等舵机到位 */
static const uint16_t kLiftSettle  = 400;  /* 升降大臂后 */
static const uint16_t kStoreSettle = 350;  /* 入仓后 */
static const uint16_t kRackLift    = 500;  /* 挑架抬起后 */

/* --- 各姿态脚本 ---------------------------------------------------------- */
static const ArmSequencer::Step seqStow[] = {
    { kGrip,  kGripOpen,     120 },
    { kArm,   kArmStow,      300 },
    { kScoop, kScoopTravel,  150 },
    { kExtra, kExtraIn,      120 },
};

static const ArmSequencer::Step seqReady[] = {
    { kScoop, kScoopDown,    200 },
    { kGrip,  kGripOpen,     150 },
    { kArm,   kArmGround,    250 },
};

/* 地面取块：贴地 -> 张爪 -> 降臂 -> 夹紧 -> 抬起 -> 兜入仓 -> 松爪 -> 收臂 */
static const ArmSequencer::Step seqPickGround[] = {
    { kScoop, kScoopDown,    150 },
    { kArm,   kArmGround,    kLiftSettle },
    { kGrip,  kGripCube,     kGripSettle },
    { kArm,   kArmLift,      kLiftSettle },
    { kScoop, kScoopUp,      kStoreSettle },
    { kGrip,  kGripOpen,     150 },
    { kArm,   kArmStow,      250 },
};

/* 台阶 / 焦点平台取块：大臂只降到台阶高度，不用贴地 */
static const ArmSequencer::Step seqPickStep[] = {
    { kArm,   kArmStep,      kLiftSettle },
    { kGrip,  kGripCore,     kGripSettle },
    { kArm,   kArmLift,      kLiftSettle },
    { kScoop, kScoopUp,      kStoreSettle },
};

/* 入仓：靠重力自锁的斜槽储仓，不需要夹爪一直用力 */
static const ArmSequencer::Step seqStore[] = {
    { kArm,   kArmLift,      250 },
    { kScoop, kScoopUp,      kStoreSettle },
    { kGrip,  kGripOpen,     200 },
    { kArm,   kArmStow,      250 },
    { kScoop, kScoopTravel,  150 },
};

/* 挑物资架：伸杆 -> 抬高 -> 收杆兜住 -> 收回 */
static const ArmSequencer::Step seqRack[] = {
    { kExtra, kExtraOut,     200 },
    { kArm,   kArmRack,      kRackLift },
    { kExtra, kExtraIn,      200 },
    { kArm,   kArmStow,      300 },
};

static const ArmSequencer::Step seqCave[] = {
    { kExtra, kExtraOut,     250 },
    { kArm,   kArmGround,    250 },
    { kExtra, kExtraIn,      250 },
    { kArm,   kArmStow,      250 },
};

static const ArmSequencer::Step seqScoop[] = {
    { kScoop, kScoopDown,    200 },
};

/* ------------------------------------------------------------------------ */
const ArmSequencer::Step *ArmSequencer::sequence(Pose pose, int &len)
{
    switch (pose)
    {
    case POSE_STOW:        len = (int)(sizeof(seqStow)       / sizeof(Step)); return seqStow;
    case POSE_READY:       len = (int)(sizeof(seqReady)      / sizeof(Step)); return seqReady;
    case POSE_PICK_GROUND: len = (int)(sizeof(seqPickGround) / sizeof(Step)); return seqPickGround;
    case POSE_PICK_STEP:   len = (int)(sizeof(seqPickStep)   / sizeof(Step)); return seqPickStep;
    case POSE_STORE:       len = (int)(sizeof(seqStore)      / sizeof(Step)); return seqStore;
    case POSE_RACK:        len = (int)(sizeof(seqRack)       / sizeof(Step)); return seqRack;
    case POSE_CAVE:        len = (int)(sizeof(seqCave)       / sizeof(Step)); return seqCave;
    case POSE_SCOOP:       len = (int)(sizeof(seqScoop)      / sizeof(Step)); return seqScoop;
    default:               len = 0; return 0;
    }
}

void ArmSequencer::init(Servo *servos, const Params &p)
{
    sv_ = servos;
    p_  = &p;
    cur_  = POSE_NONE;
    busy_ = 0;
    idx_  = 0;
    tMark_ = 0;
}

void ArmSequencer::start(Pose pose, uint32_t nowMs)
{
    int len = 0;
    const Step *s = sequence(pose, len);
    if (s == 0 || len == 0) return;

    cur_   = pose;
    idx_   = 0;
    busy_  = 1;
    tMark_ = nowMs;
    sv_[s[0].servo].setTarget(s[0].angle);   /* 立刻下达第一步 */
}

void ArmSequencer::update(uint32_t nowMs)
{
    if (!busy_) return;

    int len = 0;
    const Step *s = sequence(cur_, len);
    if (s == 0 || len == 0) { busy_ = 0; cur_ = POSE_NONE; return; }
    if (idx_ >= len)        { busy_ = 0; cur_ = POSE_NONE; return; }

    const Step *st = &s[idx_];

    /* 当前舵机还在动 -> 等（不推进计时） */
    if (sv_[st->servo].isMoving()) { tMark_ = nowMs; return; }

    /* 到位了，还要等 wait_ms 让机构稳定 */
    if ((uint32_t)(nowMs - tMark_) < (uint32_t)st->waitMs) return;

    idx_++;
    if (idx_ >= len)
    {
        busy_ = 0;
        cur_  = POSE_NONE;
        return;
    }
    sv_[s[idx_].servo].setTarget(s[idx_].angle);
    tMark_ = nowMs;
}

void ArmSequencer::grip(float deg)
{
    if (sv_) sv_[kGrip].setTarget(deg);
}

void ArmSequencer::scoopDown(bool down)
{
    if (sv_) sv_[kScoop].setTarget(down ? kScoopDown : kScoopTravel);
}

} /* namespace crtc */
