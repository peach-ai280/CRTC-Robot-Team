/* ============================================================================
 * arm.c —— 机构动作序列器（串行执行，一次只动一个舵机）
 * ==========================================================================*/

#include "arm.h"
#include "servo.h"
#include "config.h"
#include <stddef.h>      /* NULL */

typedef struct {
    ServoId_t servo;
    float     angle;
    uint16_t  wait_ms;    /* 这一步到位后再等多少毫秒 */
} Step_t;

/* ---------------- 各个姿态的动作脚本（改动作顺序/角度就改这里） -------------- */
/* 收起：夹爪张开 -> 大臂收到最低 -> 前铲抬到行驶位 -> 备用收回 */
static const Step_t seq_stow[] = {
    {SERVO_GRIP,  SERVO_GRIP_OPEN,    120},
    {SERVO_ARM,   SERVO_ARM_STOW,     300},
    {SERVO_SCOOP, SERVO_SCOOP_TRAVEL, 150},
    {SERVO_EXTRA, SERVO_EXTRA_IN,     120},
};

/* 待命：前铲贴地准备收集 */
static const Step_t seq_ready[] = {
    {SERVO_SCOOP, SERVO_SCOOP_DOWN,   200},
    {SERVO_GRIP,  SERVO_GRIP_OPEN,    150},
    {SERVO_ARM,   SERVO_ARM_GROUND,   250},
};

/* 地面取块（核心动作，配合对中限位开关使用）
 *   前铲贴地 -> 夹爪张开到位 -> 大臂下降 -> 夹紧 -> 抬起 -> 前铲兜起送入仓 */
static const Step_t seq_pick_ground[] = {
    {SERVO_SCOOP, SERVO_SCOOP_DOWN,      150},
    {SERVO_ARM,   SERVO_ARM_GROUND,      AUTO_LIFT_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_HOLD_CUBE,  AUTO_GRIP_SETTLE_MS},
    {SERVO_ARM,   SERVO_ARM_LIFT,        AUTO_LIFT_SETTLE_MS},
    {SERVO_SCOOP, SERVO_SCOOP_UP,        AUTO_STORE_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_OPEN,       150},
    {SERVO_ARM,   SERVO_ARM_STOW,        250},
};

/* 台阶 / 焦点平台取块：大臂只降到台阶高度，不用贴地 */
static const Step_t seq_pick_step[] = {
    {SERVO_ARM,   SERVO_ARM_STEP,        AUTO_LIFT_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_HOLD_CORE,  AUTO_GRIP_SETTLE_MS},
    {SERVO_ARM,   SERVO_ARM_LIFT,        AUTO_LIFT_SETTLE_MS},
    {SERVO_SCOOP, SERVO_SCOOP_UP,        AUTO_STORE_SETTLE_MS},
};

/* 入仓：把举着的块推进斜槽储仓（靠重力自锁，不需要夹爪一直用力） */
static const Step_t seq_store[] = {
    {SERVO_ARM,   SERVO_ARM_LIFT,        250},
    {SERVO_SCOOP, SERVO_SCOOP_UP,        AUTO_STORE_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_OPEN,       200},
    {SERVO_ARM,   SERVO_ARM_STOW,        250},
    {SERVO_SCOOP, SERVO_SCOOP_TRAVEL,    150},
};

/* 挑物资架：把挑杆抬到最高 -> 车体前进（由 auto_task 控） -> 兜住 -> 收回 */
static const Step_t seq_rack[] = {
    {SERVO_EXTRA, SERVO_EXTRA_OUT,      200},
    {SERVO_ARM,   SERVO_ARM_RACK,       AUTO_RACK_LIFT_MS},
    {SERVO_EXTRA, SERVO_EXTRA_IN,       200},
    {SERVO_ARM,   SERVO_ARM_STOW,       300},
};

/* 洞窟探杆 */
static const Step_t seq_cave[] = {
    {SERVO_EXTRA, SERVO_EXTRA_OUT,      250},
    {SERVO_ARM,   SERVO_ARM_GROUND,     250},
    {SERVO_EXTRA, SERVO_EXTRA_IN,       250},
    {SERVO_ARM,   SERVO_ARM_STOW,       250},
};

static const Step_t seq_scoop_toggle[] = {
    {SERVO_SCOOP, SERVO_SCOOP_DOWN,     200},
};

typedef struct {
    const Step_t *seq;
    uint8_t       len;
} Seq_t;

static const Seq_t g_seqs[] = {
    {NULL,              0},
    {seq_stow,          sizeof(seq_stow)/sizeof(Step_t)},
    {seq_ready,         sizeof(seq_ready)/sizeof(Step_t)},
    {seq_pick_ground,   sizeof(seq_pick_ground)/sizeof(Step_t)},
    {seq_pick_step,     sizeof(seq_pick_step)/sizeof(Step_t)},
    {seq_store,         sizeof(seq_store)/sizeof(Step_t)},
    {seq_rack,          sizeof(seq_rack)/sizeof(Step_t)},
    {seq_cave,          sizeof(seq_cave)/sizeof(Step_t)},
    {seq_scoop_toggle,  sizeof(seq_scoop_toggle)/sizeof(Step_t)},
};

static uint8_t    g_busy = 0;
static uint8_t    g_idx  = 0;
static uint32_t   g_t_mark = 0;
static ArmPose_t  g_cur_pose = POSE_NONE;
static uint8_t    g_scoop_down = 0;

void Arm_Init(void)
{
    g_busy = 0; g_idx = 0; g_cur_pose = POSE_NONE; g_scoop_down = 0;
    Arm_Start(POSE_STOW);
}

void Arm_Start(ArmPose_t pose)
{
    if ((int)pose <= 0 || (int)pose >= (int)(sizeof(g_seqs)/sizeof(Seq_t))) return;
    g_cur_pose = pose;
    g_idx      = 0;
    g_busy     = 1;
    g_t_mark   = HAL_GetTick();
    /* 立刻下达第一步 */
    Servo_SetAngle(g_seqs[g_cur_pose].seq[0].servo,
                   g_seqs[g_cur_pose].seq[0].angle);
}

/* 每 1~5ms 调用一次即可，内部自己计时 */
void Arm_Update(uint32_t now_ms)
{
    if (!g_busy) return;

    const Seq_t *s = &g_seqs[g_cur_pose];
    if (g_idx >= s->len) { g_busy = 0; g_cur_pose = POSE_NONE; return; }

    const Step_t *st = &s->seq[g_idx];

    /* 当前舵机还在动 -> 等 */
    if (Servo_IsMoving(st->servo)) { g_t_mark = now_ms; return; }

    /* 到位了，还要等 wait_ms */
    if ((uint32_t)(now_ms - g_t_mark) < st->wait_ms) return;

    /* 下一步 */
    g_idx++;
    if (g_idx >= s->len)
    {
        g_busy = 0;
        g_cur_pose = POSE_NONE;
        return;
    }
    Servo_SetAngle(s->seq[g_idx].servo, s->seq[g_idx].angle);
    g_t_mark = now_ms;
}

uint8_t   Arm_IsBusy(void)  { return g_busy; }
ArmPose_t Arm_Current(void) { return g_cur_pose; }

void Arm_Grip(float angle)
{
    Servo_SetAngle(SERVO_GRIP, angle);
}

void Arm_ScoopDown(uint8_t down)
{
    g_scoop_down = down;
    Servo_SetAngle(SERVO_SCOOP, down ? SERVO_SCOOP_DOWN : SERVO_SCOOP_TRAVEL);
}
