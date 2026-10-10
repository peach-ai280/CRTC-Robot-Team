/* ============================================================================
 * arm.c —— 机构动作序列器（串行执行，一次只动一个舵机）
 *
 * 【为什么必须串行】MG995 启动/堵转能到 1.2A，5V 那一小片降压模块根本带不动
 *   两路同时动 —— 表现就是「一动舵机 STM32 就复位 / PS2 掉线」。
 *   本文件的写法天然保证**同一时刻只有一路舵机在动**（下一步一定要等上一步到位），
 *   这也是中期指标 5「任意非初始状态下保持相对静止」的天然条件。
 *
 * 【★★ 回转的顺序铁律（改任何序列都不能违反）】
 *   回转 SG90 的扭矩账只有约 1.2 kg·cm（臂 + 爪约 200g、质心半径约 60mm），
 *   而 MG995 那一段的减速比很大、反驱阻力很大。所以：
 *        ① 先把臂抬到「水平」（SERVO_ARM_STOW）
 *        ② 再回转（SERVO_YAW_*）
 *        ③ 转到位之后，才允许摆臂去取块
 *   **绝对不许带着大角度差硬转** —— 那是扫齿的标准姿势。
 *
 * 【五个舵机分别管什么】
 *   SERVO_ARM   MG995  整条平行四连杆臂上下摆（φ = −50°~+90°）
 *   SERVO_YAW   SG90   整条臂绕竖直轴转（0° 正前 / 180° 正后）
 *   SERVO_GRIP  SG90   末端平行爪开合
 *   SERVO_SCOOP SG90   前铲（★ 地面方块全靠它）
 *   SERVO_EXTRA SG90   挑杆 / 洞窟探杆
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

/* 收起（检录位）：先把臂抬平 → 再转到正后方 → 别的机构归位
 * 这是唯一一个「顺序错了就会撞储仓」的序列，别动它的次序。 */
static const Step_t seq_stow[] = {
    {SERVO_GRIP,  SERVO_GRIP_OPEN,    120},   /* 先松爪，别攥着东西转 */
    {SERVO_ARM,   SERVO_ARM_STOW,     350},   /* ① 抬平 */
    {SERVO_YAW,   SERVO_YAW_STOW,     450},   /* ② 转到正后方（行程最长，给足时间） */
    {SERVO_SCOOP, SERVO_SCOOP_TRAVEL, 150},
    {SERVO_EXTRA, SERVO_EXTRA_IN,     120},
};

/* 待命：臂转回正前方并抬平，前铲放下准备收块 */
static const Step_t seq_ready[] = {
    {SERVO_ARM,   SERVO_ARM_STOW,     300},   /* ① 先抬平才能转 */
    {SERVO_YAW,   SERVO_YAW_FRONT,    450},   /* ② 转回正前方 */
    {SERVO_GRIP,  SERVO_GRIP_OPEN,    150},
    {SERVO_SCOOP, SERVO_SCOOP_DOWN,   200},
};

/* ★ 地面取块：**只用前铲，机械臂完全不参与**
 *   为什么不让臂来夹？因为 φ 要压到 −60° 以下爪子才够得着地面，
 *   而 −50° 以下主臂就扫到收集铲了 —— 这是几何死结（见
 *   tools/check_arm_clearance.py 的输出）。
 *   车前进由 auto_task.c 控（宏 AUTO_APPROACH_*），这里只管铲的姿态：
 *   放铲贴地 → 兜住（抬起一点把块压在铲里）→ 回行驶位。 */
static const Step_t seq_pick_ground[] = {
    {SERVO_SCOOP, SERVO_SCOOP_DOWN,   200},   /* 放铲贴地，等车推过来 */
    {SERVO_SCOOP, SERVO_SCOOP_UP,     AUTO_STORE_SETTLE_MS},   /* 兜住，块就跟着车走 */
};

/* ★ 台阶 / 高台面取块（用臂）：臂转到正前 → 摆到那个高度 → 夹 → 举高
 *   台阶面白块 150~180 → SERVO_ARM_STEP (φ=+19°)
 *   中央高台面白块（相对车底板 135~165）→ 把 SERVO_ARM_STEP 换成 PLATFORM (φ=+11°) */
static const Step_t seq_pick_step[] = {
    {SERVO_ARM,   SERVO_ARM_STOW,        300},   /* 先抬平 */
    {SERVO_YAW,   SERVO_YAW_FRONT,       450},   /* 转正前 */
    {SERVO_GRIP,  SERVO_GRIP_OPEN,       150},
    {SERVO_ARM,   SERVO_ARM_STEP,        AUTO_LIFT_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_HOLD_CUBE,  AUTO_GRIP_SETTLE_MS},
    {SERVO_ARM,   SERVO_ARM_LIFT,        AUTO_LIFT_SETTLE_MS},
};

/* 中央焦点黄块（相对车底板 70~100）→ SERVO_ARM_FOCUS (φ=−26°) */
static const Step_t seq_pick_focus[] = {
    {SERVO_ARM,   SERVO_ARM_STOW,        300},
    {SERVO_YAW,   SERVO_YAW_FRONT,       450},
    {SERVO_GRIP,  SERVO_GRIP_OPEN,       150},
    {SERVO_ARM,   SERVO_ARM_FOCUS,       AUTO_LIFT_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_HOLD_CORE,  AUTO_GRIP_SETTLE_MS},
    {SERVO_ARM,   SERVO_ARM_LIFT,        AUTO_LIFT_SETTLE_MS},
};

/* ★★ 入仓（把爪里的块放进储仓）—— 本方案的核心动作
 *   储仓最后一列正好在「臂朝正后方 + 水平」时爪子的正下方，
 *   落差只有 22mm，是全仓最温和的投料位。
 *   顺序：抬平 → 转正后 → 松爪 → 保持收起。 */
static const Step_t seq_store[] = {
    {SERVO_ARM,   SERVO_ARM_STOW,        350},   /* 先把块举平 */
    {SERVO_YAW,   SERVO_YAW_STOW,        450},   /* ② 转到正后方（块从车上空划过，安全） */
    {SERVO_ARM,   SERVO_ARM_STOW,        250},   /* 落位 */
    {SERVO_GRIP,  SERVO_GRIP_OPEN,       250},   /* 松爪，块掉进仓（落差 ≈22mm） */
    {SERVO_EXTRA, SERVO_EXTRA_IN,        100},
};

/* 挑物资架黄块：抬到最高（φ=+90，只能夹住黄块下半截）
 * ⚠ 队长指示：现场要是拿不下来就**直接放弃**，别为它超时。
 *   车体前进/后退由 auto_task.c 控（AUTO_RACK_*）。 */
static const Step_t seq_rack[] = {
    {SERVO_ARM,   SERVO_ARM_STOW,        300},
    {SERVO_YAW,   SERVO_YAW_FRONT,       450},
    {SERVO_GRIP,  SERVO_GRIP_OPEN,       150},
    {SERVO_ARM,   SERVO_ARM_RACK,        AUTO_RACK_LIFT_MS},
    {SERVO_GRIP,  SERVO_GRIP_HOLD_CORE,  AUTO_GRIP_SETTLE_MS},
    {SERVO_ARM,   SERVO_ARM_LIFT,        AUTO_LIFT_SETTLE_MS},
    {SERVO_EXTRA, SERVO_EXTRA_OUT,       200},   /* 挑杆帮一下：把黄块沿 φ14 圆柱往外拨 */
};

/* 洞窟探杆（爪进不去 70mm 的洞，只能靠它） */
static const Step_t seq_cave[] = {
    {SERVO_ARM,   SERVO_ARM_STOW,        300},   /* ★ 收臂让开，别跟探杆抢空间 */
    {SERVO_YAW,   SERVO_YAW_STOW,        400},
    {SERVO_EXTRA, SERVO_EXTRA_OUT,       250},
    {SERVO_EXTRA, SERVO_EXTRA_IN,        250},
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
    {seq_pick_focus,    sizeof(seq_pick_focus)/sizeof(Step_t)},
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

/* ★ 手动模式用：直接给大臂摆角（已经带了上下限保护）
 *   φ 单位是度，0 = 水平朝前，+ 抬头。 */
void Arm_SetPitch(float phi_deg)
{
    float s = 90.0f + phi_deg;
    if (s < (float)SERVO_ARM_MIN) s = (float)SERVO_ARM_MIN;
    if (s > (float)SERVO_ARM_MAX) s = (float)SERVO_ARM_MAX;
    Servo_SetAngle(SERVO_ARM, s);
}

/* ★ 手动模式用：直接给臂的朝向（0 = 正前，180 = 正后）
 *   ⚠ 调用前请自己确认臂已经抬平，本函数不做保护。 */
void Arm_SetYaw(float yaw_deg)
{
    Servo_SetAngle(SERVO_YAW, yaw_deg);
}
