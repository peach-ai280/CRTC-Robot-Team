/* ============================================================================
 * arm.c —— 柱坐标机械臂动作序列器（串行执行，一次只动一个舵机）
 *
 * 【构型】三个独立自由度：θ 回转(yaw) / Z 升降(lift) / 夹爪(grip)。
 *   没有逆运动学、没有联动 —— 序列里只写"爪尖要升到多高"和"朝哪边"，
 *   舵机角由 arm_cal.h 的两张标定表算。
 *
 * 【为什么必须串行】舵机堵转能到 1A 级，车上那一片 5V 降压模块带不动两路
 *   同时动 —— 表现就是"一动舵机 STM32 就复位 / PS2 掉线"。
 *   本文件的写法天然保证**同一时刻只有一路舵机在动**（下一步一定等上一步到位），
 *   这也顺带满足中期指标 5「任意非初始状态下保持相对静止」。
 *
 * 【★★ 回转的顺序铁律（改任何序列都不能违反）】
 *     ① 先把滑座升到最高（ARM_TIP_HOME，爪尖 95 > 甲板 88.5）
 *     ② 再回转（SERVO_YAW_*）
 *     ③ 转到位后，才允许降下来去取块
 *   爪偏心 52mm，回转时爪会扫过车侧；爪尖最低只有 64.5，比甲板还低 24mm。
 *   **低位硬转 = 爪插进甲板/储仓 = 扫齿或别断爪架**。
 *   Arm_SetYaw() 里还有一道互锁兜底，但序列本身就该按这个顺序写。
 *
 * 【五个舵机分别管什么】
 *   SERVO_LIFT(SERVO_ARM) SG90  滑座沿立柱上下滑（柱坐标的 Z 轴）
 *   SERVO_YAW   SG90  整根立柱绕底座转（柱坐标的 θ 轴）
 *   SERVO_GRIP  SG90  爪开合
 *   SERVO_SCOOP SG90  前铲（★ 地面方块全靠它）
 *   SERVO_EXTRA SG90  挑杆 / 洞窟探杆
 * ==========================================================================*/

#include "arm.h"
#include "servo.h"
#include "config.h"
#include "arm_cal.h"
#include <stddef.h>      /* NULL */

typedef struct {
    ServoId_t servo;
    float     val;        /* 含义随 servo：升降舵机 = "爪尖离地 mm"；其它 = "舵机角" */
    uint16_t  wait_ms;    /* 这一步到位后再等多少毫秒 */
} Step_t;

/* ---------------------------------------------------------------------------
 * 标定表（编译期从 arm_cal.h 展开；机械组改尺寸只改 arm_cal.h）
 * -------------------------------------------------------------------------*/
static const float g_lift_tip[ARM_LIFT_N] = {
    ARM_CAL_TIP_0, ARM_CAL_TIP_1, ARM_CAL_TIP_2,
    ARM_CAL_TIP_3, ARM_CAL_TIP_4, ARM_CAL_TIP_5
};
static const float g_lift_deg[ARM_LIFT_N] = {
    ARM_CAL_DEG_0, ARM_CAL_DEG_1, ARM_CAL_DEG_2,
    ARM_CAL_DEG_3, ARM_CAL_DEG_4, ARM_CAL_DEG_5
};

/* 爪尖高度 mm → 升降舵机角（分段线性插值，两端钳位） */
float Arm_LiftDegFromMm(float tip_mm)
{
    if (tip_mm <= g_lift_tip[0])            return g_lift_deg[0];
    if (tip_mm >= g_lift_tip[ARM_LIFT_N-1]) return g_lift_deg[ARM_LIFT_N-1];

    for (uint8_t i = 1; i < ARM_LIFT_N; i++)
    {
        if (tip_mm <= g_lift_tip[i])
        {
            float span = g_lift_tip[i] - g_lift_tip[i-1];
            float t    = (span > 0.0f) ? (tip_mm - g_lift_tip[i-1]) / span : 0.0f;
            return g_lift_deg[i-1] + t * (g_lift_deg[i] - g_lift_deg[i-1]);
        }
    }
    return g_lift_deg[ARM_LIFT_N-1];
}

/* 升降舵机角 → 爪尖高度 mm（反查，给"现在到底多高"用） */
static float lift_tip_from_deg(float deg)
{
    if (deg <= g_lift_deg[0])            return g_lift_tip[0];
    if (deg >= g_lift_deg[ARM_LIFT_N-1]) return g_lift_tip[ARM_LIFT_N-1];

    for (uint8_t i = 1; i < ARM_LIFT_N; i++)
    {
        if (deg <= g_lift_deg[i])
        {
            float span = g_lift_deg[i] - g_lift_deg[i-1];
            float t    = (span > 0.0f) ? (deg - g_lift_deg[i-1]) / span : 0.0f;
            return g_lift_tip[i-1] + t * (g_lift_tip[i] - g_lift_tip[i-1]);
        }
    }
    return g_lift_tip[ARM_LIFT_N-1];
}

/* ---------------- 各个姿态的动作脚本（改动作顺序/高度就改这里） -------------- */
/* ★ 注意顺序：凡是动 yaw 的，前面一定先有一步把升降送到 ARM_TIP_HOME。 */

/* 收起（检录位）：松爪 → 升最高 → 转正后 → 别的机构归位
 * 这是唯一一个「顺序错了就会撞储仓」的序列，别动它的次序。
 * ★ 铲收到 SCOOP_ANG_STOW = 齿朝上竖立。**这一条是检录车长红线（≤290）**：
 *   齿长 90~100 的话，只有竖立才不超长；平放（GROUND/RAKE）会直接顶到 340+。 */
static const Step_t seq_stow[] = {
    {SERVO_GRIP,  SERVO_GRIP_OPEN,    120},   /* 先松爪，别攥着东西转 */
    {SERVO_LIFT,  ARM_TIP_HOME,       350},   /* ① 升到最高（=回转安全高度） */
    {SERVO_YAW,   SERVO_YAW_STOW,     600},   /* ② 转到正后方（行程最长，给足时间） */
    {SERVO_SCOOP, SCOOP_ANG_STOW,     250},   /* ③ 铲竖立（车长红线） */
    {SERVO_EXTRA, SERVO_EXTRA_IN,     120},
};

/* 待命：转回正前方 + 张爪 + 前铲放下准备收块 */
static const Step_t seq_ready[] = {
    {SERVO_LIFT,  ARM_TIP_HOME,       300},   /* ① 先升最高才能转 */
    {SERVO_YAW,   SERVO_YAW_FRONT,    600},   /* ② 转回正前方 */
    {SERVO_GRIP,  SERVO_GRIP_OPEN,    150},
    {SERVO_SCOOP, SERVO_SCOOP_DOWN,   200},
};

/* ★ 地面取块：**只用前铲，机械臂完全不参与**
 *   柱坐标臂爪尖最低只能到离地 64.5，而地面方块顶面才 30 —— 差 34.5，够不着。
 *
 *   ★★ 2026-10-17 机械组实物定稿：不是"铲"，是**翻转式四齿耙 + 背板**。
 *   收块原理不是楔入（方块贴地零间隙，齿插不进去），而是「扫到齿前 → 翻转搂起」。
 *
 *   ⚠ 本序列**不含车体运动**（它是纯机构序列），只适合手动模式/调试：
 *     人推着车把块顶到齿前，再让它翻。
 *     **半自动宏（○ 键）走的是 auto_task.c 里完整的六段：贴地→扫掠→爬行顶→
 *     翻转→抖动送料→后退**，那个才是实战用法，别指望这个序列能自动收块。 */
static const Step_t seq_pick_ground[] = {
    {SERVO_SCOOP, SCOOP_ANG_GROUND,   250},   /* ① 压到贴地（齿尖最低） */
    {SERVO_SCOOP, SCOOP_ANG_FLIP,     550},   /* ② 一把翻上来（120°/240dps ≈ 0.5s） */
    {SERVO_SCOOP, SCOOP_ANG_HOLD,     300},   /* ③ 停在收纳位（背板托住块） */
};

/* ★ 高台面取块（用臂）：升最高 → 转正前 → 张爪 → 降到目标高度 → 夹 → 抬回最高 */
static const Step_t seq_pick_platform[] = {
    {SERVO_LIFT,  ARM_TIP_HOME,         300},
    {SERVO_YAW,   SERVO_YAW_FRONT,      600},
    {SERVO_GRIP,  SERVO_GRIP_OPEN,      150},
    {SERVO_LIFT,  ARM_TIP_PLATFORM,     AUTO_LIFT_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_HOLD_CUBE, AUTO_GRIP_SETTLE_MS},
    {SERVO_LIFT,  ARM_TIP_HOME,         AUTO_LIFT_SETTLE_MS},
};

/* 台阶面白块（★ 臂最高只到 95，块底 87.5 —— 只够下半截，现场不行就放弃） */
static const Step_t seq_pick_step[] = {
    {SERVO_LIFT,  ARM_TIP_HOME,         300},
    {SERVO_YAW,   SERVO_YAW_FRONT,      600},
    {SERVO_GRIP,  SERVO_GRIP_OPEN,      150},
    {SERVO_LIFT,  ARM_TIP_STEP,         AUTO_LIFT_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_HOLD_CUBE, AUTO_GRIP_SETTLE_MS},
    {SERVO_LIFT,  ARM_TIP_HOME,         AUTO_LIFT_SETTLE_MS},
};

/* 中央焦点黄块（爪尖 85 落在块高 70~100 里） */
static const Step_t seq_pick_focus[] = {
    {SERVO_LIFT,  ARM_TIP_HOME,         300},
    {SERVO_YAW,   SERVO_YAW_FRONT,      600},
    {SERVO_GRIP,  SERVO_GRIP_OPEN,      150},
    {SERVO_LIFT,  ARM_TIP_FOCUS,        AUTO_LIFT_SETTLE_MS},
    {SERVO_GRIP,  SERVO_GRIP_HOLD_CORE, AUTO_GRIP_SETTLE_MS},
    {SERVO_LIFT,  ARM_TIP_HOME,         AUTO_LIFT_SETTLE_MS},
};

/* ★★ 入仓（把爪里的块放进储仓）—— 本方案的核心动作
 *   储仓最后一列正好在「臂朝正后方」时爪子的正下方，落差只有 0.5mm，
 *   是全仓最温和的投料位。顺序：升最高 → 转正后 → 落到投放高度 → 松爪。 */
static const Step_t seq_store[] = {
    {SERVO_LIFT,  ARM_TIP_HOME,       350},   /* 先把块举到最高 */
    {SERVO_YAW,   SERVO_YAW_STOW,     600},   /* ② 转到正后方（块从车上空划过，安全） */
    {SERVO_LIFT,  ARM_TIP_DROP,       250},   /* 落到投放高度 */
    {SERVO_GRIP,  SERVO_GRIP_OPEN,    250},   /* 松爪，块掉进仓（落差 ≈0.5mm） */
    {SERVO_EXTRA, SERVO_EXTRA_IN,     100},
};

/* ★ 挑物资架黄块：**柱坐标臂根本够不到**（爪尖最高 95，黄块质心离地 250，差 155），
 *   所以改成"收臂让开 + 用挑杆 SERVO_EXTRA 拨"。车体前进/后退由 auto_task.c 控。
 *   ⚠ 队长指示：现场要是拿不下来就**直接放弃**，别为它超时。 */
static const Step_t seq_rack[] = {
    {SERVO_LIFT,  ARM_TIP_HOME,       300},   /* ★ 收臂让开，别跟挑杆抢空间 */
    {SERVO_YAW,   SERVO_YAW_STOW,     600},
    {SERVO_EXTRA, SERVO_EXTRA_OUT,    250},   /* 挑杆伸出 */
    {SERVO_EXTRA, SERVO_EXTRA_IN,     250},   /* 带回来 */
};

/* 洞窟探杆（爪外宽 74mm，进不去 70mm 的洞，只能靠探杆） */
static const Step_t seq_cave[] = {
    {SERVO_LIFT,  ARM_TIP_HOME,       300},   /* ★ 收臂让开 */
    {SERVO_YAW,   SERVO_YAW_STOW,     600},
    {SERVO_EXTRA, SERVO_EXTRA_OUT,    250},
    {SERVO_EXTRA, SERVO_EXTRA_IN,     250},
};

static const Step_t seq_scoop_toggle[] = {
    {SERVO_SCOOP, SERVO_SCOOP_DOWN,   200},
};

typedef struct {
    const Step_t *seq;
    uint8_t       len;
} Seq_t;

#define SEQ(a)  { a, (uint8_t)(sizeof(a) / sizeof(Step_t)) }

/* ★ 索引必须与 arm.h 的 ArmPose_t 一一对应（用指定初始化器，加姿态不会错位） */
static const Seq_t g_seqs[POSE_NUM] = {
    [POSE_NONE]          = {NULL, 0},
    [POSE_STOW]          = SEQ(seq_stow),
    [POSE_READY]         = SEQ(seq_ready),
    [POSE_PICK_GROUND]   = SEQ(seq_pick_ground),
    [POSE_PICK_FOCUS]    = SEQ(seq_pick_focus),
    [POSE_PICK_PLATFORM] = SEQ(seq_pick_platform),
    [POSE_PICK_STEP]     = SEQ(seq_pick_step),
    [POSE_STORE]         = SEQ(seq_store),
    [POSE_RACK_HOOK]     = SEQ(seq_rack),
    [POSE_CAVE_PROBE]    = SEQ(seq_cave),
    [POSE_SCOOP_TOGGLE]  = SEQ(seq_scoop_toggle),
};

static uint8_t    g_busy = 0;
static uint8_t    g_idx  = 0;
static uint32_t   g_t_mark = 0;
static ArmPose_t  g_cur_pose = POSE_NONE;
static uint8_t    g_scoop_down = 0;
static uint8_t    g_yaw_locked = 0;

/* 下发一步：升降舵机走"mm → 舵机角"换算，其余直接给角度 */
static void step_apply(const Step_t *st)
{
    if (st->servo == SERVO_LIFT) Arm_SetLift(st->val);
    else                         Servo_SetAngle(st->servo, st->val);
}

void Arm_Init(void)
{
    g_busy = 0; g_idx = 0; g_cur_pose = POSE_NONE; g_scoop_down = 0; g_yaw_locked = 0;
    Arm_Start(POSE_STOW);
}

void Arm_Start(ArmPose_t pose)
{
    if ((int)pose <= 0 || (int)pose >= (int)POSE_NUM) return;
    if (g_seqs[pose].seq == NULL || g_seqs[pose].len == 0) return;

    g_cur_pose = pose;
    g_idx      = 0;
    g_busy     = 1;
    g_t_mark   = HAL_GetTick();
    /* 立刻下达第一步 */
    step_apply(&g_seqs[g_cur_pose].seq[0]);
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
    step_apply(&s->seq[g_idx]);
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

/* ---------------------------------------------------------------------------
 * ★ 柱坐标三轴的手动接口
 * -------------------------------------------------------------------------*/

/* 升降：给"爪尖离地高度 mm"，内部查标定表 → 舵机角，并做硬限位 */
void Arm_SetLift(float tip_mm)
{
    float deg = Arm_LiftDegFromMm(tip_mm);
    if (deg < ARM_LIFT_DEG_MIN) deg = ARM_LIFT_DEG_MIN;
    if (deg > ARM_LIFT_DEG_MAX) deg = ARM_LIFT_DEG_MAX;
    Servo_SetAngle(SERVO_LIFT, deg);
}

/* 直接给标定表档位（0..ARM_LIFT_N-1），标定时方便逐档对 */
void Arm_SetLiftStep(uint8_t idx)
{
    if (idx >= ARM_LIFT_N) idx = ARM_LIFT_N - 1;
    Arm_SetLift(g_lift_tip[idx]);
}

float Arm_LiftDegNow(void) { return Servo_GetAngle(SERVO_LIFT); }
float Arm_LiftMmNow(void)  { return lift_tip_from_deg(Servo_GetAngle(SERVO_LIFT)); }

/* 回转：0 = 正前，180 = 正后。
 *
 * ★★ 互锁兜底：如果**这次回转的路径**会经过车侧危险扇区（60°~120°），
 *    而当前爪尖还没抬到安全高度（ARM_TIP_SAFE 89 > 甲板 88.5），
 *    那就**先把升降抬到最高并返回**（这一拍不转）；下一拍升降还在动，
 *    判断仍成立 → 继续等；直到真的抬到位才允许转。
 *    于是天然变成"先抬后转"，绝不会低位硬转。 */
void Arm_SetYaw(float yaw_deg)
{
    float cur = Servo_GetAngle(SERVO_YAW);

    float lo = (cur < yaw_deg) ? cur : yaw_deg;
    float hi = (cur < yaw_deg) ? yaw_deg : cur;
    /* 路径 [lo,hi] 与危险扇区 [LO,HI] 有交集 -> 会路过车侧 */
    uint8_t cross = (hi > ARM_YAW_SECTOR_LO) && (lo < ARM_YAW_SECTOR_HI);

    g_yaw_locked = 0;
    if (cross && Arm_LiftMmNow() < ARM_TIP_SAFE)
    {
        Arm_SetLift(ARM_TIP_HOME);
        g_yaw_locked = 1;
        return;
    }
    Servo_SetAngle(SERVO_YAW, yaw_deg);
}

uint8_t Arm_YawLocked(void) { return g_yaw_locked; }
