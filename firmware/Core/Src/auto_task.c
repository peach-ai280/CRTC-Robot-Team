/* ============================================================================
 * auto_task.c —— 半自动宏状态机
 *
 * 每个宏都由若干个 step 组成，每个 step 的输出是「底盘速度指令 + 机构动作」。
 * step 之间靠两种条件推进：
 *   - 时间到（AUTO_xxx_MS，config.h 里调）
 *   - 传感器事件（对中限位闭合 / 舵机动作序列跑完）
 * 任何一个 step 超时都会跳到 FAIL 分支：立刻停车 + 机构收起，绝不允许顶着电机。
 * ==========================================================================*/

#include "auto_task.h"
#include "arm.h"
#include "servo.h"
#include "sensor.h"
#include "vision.h"
#include "config.h"
#include <math.h>

static Macro_t   g_macro = MACRO_NONE;
static uint8_t   g_step  = 0;
static uint32_t  g_mark  = 0;
static uint8_t   g_result = 0;
static uint8_t   g_cnt   = 0;      /* 通用计数器（抖动送料剩下的次数 / 重试次数） */
static float     g_cmd[3] = {0, 0, 0};

static void set_cmd(float vx, float vy, float wz)
{
    g_cmd[0] = vx; g_cmd[1] = vy; g_cmd[2] = wz;
}

void Auto_Init(void)
{
    g_macro = MACRO_NONE; g_step = 0; g_result = 0;
    set_cmd(0, 0, 0);
}

uint8_t Auto_Running(void) { return (g_macro != MACRO_NONE) ? 1 : 0; }
Macro_t Auto_Current(void) { return g_macro; }
uint8_t Auto_LastResult(void) { return g_result; }

void Auto_Start(Macro_t m)
{
    if (g_macro != MACRO_NONE) return;   /* 宏不嵌套 */
    g_macro = m;
    g_step  = 0;
    g_mark  = HAL_GetTick();
    g_result = 0;
    set_cmd(0, 0, 0);
}

static void finish(uint8_t ok)
{
    g_result = ok;
    g_macro  = MACRO_NONE;
    g_step   = 0;
    set_cmd(0, 0, 0);
    if (!ok) Arm_Start(POSE_STOW);       /* 失败也要把机构收回来，别张着爪子到处跑 */
}

void Auto_Abort(void)
{
    if (g_macro == MACRO_NONE) return;
    finish(0);
}

void Auto_GetCmd(float *vx, float *vy, float *wz)
{
    if (vx) *vx = g_cmd[0];
    if (vy) *vy = g_cmd[1];
    if (wz) *wz = g_cmd[2];
}

/* --------------------------------------------------------------------------- */
void Auto_Update(uint32_t now_ms)
{
    if (g_macro == MACRO_NONE) { set_cmd(0, 0, 0); return; }

    uint32_t dt = now_ms - g_mark;

    switch (g_macro)
    {
    /* ================= 宏1：地面方块收集（★ 2026-10-17 按翻转式四齿耙重写）===
     *
     * 【为什么整段重写】
     *   旧写法是「限位对中 → Arm_Start(POSE_PICK_GROUND) 让机械臂夹」。
     *   但柱坐标臂的爪尖最低只到离地 64.5，而地面方块顶面才 30 —— **差 34.5mm，
     *   臂物理上够不到地面块**。旧宏每一次都必然抓空，还会白等 5 秒。
     *
     * 【新原理：不清扫楔入，靠"搂"】
     *   方块贴着地面，底下零间隙 ⇒ 齿尖不可能插进去（楔入这条路不通）。
     *   唯一可行的路径是：**把块顶/扫到齿前 → 快速翻转 120° → 块被齿和挡唇
     *   一起"搂"到背板上 → 背板变成托盘，块躺在里面**，兜住即算「携带」。
     *
     * 【六段动作，每段的参数都在 config.h 的 SCOOP_* 里】
     *   ⓪ 退开   先倒一点，拉开助跑距离（★ 这一步是全自动模式必须的，见下）
     *   ① 贴地   铲压到齿尖最低
     *   ② 扫掠   0.18 覆盖一片，把散布的块归拢到齿前
     *   ③ 爬行顶 0.14 极慢贴住块（★ 这一段决定块到底在不在齿前，最要掐表）
     *   ④ 翻转   150°→30° 一把翻上来（120°/240dps ≈ 0.5s）
     *   ⑤ 抖动   FLIP↔HOLD 交替，靠惯性把块推到背板根部（★ 不需要任何新零件）
     *   ⑥ 后退   退一步，确认块没被场地边框刮掉
     *
     * ★ 为什么开头要"先退开"：
     *   半自动时操作手是把车停在块前方 200mm 左右再按 ○，扫掠正好把块收进齿前；
     *   但**全自动**（auto_full.c）是"车头撞上对中限位"才触发宏 —— 那时车已经
     *   贴住块了，再往前扫 240mm 会直接把块顶飞。统一先退 200mm，两种启动方式
     *   就都落在同一个起跑线上。代价只是每次多 1 秒。
     *
     * ⚠ 没有块在位传感器（车上一共就两个对中限位开关，装在车头两侧）。
     *   所以这段是**纯开环盲动作**：参数不准 → 收不到块，但不会烧东西。
     *   判断成功与否只能靠肉眼。**别连着按第三次**，铲会溢出把块推掉。
     * =======================================================================*/
    case MACRO_GROUND:
        switch (g_step)
        {
        case 0:   /* ① 铲压到贴地位（齿尖最低）。臂不用动 —— 臂在车中，铲在车头，不打架 */
            Servo_SetAngle(SERVO_SCOOP, SCOOP_ANG_GROUND);
            g_cnt  = SCOOP_SHAKE_TIMES;      /* 剩余抖动次数 */
            g_mark = now_ms;
            g_step = 1;
            break;
        case 1:   /* 等铲到位（150° 这个位置行程长，给 2.5s 富余） */
            if (!Servo_IsMoving(SERVO_SCOOP) && dt > 150) { g_mark = now_ms; g_step = 2; }
            else if (dt > 2500) finish(0);
            break;
        case 2:   /* ② 先退开，拉开助跑距离（全自动触发时车是贴着块的） */
            set_cmd(-SCOOP_BACK_SPEED, 0, 0);
            if (dt > SCOOP_BACKOFF_MS) { g_mark = now_ms; g_step = 3; }
            break;
        case 3:   /* ③ 扫掠：低速直线扫过去，把块归拢到齿前 */
            set_cmd(SCOOP_SWEEP_SPEED, 0, 0);
            if (dt > SCOOP_SWEEP_MS) { g_mark = now_ms; g_step = 4; }
            break;
        case 4:   /* ④ 爬行顶：极慢，让挡唇"贴住"块（快了会把块顶飞，慢了对不上） */
            set_cmd(SCOOP_PUSH_SPEED, 0, 0);
            if (dt > SCOOP_PUSH_MS) { set_cmd(0, 0, 0); g_mark = now_ms; g_step = 5; }
            break;
        case 5:   /* ⑤ 快速翻转：一把把块搂到背板上（车此时静止，别边走边翻） */
            Servo_SetAngle(SERVO_SCOOP, SCOOP_ANG_FLIP);
            if (!Servo_IsMoving(SERVO_SCOOP) && dt > SCOOP_FLIP_SETTLE_MS)
            { g_mark = now_ms; g_step = 6; }
            else if (dt > SCOOP_FLIP_SETTLE_MS + 1500) finish(0);
            break;
        case 6:   /* ⑥ 抖动送料：转到 HOLD 停一下；还有次数就回 FLIP，再翻一次 */
            Servo_SetAngle(SERVO_SCOOP, SCOOP_ANG_HOLD);
            if (!Servo_IsMoving(SERVO_SCOOP) && dt > SCOOP_SHAKE_MS)
            {
                if (g_cnt > 0) { g_cnt--; Servo_SetAngle(SERVO_SCOOP, SCOOP_ANG_FLIP); g_mark = now_ms; g_step = 7; }
                else           { g_mark = now_ms; g_step = 8; }
            }
            else if (dt > SCOOP_SHAKE_MS + 2000) { g_mark = now_ms; g_step = 8; }
            break;
        case 7:   /* 抖动中的「回翻」半拍：回到 HOLD 就回 step 6 */
            if (!Servo_IsMoving(SERVO_SCOOP) && dt > SCOOP_SHAKE_MS) { g_mark = now_ms; g_step = 6; }
            else if (dt > SCOOP_SHAKE_MS + 1500) { g_mark = now_ms; g_step = 6; }
            break;
        case 8:   /* ⑦ 退一步：离开场地边框/矮墙，确认块还在（在不在只能肉眼看） */
            set_cmd(-SCOOP_BACK_SPEED, 0, 0);
            if (dt > SCOOP_BACK_MS) { set_cmd(0, 0, 0); finish(1); }
            break;
        default: finish(0); break;
        }
        break;

    /* ================= 宏2：取物资架上的黄块（★ 2026-10-10 按柱坐标重写）=====
     *   ⚠ 柱坐标臂**根本够不到**物资架黄块：爪尖最高才 95，黄块质心离地 250，**差 155mm**。
     *     所以这个宏**不再让机械臂参与**，改成「收臂让开 → 挑杆伸到黄块下方 →
     *     车慢慢前顶 → 挑杆收回、把黄块沿 φ14 圆柱往外拨 → 倒车把块带进车头铲」。
     *   ⚠ 这是**全车最不确定的一段**，必须实车标定 AUTO_RACK_* 那几个时间。
     *     **现场拿不下来就直接放弃**，别为 5 分超时丢了一整局。 */
    case MACRO_RACK:
        switch (g_step)
        {
        case 0:   /* ① 收臂让开 + 挑杆收回 + **铲先压到贴地位** */
            /* ★ 为什么铲要提前贴地：挑杆把黄块从 250 高拨下来之后，块是落在地面上的，
             *   落下后要按 ○（MACRO_GROUND）让铲去扫。铲这时候如果还竖着（检录位），
             *   操作手按下 ○ 会白等半秒才压到位。提前压好，拨完直接按 ○。 */
            Arm_SetLift(ARM_TIP_HOME);
            Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_IN);
            Servo_SetAngle(SERVO_SCOOP, SCOOP_ANG_GROUND);
            g_mark = now_ms; g_step = 1;
            break;
        case 1:   /* 等升降到位，再转正后方（Arm_SetYaw 的互锁天然生效：低位不许转） */
            if (!Servo_IsMoving(SERVO_LIFT) && dt > 300) { Arm_SetYaw(SERVO_YAW_STOW); g_mark = now_ms; g_step = 2; }
            else if (dt > 2500) finish(0);
            break;
        case 2:   /* ② 转到位后再把挑杆摆出去 */
            if (!Servo_IsMoving(SERVO_YAW) && dt > 300) { Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_OUT); g_mark = now_ms; g_step = 3; }
            else if (dt > 2500) finish(0);
            break;
        case 3:   /* 挑杆伸到位 */
            if (!Servo_IsMoving(SERVO_EXTRA) && dt > 300) { g_mark = now_ms; g_step = 4; }
            else if (dt > 1500) finish(0);
            break;
        case 4:   /* ③ 车慢慢往前顶，让挑杆伸到黄块下面 */
            set_cmd(AUTO_RACK_PUSH_SPEED, 0, 0);
            if (dt > AUTO_RACK_PUSH_MS) { set_cmd(0, 0, 0); g_mark = now_ms; g_step = 5; }
            break;
        case 5:   /* ④ 挑杆收回：把黄块沿 φ14 圆柱往外拨出来 */
            Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_IN);
            if (!Servo_IsMoving(SERVO_EXTRA) && dt > 500) { g_mark = now_ms; g_step = 6; }
            else if (dt > 1500) finish(0);
            break;
        case 6:   /* ⑥ 倒车拉开距离，把黄块留在车前方地面上 */
            /* ★ 不在这里收块：块是从 250 高掉下来的，弹跳落点不可控（散布 ±300mm），
             *   盲动作去"搂"成功率很低。**退到块后方，交回操作手看一眼再按 ○** ——
             *   这就是"半自动"的正确用法：人负责判断块在哪，机器负责把动作做准。
             *   退够了距离之后，按 ○ 走的就是标准扫掠+铲翻流程，一次到位。 */
            set_cmd(-AUTO_RACK_PULL_SPEED, 0, 0);
            if (dt > AUTO_RACK_PULL_MS) { set_cmd(0, 0, 0); finish(1); }
            break;
        default: finish(0); break;
        }
        break;

    /* ================= 宏3：洞窟探杆取块 =================
     *   柱坐标臂的爪外宽 74mm，进不去 70mm 的洞 —— 只能靠探杆。
     *   顺序不变：**先升最高 + 转正后让开空间，再摆探杆**（Arm_SetYaw 的互锁会兜底）。 */
    case MACRO_CAVE:
        switch (g_step)
        {
        case 0:   /* ① 先把滑座升到最高（回转安全高度），顺便把探杆收回 */
            Arm_SetLift(ARM_TIP_HOME);
            Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_IN);
            g_mark = now_ms; g_step = 1;
            break;
        case 1:   /* 等升降到位，再转正后方 */
            if (!Servo_IsMoving(SERVO_LIFT) && dt > 300) { Arm_SetYaw(SERVO_YAW_STOW); g_mark = now_ms; g_step = 2; }
            else if (dt > 2500) finish(0);
            break;
        case 2:   /* ② 转到位后再把探杆摆出去 */
            if (!Servo_IsMoving(SERVO_YAW) && dt > 300) { Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_OUT); g_mark = now_ms; g_step = 3; }
            else if (dt > 2500) finish(0);
            break;
        case 3:   /* 探杆伸到位 */
            if (!Servo_IsMoving(SERVO_EXTRA) && dt > 300) { g_mark = now_ms; g_step = 4; }
            else if (dt > 1500) finish(0);
            break;
        case 4:   /* ③ 车前进，探头伸进 70x70x50 的洞 */
            set_cmd(AUTO_CAVE_IN_SPEED, 0, 0);
            if (dt > AUTO_CAVE_IN_MS) { set_cmd(0, 0, 0); g_mark = now_ms; g_step = 5; }
            break;
        case 5:   /* ④ 勾住 */
            Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_IN);
            if (!Servo_IsMoving(SERVO_EXTRA) && dt > 400) { g_mark = now_ms; g_step = 6; }
            else if (dt > 1200) finish(0);
            break;
        case 6:   /* ⑤ 倒车把块拉出来 */
            set_cmd(-AUTO_CAVE_OUT_SPEED, 0, 0);
            if (dt > AUTO_CAVE_OUT_MS) { set_cmd(0, 0, 0); finish(1); }
            break;
        default: finish(0); break;
        }
        break;

    /* ================= 宏4：一键贴边（盲走，时间参数必须实车标定） ========= */
    case MACRO_EDGE:
        switch (g_step)
        {
        case 0:
            set_cmd(0, AUTO_EDGE_SPEED, 0);      /* 向左横移贴边，方向反了改符号 */
            if (dt > AUTO_EDGE_TIMEOUT) { set_cmd(0, 0, 0); finish(1); }
            break;
        default: finish(0); break;
        }
        break;

    /* ================= 宏5：视觉辅助对位（进阶） ================= */
    case MACRO_VISION:
#if USE_VISION
        switch (g_step)
        {
        case 0:
            Arm_Start(POSE_READY);
            g_mark = now_ms; g_step = 1;
            break;
        case 1:
            if (!Arm_IsBusy()) { g_mark = now_ms; g_step = 2; }
            else if (dt > 2000) finish(0);
            break;
        case 2:
        {
            if (!Vision_Valid()) { set_cmd(0, 0, 0); if (dt > 2500) finish(0); break; }

            /* 已经很近了（色块面积够大）-> 直接夹 */
            if (vision.area > VISION_NEAR_AREA)
            {
                set_cmd(0, 0, 0);
                Arm_Start(POSE_PICK_GROUND);
                g_mark = now_ms; g_step = 3;
                break;
            }

            /* 横向偏差 P 控制：x>0 表示目标在画面右边 -> 车要往右转 */
            float err = (float)vision.x;
            if (err < 0) err = -err;
            float turn = 0.0f;
            if (err > (float)VISION_X_DEADZONE)
                turn = VISION_TURN_KP * (float)vision.x;
            if (turn >  VISION_TURN_MAX) turn =  VISION_TURN_MAX;
            if (turn < -VISION_TURN_MAX) turn = -VISION_TURN_MAX;

            set_cmd(VISION_APPROACH_SPEED, 0, turn);
            if (dt > 4000) finish(0);
            break;
        }
        case 3:
            set_cmd(0, 0, 0);
            if (!Arm_IsBusy()) { g_mark = now_ms; g_step = 4; }
            else if (dt > AUTO_PICK_HARD_TIMEOUT) finish(0);
            break;
        case 4:
            set_cmd(-AUTO_BACKUP_SPEED, 0, 0);
            if (dt > AUTO_BACKUP_MS) { set_cmd(0, 0, 0); finish(1); }
            break;
        default: finish(0); break;
        }
#else
        finish(0);
#endif
        break;

    default:
        finish(0);
        break;
    }
}
