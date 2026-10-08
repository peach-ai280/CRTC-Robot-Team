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
    /* ================= 宏1：地面方块自动收集 ================= */
    case MACRO_GROUND:
        switch (g_step)
        {
        case 0:   /* 前铲放下、夹爪张开、大臂到位 */
            Arm_Start(POSE_READY);
            g_mark = now_ms;
            g_step = 1;
            break;
        case 1:
            if (!Arm_IsBusy()) { g_mark = now_ms; g_step = 2; }
            else if (dt > 2000) { finish(0); }
            break;
        case 2:   /* 慢速靠拢，等对中 */
            set_cmd(AUTO_APPROACH_SPEED, 0, 0);
            if (Limit_Centered(now_ms)) { set_cmd(0, 0, 0); g_mark = now_ms; g_step = 3; }
            else if (dt > AUTO_APPROACH_TIMEOUT) finish(0);
            break;
        case 3:   /* 夹紧 -> 抬起 -> 兜入仓，整段交给 arm 序列 */
            set_cmd(0, 0, 0);
            Arm_Start(POSE_PICK_GROUND);
            g_mark = now_ms;
            g_step = 4;
            break;
        case 4:
            /* 这里的超时必须比 POSE_PICK_GROUND 序列本身长，见 config.h 的说明 */
            if (!Arm_IsBusy()) { g_mark = now_ms; g_step = 5; }
            else if (dt > AUTO_PICK_HARD_TIMEOUT) finish(0);
            break;
        case 5:   /* 退一小步，确认块真的在车上 */
            set_cmd(-AUTO_BACKUP_SPEED, 0, 0);
            if (dt > AUTO_BACKUP_MS) { set_cmd(0, 0, 0); finish(1); }
            break;
        default: finish(0); break;
        }
        break;

    /* ================= 宏2：挑物资架上的黄块 ================= */
    case MACRO_RACK:
        switch (g_step)
        {
        case 0:
            Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_OUT);
            Servo_SetAngle(SERVO_ARM,   SERVO_ARM_RACK);
            g_mark = now_ms; g_step = 1;
            break;
        case 1:
            if (!Servo_IsMoving(SERVO_ARM) && dt > AUTO_RACK_LIFT_MS)
            { g_mark = now_ms; g_step = 2; }
            else if (dt > AUTO_RACK_LIFT_MS + 1500) finish(0);
            break;
        case 2:   /* 车往前顶，让挑杆从黄块下方/侧面穿过去 */
            set_cmd(AUTO_RACK_PUSH_SPEED, 0, 0);
            if (dt > AUTO_RACK_PUSH_MS) { set_cmd(0, 0, 0); g_mark = now_ms; g_step = 3; }
            break;
        case 3:   /* 下压挑杆，把黄块从 14mm 圆柱上撬下来 */
            Servo_SetAngle(SERVO_ARM, SERVO_ARM_STEP);
            if (!Servo_IsMoving(SERVO_ARM) && dt > 400) { g_mark = now_ms; g_step = 4; }
            else if (dt > 1200) finish(0);
            break;
        case 4:   /* 倒车，块掉进车头兜里 */
            set_cmd(-AUTO_RACK_PULL_SPEED, 0, 0);
            if (dt > AUTO_RACK_PULL_MS) { set_cmd(0, 0, 0); g_mark = now_ms; g_step = 5; }
            break;
        case 5:
            Servo_SetAngle(SERVO_ARM,   SERVO_ARM_STOW);
            Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_IN);
            if (!Servo_AnyMoving() && dt > 400) finish(1);
            else if (dt > 1500) finish(0);
            break;
        default: finish(0); break;
        }
        break;

    /* ================= 宏3：洞窟探杆取块 ================= */
    case MACRO_CAVE:
        switch (g_step)
        {
        case 0:
            Servo_SetAngle(SERVO_ARM,   SERVO_ARM_GROUND);
            Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_OUT);
            g_mark = now_ms; g_step = 1;
            break;
        case 1:
            if (!Servo_AnyMoving() && dt > 500) { g_mark = now_ms; g_step = 2; }
            else if (dt > 1500) finish(0);
            break;
        case 2:   /* 探头伸进 70x70x50 的洞 */
            set_cmd(AUTO_CAVE_IN_SPEED, 0, 0);
            if (dt > AUTO_CAVE_IN_MS) { set_cmd(0, 0, 0); g_mark = now_ms; g_step = 3; }
            break;
        case 3:   /* 勾住 */
            Servo_SetAngle(SERVO_EXTRA, SERVO_EXTRA_IN);
            if (!Servo_IsMoving(SERVO_EXTRA) && dt > 400) { g_mark = now_ms; g_step = 4; }
            else if (dt > 1200) finish(0);
            break;
        case 4:   /* 拉出来 */
            set_cmd(-AUTO_CAVE_OUT_SPEED, 0, 0);
            if (dt > AUTO_CAVE_OUT_MS) { set_cmd(0, 0, 0); g_mark = now_ms; g_step = 5; }
            break;
        case 5:
            Servo_SetAngle(SERVO_ARM, SERVO_ARM_STOW);
            if (!Servo_AnyMoving() && dt > 400) finish(1);
            else if (dt > 1200) finish(0);
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
