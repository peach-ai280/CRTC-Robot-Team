/* ============================================================================
 * auto_full.c —— 全自动：在半自动宏之上加一层「自动调度」
 *
 * 一句话理解：全自动 = 巡游找块 + 自动按下宏的键 + 抓完换地方继续。
 * 抓取动作本身仍然是 auto_task.c 里那个 MACRO_GROUND，一处调好两处用。
 *
 * 状态流转：
 *
 *   OFF ──Start──> ARM ──延时──> SEARCH ──┬─看到目标──> APPROACH ──对中──> MACRO
 *                                          └─没看到────> CRUISE ─碰上───────┘
 *                                                         └─超时候 RECOVER ─┐
 *   MACRO ──成功──> RECOVER ──> SEARCH（继续找下一个）                       │
 *          └─失败──> RECOVER ──> SEARCH（连败 N 次就 DONE）                 ←┘
 *   任意状态：总时长到 / 连败到上限 ──> DONE
 *
 * 【开环的诚实说明】
 *   我们没有编码器，所有"走了多远"都是靠时间估的。所以巡游不是精确覆盖，
 *   是"大概扫一遍"。这没关系——目标是多拿块，不是走整齐的轨迹。
 *   所有时间参数都要你在场地上掐表标定，config.h 里每个都写了怎么标。
 * ==========================================================================*/

#include "auto_full.h"
#include "auto_task.h"
#include "arm.h"
#include "servo.h"
#include "sensor.h"
#include "vision.h"
#include "mecanum.h"
#include "config.h"
#include <math.h>

/* -------------------------------------------------------------------------- */
static FullState_t g_state  = FA_OFF;
static uint32_t    g_mark   = 0;      /* 当前状态的起始时刻 */
static uint32_t    g_t0     = 0;      /* 全自动启动时刻（算总时长用） */
static uint8_t     g_picked = 0;      /* 成功取到几个 */
static uint8_t     g_fail   = 0;      /* 连续失败次数 */
static uint8_t     g_leg    = 0;      /* 巡游第几条腿 */
static int8_t      g_dir    = 1;      /* 横移方向 +1 / -1 */
static int8_t      g_fwd    = 1;      /* 纵向方向 +1 前进 / -1 后退 */
static float       g_cmd[3] = {0, 0, 0};

static void set_cmd(float vx, float vy, float wz)
{
    g_cmd[0] = vx; g_cmd[1] = vy; g_cmd[2] = wz;
}

static void go(FullState_t s, uint32_t now)
{
    g_state = s;
    g_mark  = now;
    set_cmd(0, 0, 0);
}

/* -------------------------------------------------------------------------- */
void FullAuto_Init(void)
{
    g_state = FA_OFF; g_picked = 0; g_fail = 0;
    g_leg = 0; g_dir = 1; g_fwd = 1;
    set_cmd(0, 0, 0);
}

uint8_t     FullAuto_Running(void) { return (g_state != FA_OFF) ? 1 : 0; }
FullState_t FullAuto_State(void)   { return g_state; }
uint8_t     FullAuto_Picked(void)  { return g_picked; }
uint8_t     FullAuto_Failed(void)  { return g_fail; }

void FullAuto_Start(void)
{
    if (g_state != FA_OFF) return;
    g_picked = 0; g_fail = 0; g_leg = 0; g_dir = 1; g_fwd = 1;
    g_t0 = HAL_GetTick();
    Arm_Start(POSE_STOW);                 /* 先收机构，免得张着爪子到处撞 */
    go(FA_ARM, HAL_GetTick());
}

void FullAuto_Stop(void)
{
    if (g_state == FA_OFF) return;
    Auto_Abort();                          /* 正在跑的宏一并中止 */
    Arm_Start(POSE_STOW);
    set_cmd(0, 0, 0);
    g_state = FA_OFF;
}

void FullAuto_GetCmd(float *vx, float *vy, float *wz)
{
    if (vx) *vx = g_cmd[0];
    if (vy) *vy = g_cmd[1];
    if (wz) *wz = g_cmd[2];
}

const char *FullAuto_StateName(void)
{
    switch (g_state)
    {
    case FA_OFF:      return "OFF";
    case FA_ARM:      return "ARM";
    case FA_SEARCH:   return "SEARCH";
    case FA_CRUISE:   return "CRUISE";
    case FA_APPROACH: return "APPROACH";
    case FA_MACRO:    return "MACRO";
    case FA_RECOVER:  return "RECOVER";
    case FA_DONE:     return "DONE";
    default:          return "?";
    }
}

/* -------------------------------------------------------------------------- */
void FullAuto_Update(uint32_t now)
{
    if (g_state == FA_OFF) { set_cmd(0, 0, 0); return; }

    uint32_t dt   = now - g_mark;                 /* 在本状态待了多久 */
    uint32_t total = now - g_t0;                  /* 全自动跑了多久 */

    /* ---- 总时长硬上限：到点无条件停，别把一局 5 分钟跑超 ---- */
    if (total > AUTO_TOTAL_LIMIT_MS && g_state != FA_DONE)
    {
        Auto_Abort(); Arm_Start(POSE_STOW);
        go(FA_DONE, now);
        return;
    }

    switch (g_state)
    {
    /* ============ ARM：预备，给人走开的时间 ============ */
    case FA_ARM:
        set_cmd(0, 0, 0);
        if (dt > AUTO_FULL_ARM_DELAY_MS)
        {
#if USE_VISION
            go(FA_SEARCH, now);
#else
            go(FA_CRUISE, now);        /* 没视觉就没必要原地转，直接扫 */
#endif
        }
        break;

    /* ============ SEARCH：原地慢转找目标（只有 USE_VISION=1 才会进来） ===== */
    case FA_SEARCH:
#if USE_VISION
        set_cmd(0, 0, AUTO_SEARCH_SPIN_SPEED);
        if (Vision_Valid()) { set_cmd(0, 0, 0); go(FA_APPROACH, now); }
        else if (dt > AUTO_SEARCH_SPIN_MS) { set_cmd(0, 0, 0); go(FA_CRUISE, now); }
#else
        go(FA_CRUISE, now);
#endif
        break;

    /* ============ CRUISE：蛇形盲扫，靠对中限位触发 ============ */
    case FA_CRUISE:
    {
        /* 有视觉时优先跟视觉 */
#if USE_VISION
        if (Vision_Valid()) { set_cmd(0, 0, 0); go(FA_APPROACH, now); break; }
#endif
        /* 撞上对中限位 = 铲口里有东西了，转入靠拢 */
        if (Limit_Centered(now)) { set_cmd(0, 0, 0); go(FA_APPROACH, now); break; }

        /* 蛇形：偶数腿走纵向，奇数腿横移换道 */
        if (g_leg % 2 == 0)
        {
            set_cmd(AUTO_CRUISE_SPEED * (float)g_fwd, 0, 0);
            if (dt > AUTO_CRUISE_FWD_MS)
            {
                set_cmd(0, 0, 0);
                g_leg++;
                go(FA_CRUISE, now);                 /* 换腿，重新计时 */
            }
        }
        else
        {
            set_cmd(0, AUTO_CRUISE_STRAFE_SPEED * (float)g_dir, 0);
            if (dt > AUTO_CRUISE_STRAFE_MS)
            {
                set_cmd(0, 0, 0);
                g_leg++;
                /* 走满一个来回就掉头：纵向反向 + 横移保持，形成往复覆盖 */
                if (g_leg >= AUTO_CRUISE_LANES * 2)
                {
                    g_leg = 0;
                    g_fwd = -g_fwd;
                    g_dir = -g_dir;
                }
                go(FA_CRUISE, now);
            }
        }
        break;
    }

    /* ============ APPROACH：靠拢，直到能夹 ============ */
    case FA_APPROACH:
    {
        /* 判定 1：机械对中已成（最可靠，不依赖光照） */
        if (Limit_Centered(now))
        {
            set_cmd(0, 0, 0);
            Auto_Start(MACRO_GROUND);
            go(FA_MACRO, now);
            break;
        }

#if USE_VISION
        /* 判定 2：视觉说够近了（色块面积够大） */
        if (Vision_Valid() && vision.area > VISION_NEAR_AREA)
        {
            set_cmd(0, 0, 0);
            Auto_Start(MACRO_GROUND);
            go(FA_MACRO, now);
            break;
        }

        /* 视觉在：用 P 控制修正横向偏差 */
        if (Vision_Valid())
        {
            float turn = VISION_TURN_KP * (float)vision.x;
            if (turn >  VISION_TURN_MAX) turn =  VISION_TURN_MAX;
            if (turn < -VISION_TURN_MAX) turn = -VISION_TURN_MAX;
            set_cmd(AUTO_APPROACH_SPEED, 0, turn);
        }
        else
#endif
        {
            /* 纯盲推：慢速直顶，等对中限位 */
            set_cmd(AUTO_APPROACH_SPEED, 0, 0);
        }

        if (dt > AUTO_APPROACH_TIMEOUT)          /* 顶太久 = 没对上，放弃 */
        {
            set_cmd(0, 0, 0);
            g_fail++;
            go(FA_RECOVER, now);
        }
        break;
    }

    /* ============ MACRO：把控制权交给半自动宏，等它跑完 ============ */
    case FA_MACRO:
    {
        if (!Auto_Running())                      /* 宏已经结束（成功或超时） */
        {
            if (Auto_LastResult()) { g_picked++; g_fail = 0; }
            else                   { g_fail++; }
            go(FA_RECOVER, now);
            break;
        }

        /* 关键：宏的更新在这里驱动，app.c 不要再调一次 Auto_Update，
           否则一个周期推进两步，动作会乱 */
        Auto_Update(now);
        {
            float vx, vy, wz;
            Auto_GetCmd(&vx, &vy, &wz);
            set_cmd(vx, vy, wz);
        }

        /* 宏本身有硬超时，这里是双保险 */
        if (dt > AUTO_MACRO_HARD_TIMEOUT) { Auto_Abort(); g_fail++; go(FA_RECOVER, now); }
        break;
    }

    /* ============ RECOVER：收机构 + 退一步，防止块卡在铲口 ============ */
    case FA_RECOVER:
        Arm_Start(POSE_STOW);
        set_cmd(-AUTO_RECOVER_BACK_SPEED, 0, 0);
        if (dt > AUTO_RECOVER_BACK_MS)
        {
            set_cmd(0, 0, 0);
            if (g_fail >= AUTO_FAIL_LIMIT || g_picked >= AUTO_MAX_PICK)
                go(FA_DONE, now);
            else
            {
#if USE_VISION
                go(FA_SEARCH, now);
#else
                go(FA_CRUISE, now);
#endif
            }
        }
        break;

    /* ============ DONE：停车等人接管 ============ */
    case FA_DONE:
    default:
        set_cmd(0, 0, 0);
        break;
    }
}
