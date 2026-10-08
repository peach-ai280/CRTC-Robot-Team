/* ============================================================================
 * sim_auto.c —— 在 ARM 模拟器里跑「全自动状态机」
 *
 * 目的：验证 auto_full.c 的**编排逻辑**不会卡死、不会跳过状态、不会漏计数。
 * 抓取动作本身（arm/servo）用假实现顶替 —— 那部分已经在 C++ 版验证过了，
 * 这里要测的是"状态怎么流转"这一层新代码。
 *
 * 两个场景一次跑完，结果分别放 g_outA / g_outB：
 *   A（顺利）：每次靠拢 1200ms 后对中成功 → 应该拿满 AUTO_MAX_PICK 个后收工
 *   B（全空）：永远对不上 → 每次靠拢超时 → 连败 AUTO_FAIL_LIMIT 次后放弃
 * ==========================================================================*/

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "auto_task.h"
#include "auto_full.h"
#include "arm.h"
#include "sensor.h"
#include "servo.h"
#include "vision.h"

/* ---------------- 模拟器提供的假硬件 ---------------- */
static uint32_t g_now = 0;
uint32_t HAL_GetTick(void) { return g_now; }

/* 机构：Start 之后"忙"一段时间就完事。
   g_arm_stuck=1 时永远忙不完 —— 用来模拟"舵机卡死/堵转"，验证连败放弃逻辑 */
static uint32_t g_busy_until = 0;
static int      g_arm_stuck  = 0;
void Arm_Init(void) {}
void Arm_Start(ArmPose_t pose)
{
    if (g_arm_stuck) { g_busy_until = 0xFFFFFFFFu; return; }
    uint32_t d = 800;                       /* 默认 */
    if (pose == POSE_PICK_GROUND) d = 3900; /* 实测地面取块约 3970ms */
    else if (pose == POSE_READY)  d = 900;
    else if (pose == POSE_STORE)  d = 1200;
    g_busy_until = g_now + d;
}
void      Arm_Update(uint32_t now) { (void)now; }
uint8_t   Arm_IsBusy(void)         { return (g_now < g_busy_until) ? 1 : 0; }
ArmPose_t Arm_Current(void)        { return POSE_NONE; }
void      Arm_Grip(float a)        { (void)a; }
void      Arm_ScoopDown(uint8_t d) { (void)d; }

/* 舵机：设定即到位（这里不关心角度曲线） */
static float g_sv[4];
void    Servo_Init(void) {}
void    Servo_SetAngle(ServoId_t id, float a) { if (id < 4) g_sv[id] = a; }
void    Servo_SetNow(ServoId_t id, float a)   { if (id < 4) g_sv[id] = a; }
void    Servo_Update(float dt)                { (void)dt; }
float   Servo_GetAngle(ServoId_t id)          { return (id < 4) ? g_sv[id] : 0; }
uint8_t Servo_IsMoving(ServoId_t id)          { (void)id; return 0; }
uint8_t Servo_AnyMoving(void)                 { return 0; }
void    Servo_Disable(ServoId_t id)           { (void)id; }

/* 传感器：靠拢累计时间够了就"对中成功"（场景 A），或永远不成功（场景 B） */
static uint32_t g_center_after = 0;    /* 0 = 永不对中 */
static uint32_t g_push_ms      = 0;    /* 当前这次靠拢已推进多久 */
void    Sensor_Init(void) {}
void    Sensor_Update(uint32_t now) { (void)now; }
uint8_t Limit_Left(void)  { return 0; }
uint8_t Limit_Right(void) { return 0; }
uint8_t Limit_Centered(uint32_t now)
{
    (void)now;
    if (g_center_after == 0) return 0;
    return (g_push_ms >= g_center_after) ? 1 : 0;
}
float   Sensor_GetVbat(void)    { return 7.8f; }
uint8_t Sensor_KeyPressed(void) { return 0; }

/* 视觉：这里按"没买视觉模块"来测（USE_VISION=0）。
   带视觉的路径由 C++ 版 sim 覆盖（那里有 Vision_Valid 的真逻辑）。 */
Vision_t vision;
void    Vision_Init(void) {}
void    Vision_RxByte(uint8_t b) { (void)b; }
void    Vision_Update(uint32_t now) { (void)now; }
uint8_t Vision_Valid(void) { return 0; }

/* ---------------- 结果区：脚本从内存里读 ---------------- */
typedef struct {
    uint32_t ticks;          /* 模拟了多少 ms */
    int      visit[16];      /* 各状态被访问的周期数 */
    int      picked;
    int      fail;
    int      final_state;
    int      cmd_nonzero;    /* 有输出指令的周期数（证明车真的在动） */
    int      max_dwell_ms;   /* 在单个状态里停留的最长时间 */
    int      dwell_of_max;   /* 是哪个状态 */
    int      transitions;    /* 状态切换次数 */
} SimOut;

volatile SimOut g_outA;
volatile SimOut g_outB;
volatile SimOut g_outC;
volatile int    g_done = 0;     /* main 跑完置 1，脚本据此判断没死循环 */

static void run_scene(int center_after_ms, uint32_t limit_ms, int arm_stuck,
                      volatile SimOut *out)
{
    memset((void *)out, 0, sizeof(SimOut));

    g_now = 0;
    g_center_after = (uint32_t)center_after_ms;
    g_push_ms = 0;
    g_busy_until = 0;
    g_arm_stuck = arm_stuck;

    FullAuto_Init();
    Auto_Init();
    FullAuto_Start();

    int      prev_state  = -1;
    uint32_t state_enter = 0;

    for (uint32_t t = 0; t < limit_ms; t += 5)
    {
        g_now = t;

        float vx = 0, vy = 0, wz = 0;
        FullAuto_Update(t);
        FullAuto_GetCmd(&vx, &vy, &wz);

        /* 只有车在往前顶的时候才累计"推进量"（模拟铲口逼近方块） */
        if (vx > 0.05f) g_push_ms += 5; else g_push_ms = 0;

        int st = (int)FullAuto_State();
        if (st != prev_state)
        {
            out->transitions++;
            int dwell = (int)(t - state_enter);
            if (dwell > out->max_dwell_ms)
            {
                out->max_dwell_ms = dwell;
                out->dwell_of_max = prev_state;
            }
            state_enter = t;
            prev_state  = st;
        }
        if (st >= 0 && st < 16) out->visit[st]++;
        if (fabsf(vx) > 0.01f || fabsf(vy) > 0.01f || fabsf(wz) > 0.01f)
            out->cmd_nonzero++;

        out->ticks = t;
        if (st == (int)FA_DONE && t > 5000) break;
    }

    out->picked      = FullAuto_Picked();
    out->fail        = FullAuto_Failed();
    out->final_state = (int)FullAuto_State();
}

int main(void)
{
    /* A 顺利：靠拢 1200ms 对中成功 → 应拿满 AUTO_MAX_PICK 个后收工 */
    run_scene(1200, 200000, 0, &g_outA);
    /* B 空场：永远碰不到块 → 一直巡游，靠总时长上限 AUTO_TOTAL_LIMIT_MS 收尾 */
    run_scene(0,    300000, 0, &g_outB);
    /* C 舵机卡死：对中了但宏每次超时 → 连败 AUTO_FAIL_LIMIT 次后放弃 */
    run_scene(1200, 200000, 1, &g_outC);
    g_done = 1;
    return 0;
}
