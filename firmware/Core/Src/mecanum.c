/* ============================================================================
 * mecanum.c —— 底盘防滑的“主战场”
 *
 * 【为什么麦轮一定要做归一化限幅（第 3 步）】
 *   斜着走的时候四个轮的解算值不一样大。如果简单粗暴地给每个轮单独限幅，
 *   超出去的那个轮被截掉，合成速度方向就歪了 —— 表现是“明明推的是 45°，
 *   车却走成了 30°”，而且转弯时车会自己画弧。
 *   正确做法是：算出四个轮里的最大绝对值，如果超过 1，就整体等比缩放。
 *   这样速度矢量的**方向**严格保持，只是整体慢一点。这一个处理，
 *   能让新手车和老手车的操控手感拉开档次。
 *
 * 【斜坡限幅（第 2 步）是怎么防滑的】
 *   TT 马达扭矩不小，一给满占空比，轮子和地胶之间先突破静摩擦 → 打滑，
 *   而且麦轮的辊子本身是塑料的、接触面又小，比普通橡胶轮更容易滑。
 *   把速度变化率限制住，让轮子始终工作在“滚动摩擦”区，比事后补偿有效得多。
 *
 * 【标定顺序】
 *   先关掉斜坡（把 ACC_UP/DOWN 都设成 99）确认方向和不打滑，再逐步收紧。
 * ==========================================================================*/

#include "mecanum.h"
#include "motor.h"
#include "board.h"
#include "config.h"
#include <math.h>

static float g_tgt[3] = {0, 0, 0};     /* 目标 vx, vy, wz */
static float g_cur[3] = {0, 0, 0};     /* 斜坡后的实际值 */
static float g_wheel[4] = {0, 0, 0, 0};
static uint8_t g_emergency = 0;

/* (a+b)·ωmax / vmax：把自转归一化到和直线速度同一量纲 */
static const float YAW_GEOM =
    ((ROBOT_HALF_WHEELBASE + ROBOT_HALF_TRACK) * CHASSIS_W_MAX) / CHASSIS_V_MAX;

void Chassis_Init(void)
{
    g_tgt[0] = g_tgt[1] = g_tgt[2] = 0;
    g_cur[0] = g_cur[1] = g_cur[2] = 0;
    g_wheel[0] = g_wheel[1] = g_wheel[2] = g_wheel[3] = 0;
    g_emergency = 0;
    Motor_Init();
}

void Chassis_SetTarget(float vx, float vy, float wz)
{
    /* 摇杆符号修正（整车装反了就改 config.h 的三个 SIGN） */
    g_tgt[0] = vx * (float)FORWARD_SIGN * SPEED_GAIN_FORWARD;
    g_tgt[1] = vy * (float)STRAFE_SIGN  * SPEED_GAIN_STRAFE;
    g_tgt[2] = wz * (float)YAW_SIGN     * SPEED_GAIN_YAW;

    for (uint8_t i = 0; i < 3; i++)
    {
        if (g_tgt[i] >  1.0f) g_tgt[i] =  1.0f;
        if (g_tgt[i] < -1.0f) g_tgt[i] = -1.0f;
    }
}

/* ---------------------------------------------------------------------------
 * 第 1 步：斜坡限幅（加速度限制）—— 防滑的第一道防线
 * -------------------------------------------------------------------------*/
static void chassis_ramp(float dt)
{
    const float acc_up[3] = {CHASSIS_ACC_UP, CHASSIS_ACC_UP, CHASSIS_YAW_ACC_UP};
    const float acc_dn[3] = {CHASSIS_ACC_DOWN, CHASSIS_ACC_DOWN, CHASSIS_YAW_ACC_DOWN};

    for (uint8_t i = 0; i < 3; i++)
    {
        float d = g_tgt[i] - g_cur[i];
        /* 增大幅度和减小幅度用不同的上限：起步慢、刹车快，符合直觉也更稳 */
        float lim = (fabsf(g_tgt[i]) > fabsf(g_cur[i]) ? acc_up[i] : acc_dn[i]) * dt;
        if (lim < 0.0001f) lim = 0.0001f;

        if      (d >  lim) d =  lim;
        else if (d < -lim) d = -lim;

        g_cur[i] += d;
        if (fabsf(g_cur[i]) < 0.001f) g_cur[i] = 0.0f;   /* 消抖，避免电机在 0 附近抖 */
    }
}

/* ---------------------------------------------------------------------------
 * 第 2 步：高速时限制自转（防离心打滑）
 * -------------------------------------------------------------------------*/
static void chassis_limit_yaw(void)
{
    float lin = sqrtf(g_cur[0] * g_cur[0] + g_cur[1] * g_cur[1]);
    float lim = YAW_LIMIT_BASE - YAW_LIMIT_K * lin;
    if (lim < 0.10f) lim = 0.10f;      /* 永远留一点转向能力，别锁死 */
    if (g_cur[2] >  lim) g_cur[2] =  lim;
    if (g_cur[2] < -lim) g_cur[2] = -lim;
}

/* ---------------------------------------------------------------------------
 * 第 3 步：麦轮逆解
 * -------------------------------------------------------------------------*/
void Mecanum_Fwd(float w[4], float *vx, float *vy, float *wz)
{
    float r = WHEEL_RADIUS;
    if (vx) *vx = ( w[0] + w[1] + w[2] + w[3]) * r / 4.0f;
    if (vy) *vy = (-w[0] + w[1] + w[2] - w[3]) * r / 4.0f;
    if (wz) *wz = (-w[0] + w[1] - w[2] + w[3]) * r /
                  (4.0f * (ROBOT_HALF_WHEELBASE + ROBOT_HALF_TRACK));
}

static void chassis_kinematics(void)
{
    float vx = g_cur[0], vy = g_cur[1], wz = g_cur[2] * YAW_GEOM;

    g_wheel[MOTOR_LF] = vx - vy - wz;
    g_wheel[MOTOR_RF] = vx + vy + wz;
    g_wheel[MOTOR_LB] = vx + vy - wz;
    g_wheel[MOTOR_RB] = vx - vy + wz;
}

/* ---------------------------------------------------------------------------
 * 第 4 步：矢量归一化限幅（保方向）—— 新手最容易漏的一步
 * -------------------------------------------------------------------------*/
static void chassis_normalize(void)
{
    float mx = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        float a = fabsf(g_wheel[i]);
        if (a > mx) mx = a;
    }
    if (mx > 1.0f)
    {
        float k = 1.0f / mx;
        for (uint8_t i = 0; i < 4; i++) g_wheel[i] *= k;
    }
}

void Chassis_Update(float dt)
{
    if (g_emergency)
    {
        Motor_BrakeAll();
        return;
    }

    chassis_ramp(dt);
    chassis_limit_yaw();
    chassis_kinematics();
    chassis_normalize();
    Motor_OutputAll(g_wheel);
}

void Chassis_Stop(void)
{
    g_tgt[0] = g_tgt[1] = g_tgt[2] = 0;
}

void Chassis_EmergencyStop(void)
{
    g_emergency = 1;
    g_tgt[0] = g_tgt[1] = g_tgt[2] = 0;
    g_cur[0] = g_cur[1] = g_cur[2] = 0;
    g_wheel[0] = g_wheel[1] = g_wheel[2] = g_wheel[3] = 0;
    Motor_BrakeAll();
}

void Chassis_GetCur(float *vx, float *vy, float *wz)
{
    if (vx) *vx = g_cur[0];
    if (vy) *vy = g_cur[1];
    if (wz) *wz = g_cur[2];
}

void Chassis_GetWheel(float w[4])
{
    for (uint8_t i = 0; i < 4; i++) w[i] = g_wheel[i];
}
