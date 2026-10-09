/* ============================================================================
 * chassis_diff.c —— B 车（橡胶轮）的差速转向运动学
 *
 * 【为什么 B 车不能直接用 mecanum.c】
 *   麦轮逆解里有 vy 项：
 *       w_LF = vx - vy - (a+b)ω
 *   橡胶轮**没有辊子**，横向完全没有自由度，vy 这一项在物理上不存在。
 *   照抄麦轮公式的后果：手柄一推横移，四个轮子会两两反向对打，
 *   车原地抽搐、电流翻倍、什么也不发生。这不是"效果差一点"，是错的。
 *
 *   正确形式（滑移转向 / skid-steer，和坦克、履带车一样）：
 *       w_L = vx + K·ω        （K = 半轮距 / 轮半径）
 *       w_R = vx - K·ω
 *   左边两个轮和右边两个轮各自同速，靠左右速度差转弯。
 *
 * 【和 A 车的代码关系：共用外圈，只换内核】
 *   斜坡限幅、死区、归一化、急停这些**与轮型无关**的安全逻辑，
 *   两个文件里是逐字一致的（见 diff_ramp / diff_normalize）。
 *   唯一不同的是 chassis_kinematics 那一个函数。
 *   这样两台车共用一个 Chassis_Update 的调用时序，调参经验可以互相借鉴。
 *
 * 【代价，写在最前面免得被忘掉】
 *   ① 不能横移。要靠"前进+转向"凑出侧向位移，窄缝对位会明显变难。
 *   ② 转弯时轮子必然打滑（滑移转向的固有损失），
 *      摩擦系数越高、车越重，打滑越厉害，电耗比麦轮高。
 *   ③ K 里含 (半轮距 / 轮半径)，**换轮径必须重算**。
 * ==========================================================================*/

#include "chassis_diff.h"
#include "motor.h"
#include "board.h"
#include "config.h"
#include <math.h>

static float g_tgt[3] = {0, 0, 0};
static float g_cur[3] = {0, 0, 0};
static float g_wheel[4] = {0, 0, 0, 0};
static uint8_t g_emergency = 0;

/* K = 半轮距 / 轮半径：把"自转指令"换算成"左右轮速差" */
static const float TURN_K = ROBOT_HALF_TRACK / WHEEL_RADIUS;

void Diff_Init(void)
{
    g_tgt[0] = g_tgt[1] = g_tgt[2] = 0;
    g_cur[0] = g_cur[1] = g_cur[2] = 0;
    for (uint8_t i = 0; i < 4; i++) g_wheel[i] = 0;
    g_emergency = 0;
    Motor_Init();
}

/* vy 参数保留在签名里，是为了和 Chassis_SetTarget 完全同形，
 * 这样 app.c 里可以按车号无脑调用同一个接口。橡胶轮直接忽略 vy。 */
void Diff_SetTarget(float vx, float vy, float wz)
{
    (void)vy;   /* 橡胶轮没有横向自由度，丢掉 */

    g_tgt[0] = vx * (float)FORWARD_SIGN * SPEED_GAIN_FORWARD;
    g_tgt[2] = wz * (float)YAW_SIGN     * SPEED_GAIN_YAW_DIFF;

    if (g_tgt[0] >  1.0f) g_tgt[0] =  1.0f;
    if (g_tgt[0] < -1.0f) g_tgt[0] = -1.0f;
    if (g_tgt[2] >  1.0f) g_tgt[2] =  1.0f;
    if (g_tgt[2] < -1.0f) g_tgt[2] = -1.0f;
}

/* 第 1 步：斜坡限幅 —— 和麦轮版逐字一致（防滑原理与轮型无关） */
static void diff_ramp(float dt)
{
    const float acc_up[2] = {CHASSIS_ACC_UP, CHASSIS_YAW_ACC_UP};
    const float acc_dn[2] = {CHASSIS_ACC_DOWN, CHASSIS_YAW_ACC_DOWN};
    const uint8_t idx[2] = {0, 2};

    for (uint8_t k = 0; k < 2; k++)
    {
        uint8_t i = idx[k];
        float d = g_tgt[i] - g_cur[i];
        float lim = (fabsf(g_tgt[i]) > fabsf(g_cur[i]) ? acc_up[k] : acc_dn[k]) * dt;
        if (lim < 0.0001f) lim = 0.0001f;

        if      (d >  lim) d =  lim;
        else if (d < -lim) d = -lim;

        g_cur[i] += d;
        if (fabsf(g_cur[i]) < 0.001f) g_cur[i] = 0.0f;
    }
}

/* 第 2 步：差速运动学 —— 唯一与麦轮不同的地方 */
static void diff_kinematics(void)
{
    float vx = g_cur[0];
    float turn = g_cur[2] * TURN_K;

    /* 左两个轮同速，右两个轮同速 */
    g_wheel[MOTOR_LF] = vx + turn;
    g_wheel[MOTOR_LB] = vx + turn;
    g_wheel[MOTOR_RF] = vx - turn;
    g_wheel[MOTOR_RB] = vx - turn;
}

/* 第 3 步：矢量归一化限幅 —— 和麦轮版逐字一致（保方向） */
static void diff_normalize(void)
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

void Diff_Update(float dt)
{
    if (g_emergency)
    {
        Motor_BrakeAll();
        return;
    }

    diff_ramp(dt);
    diff_kinematics();
    diff_normalize();
    Motor_OutputAll(g_wheel);
}

void Diff_Stop(void)
{
    g_tgt[0] = g_tgt[1] = g_tgt[2] = 0;
}

void Diff_EmergencyStop(void)
{
    g_emergency = 1;
    g_tgt[0] = g_tgt[1] = g_tgt[2] = 0;
    g_cur[0] = g_cur[1] = g_cur[2] = 0;
    for (uint8_t i = 0; i < 4; i++) g_wheel[i] = 0;
    Motor_BrakeAll();
}

void Diff_GetWheel(float w[4])
{
    for (uint8_t i = 0; i < 4; i++) w[i] = g_wheel[i];
}
