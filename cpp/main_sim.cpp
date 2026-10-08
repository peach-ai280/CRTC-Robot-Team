/* ============================================================================
 * main_sim.cpp —— 把 C++ 版的三块逻辑放进 ARM 模拟器里真跑一遍
 *
 * 和 C 版 tools/sim/run_sim.py 的区别：这里连**机构序列**和**视觉解析**一起测，
 * 不只是运动学。跑完把所有结果 printf 进 g_log，Python 读出来打印。
 *
 * 测六件事：
 *   1. 三个特例的方向对不对（纯前进 / 纯横移 / 纯自转）
 *   2. 有没有四轮解算值超过 1（归一化失效）
 *   3. 归一化有没有保住速度矢量方向（正解反算 vs 指令）
 *   4. 加速度斜坡：0 -> 95% 目标要多久（防滑有没有生效）
 *   5. 机构序列：一个完整"地面取块"要跑多久，舵机有没有串行走完
 *   6. 视觉帧解析：好帧 / 坏校验 / 垃圾数据后能不能自恢复
 * ==========================================================================*/

#include "crtc.hpp"
#include <math.h>
#include <stdio.h>

using namespace crtc;

extern "C" {
    extern uint32_t g_done;
}

namespace crtc { namespace hal { void simAdvance(uint32_t ms); } }

static Chassis      g_chassis;
static Servo        g_servo[kServoCount];
static ArmSequencer g_arm;
static VisionLink   g_vis;

static int g_fail = 0;

static void check(bool ok, const char *what)
{
    printf("  [%s] %s\r\n", ok ? "PASS" : "FAIL", what);
    if (!ok) g_fail++;
}

/* --------------------------------------------------------------------------- */
static void caseWhere(const char *name, float vx, float vy, float wz,
                      float w[4], float &aCmd, float &aExp, float &aOut)
{
    g_chassis.stop();
    g_chassis.setTarget(vx, vy, wz);

    /* 跑 1 秒（200 步 × 5ms）让斜坡收敛 */
    for (int i = 0; i < 200; i++)
    {
        hal::simAdvance(5);
        g_chassis.setTarget(vx, vy, wz);
        g_chassis.update(0.005f, hal::millis());
    }

    const float *ww = g_chassis.wheels();
    for (int i = 0; i < 4; i++) w[i] = ww[i];

    /* 正解反算 */
    const Params *p = &kDefaultParams;
    float fvx = ( w[0] + w[1] + w[2] + w[3]) * p->wheelRadius / 4.0f;
    float fvy = (-w[0] + w[1] + w[2] - w[3]) * p->wheelRadius / 4.0f;

    aCmd = atan2f(vy, vx) * 180.0f / 3.14159265f;
    aExp = atan2f(vy * p->gainStrafe, vx * p->gainForward) * 180.0f / 3.14159265f;
    aOut = atan2f(fvy, fvx) * 180.0f / 3.14159265f;
    (void)name;
}

static void testKinematics(void)
{
    printf("\r\n[1] 麦轮解算与归一化\r\n");

    struct Case { const char *n; float vx, vy, wz; };
    static const Case cases[] = {
        { "forward ",  1.0f,  0.0f,  0.0f },
        { "strafe  ",  0.0f,  1.0f,  0.0f },
        { "yaw     ",  0.0f,  0.0f,  1.0f },
        { "fwd+yaw ",  1.0f,  0.0f,  0.5f },
        { "diag45  ",  0.707f, 0.707f, 0.0f },
        { "all3    ",  0.6f,  0.6f,  0.6f },
        { "fwd-yaw ",  1.0f,  0.0f, -0.5f },
        { "back    ", -1.0f,  0.0f,  0.0f },
    };
    const int n = (int)(sizeof(cases) / sizeof(Case));

    float store[8][4];
    float aCmd[8], aExp[8], aOut[8];

    printf("  case      cmd(vx,vy,wz)         wheels                          |w|max\r\n");
    for (int i = 0; i < n; i++)
    {
        float w[4];
        caseWhere(cases[i].n, cases[i].vx, cases[i].vy, cases[i].wz, w,
                  aCmd[i], aExp[i], aOut[i]);
        for (int k = 0; k < 4; k++) store[i][k] = w[k];

        float mx = 0.0f;
        for (int k = 0; k < 4; k++) if (fabsf(w[k]) > mx) mx = fabsf(w[k]);

        printf("  %s (%+.2f,%+.2f,%+.2f)  [%+.3f %+.3f %+.3f %+.3f]  %.3f\r\n",
               cases[i].n, cases[i].vx, cases[i].vy, cases[i].wz,
               w[0], w[1], w[2], w[3], mx);

        if (mx > 1.001f)
        {
            printf("  [FAIL] %s 四轮解算值超过 1（归一化没生效）\r\n", cases[i].n);
            g_fail++;
        }
    }

    check(store[0][0] > 0.4f && store[0][1] > 0.4f && store[0][2] > 0.4f &&
          store[0][3] > 0.4f && (fabsf(store[0][0] - store[0][3]) < 0.02f),
          "纯前进：四轮同为正且相等");
    check(store[1][0] < -0.4f && store[1][3] < -0.4f &&
          store[1][1] >  0.4f && store[1][2] >  0.4f,
          "纯左移：LF/RB 负，RF/LB 正");
    check(store[2][0] < -0.3f && store[2][2] < -0.3f &&
          store[2][1] >  0.3f && store[2][3] >  0.3f,
          "纯逆时针：左两轮负，右两轮正");
    check(store[7][0] < -0.4f && store[7][1] < -0.4f &&
          store[7][2] < -0.4f && store[7][3] < -0.4f,
          "纯后退：四轮同为负");

    printf("\r\n[2] 归一化有没有保住方向（正解反算 vs 增益后期望）\r\n");
    for (int i = 0; i < n; i++)
    {
        if (fabsf(cases[i].vx) < 1e-6f && fabsf(cases[i].vy) < 1e-6f) continue;
        printf("  %s cmd=%7.2f  expect=%7.2f  back=%7.2f  err=%+.3f\r\n",
               cases[i].n, aCmd[i], aExp[i], aOut[i], aOut[i] - aExp[i]);
        if (fabsf(aOut[i] - aExp[i]) > 0.5f)
        {
            printf("  [FAIL] %s 方向偏了 %.2f 度\r\n", cases[i].n, aOut[i] - aExp[i]);
            g_fail++;
        }
    }
    /* 增益相等时，指令方向应该就是实际方向 */
    check(fabsf(aOut[4] - aCmd[4]) < 1.0f, "斜走45° 实际方向 == 指令方向（两增益必须相等）");
}

static void testRamp(void)
{
    printf("\r\n[3] 加速度斜坡（防滑第一道防线）\r\n");

    /* 先把上一个用例残留的速度放掉，否则测出来的是"从倒车到前进"的时间，
       会比真实起步时间大一倍，看着像斜坡参数不对。 */
    g_chassis.stop();
    for (int i = 0; i < 400; i++)
    {
        hal::simAdvance(5);
        g_chassis.update(0.005f, hal::millis());
    }

    uint32_t t0 = hal::millis();
    uint32_t t95 = 0;
    float target = 1.0f * kDefaultParams.gainForward;

    for (int i = 0; i < 400; i++)
    {
        hal::simAdvance(5);
        g_chassis.setTarget(1.0f, 0.0f, 0.0f);
        g_chassis.update(0.005f, hal::millis());
        if (t95 == 0 && g_chassis.cur()[0] >= target * 0.95f)
            t95 = hal::millis() - t0;
    }

    printf("  0 -> 95%% 用时 %u ms（ACC_UP=%.1f，理论约 %.0f ms）\r\n",
           (unsigned)t95, kDefaultParams.accUp, target / kDefaultParams.accUp * 1000.0f);
    check(t95 > 150 && t95 < 600, "起步有斜坡，不是瞬间到满速");

    /* 松手后应该快速降下来 */
    uint32_t t1 = hal::millis();
    uint32_t tStop = 0;
    for (int i = 0; i < 400; i++)
    {
        hal::simAdvance(5);
        g_chassis.setTarget(0.0f, 0.0f, 0.0f);
        g_chassis.update(0.005f, hal::millis());
        if (tStop == 0 && fabsf(g_chassis.cur()[0]) < 0.01f)
            tStop = hal::millis() - t1;
    }
    printf("  松手 -> 停住用时 %u ms（ACC_DOWN=%.1f）\r\n",
           (unsigned)tStop, kDefaultParams.accDown);
    check(tStop > 20 && tStop < 400, "松手后能及时停住（不是滑行很远）");
}

static void testDeadzone(void)
{
    printf("\r\n[4] 死区补偿（小油门到底动不动）\r\n");
    MotorMixer mx;
    mx.init(kDefaultParams);
    mx.setVbat(7.4f);

    mx.set(0, 0.05f, hal::millis());
    float d5 = mx.duty(0);
    mx.set(1, 0.50f, hal::millis());
    float d50 = mx.duty(1);
    mx.set(2, 1.00f, hal::millis());
    float d100 = mx.duty(2);

    printf("  speed 0.05 -> duty %.3f\r\n", d5);
    printf("  speed 0.50 -> duty %.3f\r\n", d50);
    printf("  speed 1.00 -> duty %.3f\r\n", d100);

    check(d5 >= kDefaultParams.deadzone - 0.001f,
          "小油门输出也被抬到死区之上（不会推小油门不动）");
    check(d100 > 0.95f && d100 <= 1.001f, "满油门占空比接近 1");
    check(d5 < d50 && d50 < d100, "占空比随速度单调增");
}

static void testArm(void)
{
    printf("\r\n[5] 机构序列（一次只动一个舵机，防 5V 塌陷）\r\n");

    g_servo[kGrip].init(kDefaultParams,  kGrip,  30.0f);
    g_servo[kArm].init(kDefaultParams,   kArm,   25.0f);
    g_servo[kScoop].init(kDefaultParams, kScoop, 60.0f);
    g_servo[kExtra].init(kDefaultParams, kExtra, 0.0f);
    g_arm.init(g_servo, kDefaultParams);

    uint32_t t0 = hal::millis();
    g_arm.start(POSE_PICK_GROUND, t0);

    uint32_t steps = 0;
    int maxMoving = 0;
    while (g_arm.busy() && steps < 4000)      /* 最多 40 秒 */
    {
        hal::simAdvance(10);
        uint32_t now = hal::millis();
        for (int i = 0; i < kServoCount; i++) g_servo[i].update(0.010f);
        g_arm.update(now);

        int moving = 0;
        for (int i = 0; i < kServoCount; i++) if (g_servo[i].isMoving()) moving++;
        if (moving > maxMoving) maxMoving = moving;
        steps++;
    }

    uint32_t used = hal::millis() - t0;
    printf("  地面取块序列耗时 %u ms，同一时刻最多 %d 个舵机在动\r\n",
           (unsigned)used, maxMoving);
    printf("  末态 grip=%.0f arm=%.0f scoop=%.0f\r\n",
           g_servo[kGrip].current(), g_servo[kArm].current(),
           g_servo[kScoop].current());

    check(!g_arm.busy(), "序列能跑完（没有卡死）");
    check(used > 800 && used < 8000, "耗时在合理区间（0.8~8 秒）");
    check(maxMoving <= 1, "同一时刻只有一个舵机在动（不会拉塌 5V）");
    check(fabsf(g_servo[kArm].current() - 25.0f) < 1.0f, "结束时大臂回到收起位 25°");
}

static void testVision(void)
{
    printf("\r\n[6] 视觉帧解析（14 字节定长帧 + 异或校验）\r\n");
    g_vis.init();

    /* 好帧 */
    uint8_t f[14];
    int16_t x = -35, y = 12;
    uint16_t w = 70, h = 66, area = 4620;
    f[0] = 0xAA; f[1] = 0x55; f[2] = 1;
    f[3] = (uint8_t)(x & 0xFF);     f[4] = (uint8_t)((x >> 8) & 0xFF);
    f[5] = (uint8_t)(y & 0xFF);     f[6] = (uint8_t)((y >> 8) & 0xFF);
    f[7] = (uint8_t)(w & 0xFF);     f[8] = (uint8_t)((w >> 8) & 0xFF);
    f[9] = (uint8_t)(h & 0xFF);     f[10] = (uint8_t)((h >> 8) & 0xFF);
    f[11] = (uint8_t)(area & 0xFF); f[12] = (uint8_t)((area >> 8) & 0xFF);
    uint8_t chk = 0;
    for (int i = 2; i < 13; i++) chk ^= f[i];
    f[13] = chk;

    uint32_t now = hal::millis();
    for (int i = 0; i < 14; i++) g_vis.feedByte(f[i], now);

    printf("  好帧  -> cls=%d x=%d y=%d w=%u h=%u area=%u\r\n",
           g_vis.frame().cls, g_vis.frame().x, g_vis.frame().y,
           g_vis.frame().w, g_vis.frame().h, g_vis.frame().area);
    check(g_vis.frame().cls == 1 && g_vis.frame().x == -35 &&
          g_vis.frame().y == 12 && g_vis.frame().area == 4620,
          "好帧解析正确（含负数坐标）");

    /* 坏校验：把校验字节改错，cls 不应该被更新 */
    uint8_t bad[14];
    for (int i = 0; i < 14; i++) bad[i] = f[i];
    bad[13] = chk ^ 0xFF;
    bad[2] = 2;
    for (int i = 0; i < 14; i++) g_vis.feedByte(bad[i], now);
    check(g_vis.frame().cls == 1, "校验错的帧被丢弃（cls 没被改）");

    /* 垃圾数据 -> 应能重新同步 */
    for (int i = 0; i < 7; i++) g_vis.feedByte((uint8_t)(0x10 + i), now);
    for (int i = 0; i < 14; i++) g_vis.feedByte(f[i], now);
    check(g_vis.frame().area == 4620, "垃圾数据后能重新对齐");

    /* 转向指令 */
    float turn = g_vis.turnCommand(12, 0.0022f, 0.40f);
    printf("  x=-35 时转向指令 %.3f（应为负，即往左转）\r\n", turn);
    check(turn < 0.0f, "目标在左边就往左转");

    /* 超时失效 */
    hal::simAdvance(1000);
    check(!g_vis.valid(hal::millis(), 300), "超过 300ms 没新帧则判定失效");
}

/* --------------------------------------------------------------------------- */
int main(void)
{
    g_chassis.init(kDefaultParams);

    printf("==== CRTC2026 C++ core offline check ====\r\n");

    testKinematics();
    testRamp();
    testDeadzone();
    testArm();
    testVision();

    printf("\r\n==== RESULT: %s (fail=%d) ====\r\n",
           g_fail == 0 ? "ALL PASS" : "FAIL", g_fail);

    g_done = 1;
    return g_fail;
}
