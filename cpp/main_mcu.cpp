/* ============================================================================
 * main_mcu.cpp —— C++ 版固件入口（运动 + 机械臂 + 视觉 三合一）
 *
 * 【这个文件是干嘛的】
 *   烧进 STM32 之后，接上串口（PA9/PA10，115200），用串口助手发字符
 *   就能分别验证底盘、机构、视觉三块，不用烧四个固件来回切。
 *
 * 【上电前必读的三条安全前提】
 *   1. 第一次上电务必把车**架空**（四个轮离地），否则参数没标定时车会乱跑。
 *   2. 电池电压 ≥7.0V；DRV8833 的 nSLEEP 必须接高，否则电机根本不动。
 *   3. 舵机听到"吱吱"声立刻断电 —— 那是超行程在硬撑，会烧舵机。
 *
 * 【串口命令表（发一个字符即可，不用回车）】
 *   运动： w 前进  s 后退  a 左移  d 右移  q 左转  e 右转  x 停  z 斜走45°
 *          r 自动演示（前进 1.5 秒然后停）
 *   机构： 1 收起  2 待命  3 地面取块  4 台阶取块  5 入仓  6 挑物资架  7 洞窟
 *          8 放铲
 *   微调： [ ] 夹爪 ∓5°    - = 大臂 ∓5°    , . 前铲 ∓5°      m 打印当前角度
 *   视觉： v 造一帧测试数据（不用接摄像头也能验证解析）   V 打印视觉状态
 *   其他： h 帮助    0 急停解除
 * ==========================================================================*/

#include "crtc.hpp"

extern "C" {
#include "main.h"
#include "board.h"
#include "bsp.h"
}

using namespace crtc;

/* ---- 全局对象。都是 POD，静态存储期零初始化，靠 init() 显式初始化 ------ */
static Chassis      g_chassis;
static Servo        g_servo[kServoCount];
static ArmSequencer g_arm;
static VisionLink   g_vis;

static float    g_vx = 0.0f, g_vy = 0.0f, g_wz = 0.0f;
static uint32_t g_tCtrl = 0, g_tServo = 0, g_tLog = 0, g_tBat = 0;
static uint32_t g_demoEnd = 0;          /* 非阻塞演示的结束时间 */
static uint16_t g_demoSpeed = 0;

/* --------------------------------------------------------------------------- */
static void startPwm(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

static void printHelp(void)
{
    hal::print("\r\n==== CRTC2026 C++ firmware (motion / arm / vision) ====\r\n");
    hal::print("move : w s a d q e  x=stop  z=45deg  r=demo\r\n");
    hal::print("arm  : 1 stow 2 ready 3 pick-ground 4 pick-step 5 store 6 rack 7 cave 8 scoop\r\n");
    hal::print("trim : [ ] grip   - = arm   , . scoop   m print angles\r\n");
    hal::print("vis  : v fake frame   V status\r\n");
    hal::print("misc : h help   0 clear emergency\r\n\r\n");
}

static void printStatus(void)
{
    const float *w = g_chassis.wheels();
    hal::print("w[%+.2f %+.2f %+.2f %+.2f] d[%.2f %.2f %.2f %.2f] "
               "sv[%3.0f %3.0f %3.0f %3.0f] arm=%s vbat=%.2f\r\n",
               w[0], w[1], w[2], w[3],
               g_chassis.mixer().duty(0), g_chassis.mixer().duty(1),
               g_chassis.mixer().duty(2), g_chassis.mixer().duty(3),
               g_servo[0].current(), g_servo[1].current(),
               g_servo[2].current(), g_servo[3].current(),
               g_arm.busy() ? "busy" : "idle",
               hal::vbat());
}

static void printAngles(void)
{
    hal::print("grip=%.0f arm=%.0f scoop=%.0f extra=%.0f\r\n",
               g_servo[kGrip].current(), g_servo[kArm].current(),
               g_servo[kScoop].current(), g_servo[kExtra].current());
}

/* 造一帧视觉数据：cls=2(黄块), x=+40, y=-10, w=60, h=60, area=3600 */
static void fakeVisionFrame(void)
{
    uint8_t f[14];
    int16_t x = 40, y = -10;
    uint16_t w = 60, h = 60, area = 3600;

    f[0] = 0xAA; f[1] = 0x55; f[2] = 2;
    f[3] = (uint8_t)(x & 0xFF);        f[4]  = (uint8_t)((x >> 8) & 0xFF);
    f[5] = (uint8_t)(y & 0xFF);        f[6]  = (uint8_t)((y >> 8) & 0xFF);
    f[7] = (uint8_t)(w & 0xFF);        f[8]  = (uint8_t)((w >> 8) & 0xFF);
    f[9] = (uint8_t)(h & 0xFF);        f[10] = (uint8_t)((h >> 8) & 0xFF);
    f[11] = (uint8_t)(area & 0xFF);    f[12] = (uint8_t)((area >> 8) & 0xFF);

    uint8_t chk = 0;
    for (int i = 2; i < 13; i++) chk ^= f[i];
    f[13] = chk;

    uint32_t now = hal::millis();
    for (int i = 0; i < 14; i++) g_vis.feedByte(f[i], now);

    hal::print("frame fed: cls=%d x=%d y=%d area=%u valid=%d\r\n",
               g_vis.frame().cls, g_vis.frame().x, g_vis.frame().y,
               g_vis.frame().area, g_vis.valid(now, 300) ? 1 : 0);
}

static void handleCmd(char c)
{
    switch (c)
    {
    /* --- 底盘 --- */
    case 'w': g_vx =  1.0f; g_vy = 0; g_wz = 0; break;
    case 's': g_vx = -1.0f; g_vy = 0; g_wz = 0; break;
    case 'a': g_vx = 0; g_vy =  1.0f; g_wz = 0; break;
    case 'd': g_vx = 0; g_vy = -1.0f; g_wz = 0; break;
    case 'q': g_vx = 0; g_vy = 0; g_wz =  1.0f; break;
    case 'e': g_vx = 0; g_vy = 0; g_wz = -1.0f; break;
    case 'x': g_vx = g_vy = g_wz = 0.0f; g_demoEnd = 0; break;
    case 'z': g_vx = 0.707f; g_vy = 0.707f; g_wz = 0; break;
    case 'r': g_demoSpeed = 1; g_demoEnd = hal::millis() + 1500; break;

    /* --- 机构 --- */
    case '1': g_arm.start(POSE_STOW,        hal::millis()); break;
    case '2': g_arm.start(POSE_READY,       hal::millis()); break;
    case '3': g_arm.start(POSE_PICK_GROUND, hal::millis()); break;
    case '4': g_arm.start(POSE_PICK_STEP,   hal::millis()); break;
    case '5': g_arm.start(POSE_STORE,       hal::millis()); break;
    case '6': g_arm.start(POSE_RACK,        hal::millis()); break;
    case '7': g_arm.start(POSE_CAVE,        hal::millis()); break;
    case '8': g_arm.start(POSE_SCOOP,       hal::millis()); break;

    /* --- 舵机在线微调（调好之后把角度抄回 cpp/src/arm.cpp） --- */
    case '[': g_servo[kGrip].setTarget(g_servo[kGrip].target() - 5.0f); break;
    case ']': g_servo[kGrip].setTarget(g_servo[kGrip].target() + 5.0f); break;
    case '-': g_servo[kArm].setTarget(g_servo[kArm].target() - 5.0f); break;
    case '=': g_servo[kArm].setTarget(g_servo[kArm].target() + 5.0f); break;
    case ',': g_servo[kScoop].setTarget(g_servo[kScoop].target() - 5.0f); break;
    case '.': g_servo[kScoop].setTarget(g_servo[kScoop].target() + 5.0f); break;
    case 'm': printAngles(); break;

    /* --- 视觉 --- */
    case 'v': fakeVisionFrame(); break;
    case 'V':
        hal::print("vis cls=%d x=%d y=%d w=%u h=%u area=%u\r\n",
                   g_vis.frame().cls, g_vis.frame().x, g_vis.frame().y,
                   g_vis.frame().w, g_vis.frame().h, g_vis.frame().area);
        break;

    case 'h': printHelp(); break;
    case '0': g_chassis.stop(); break;
    default: break;
    }
}

/* --------------------------------------------------------------------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    BSP_InitAll();
    startPwm();

    g_chassis.init(kDefaultParams);
    g_servo[kGrip].init(kDefaultParams,  kGrip,  30.0f);
    g_servo[kArm].init(kDefaultParams,   kArm,   25.0f);
    g_servo[kScoop].init(kDefaultParams, kScoop, 60.0f);
    g_servo[kExtra].init(kDefaultParams, kExtra, 0.0f);
    g_arm.init(g_servo, kDefaultParams);
    g_vis.init();

    g_arm.start(POSE_STOW, hal::millis());   /* 上电就收到检录姿态 */

    printHelp();

    for (;;)
    {
        uint32_t now = hal::millis();

        /* 串口命令 */
        int ch = hal::uartRead();
        if (ch >= 0) handleCmd((char)ch);

        /* 演示：非阻塞，到点自动停 */
        if (g_demoEnd != 0)
        {
            if ((int32_t)(now - g_demoEnd) >= 0)
            {
                g_demoEnd = 0;
                g_vx = g_vy = g_wz = 0.0f;
            }
            else
            {
                g_vx = 0.5f; g_vy = 0.0f; g_wz = 0.0f;
            }
        }

        /* 底盘控制 5ms */
        if ((uint32_t)(now - g_tCtrl) >= 5)
        {
            float dt = (float)(now - g_tCtrl) * 0.001f;
            g_tCtrl = now;
            g_chassis.setTarget(g_vx, g_vy, g_wz);
            g_chassis.update(dt, now);
        }

        /* 舵机插值 10ms */
        if ((uint32_t)(now - g_tServo) >= 10)
        {
            float dt = (float)(now - g_tServo) * 0.001f;
            g_tServo = now;
            for (int i = 0; i < kServoCount; i++) g_servo[i].update(dt);
            g_arm.update(now);
            g_vis.update(now);
        }

        /* 电压补偿 500ms */
        if ((uint32_t)(now - g_tBat) >= 500)
        {
            g_tBat = now;
            g_chassis.setVbat(hal::vbat());
        }

        /* 状态打印 400ms */
        if ((uint32_t)(now - g_tLog) >= 400)
        {
            g_tLog = now;
            printStatus();
        }
    }
}
