/* ============================================================================
 * app.c —— 应用层：手柄输入 -> 模式判定 -> 底盘/机构指令 -> 状态指示
 *
 * 【按键总表（PS2）】
 *   左摇杆   前后 + 左右平移（麦轮全向）
 *   右摇杆 X 自转
 *   ○        宏1 地面方块自动收集
 *   △        宏2 挑物资架黄块
 *   □        宏3 洞窟探杆取块
 *   ×        宏4 一键贴边横移
 *   L2       宏5 视觉自动对位（需 USE_VISION=1）
 *   L1       全部机构收起（检录姿态）
 *   R1       前铲 放下/抬起 切换
 *   SELECT   模式切换：手动 → 半自动 → 全自动 → 手动（循环）
 *   START    急停（再按一次解除）
 *   全自动档：进档即自动开跑；动一下摇杆立刻接管并退回半自动；
 *             跑到 DONE 后按 ○ 可以再来一轮
 *   ↑↓       大臂：点按 ±2°（标定，串口打印角度）／按住连续转 90°/s（手动升降）
 *   ←→       夹爪：点按 ±2°／按住连续转（手动开合）
 *   L2 / R2  前铲：点按 ±2°／按住连续转（未开视觉时；开了视觉 L2 是视觉宏）
 * ==========================================================================*/

#include "app.h"
#include "board.h"
#include "config.h"
#include "ps2.h"
#include "motor.h"
#include "mecanum.h"
#include "servo.h"
#include "arm.h"
#include "sensor.h"
#include "vision.h"
#include "auto_task.h"
#include "auto_full.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

static uint8_t  g_mode = DEFAULT_MODE;      /* MODE_MANUAL / MODE_SEMI / MODE_FULL */
static uint8_t  g_estop = 0;
static uint32_t g_t_ctrl = 0, g_t_servo = 0, g_t_log = 0, g_t_led = 0;
static float    g_tune_arm  = SERVO_ARM_STOW;
static float    g_tune_grip = SERVO_GRIP_OPEN;
static float    g_tune_scoop= SERVO_SCOOP_TRAVEL;

/* ---------------------------------------------------------------------------
 * 串口调试输出（不用 printf 重定向，直接发，省得碰 syscalls.c）
 * -------------------------------------------------------------------------*/
static void LOG(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0)
        HAL_UART_Transmit(&DBG_UART, (uint8_t *)buf,
                          (n > (int)sizeof(buf)) ? sizeof(buf) : (uint16_t)n, 50);
}

/* ---------------------------------------------------------------------------
 * 摇杆 -> 归一化速度，带死区 + expo 曲线
 * expo > 1：中间段更细腻，便于慢速对位；满舵不变
 * -------------------------------------------------------------------------*/
static float stick_map(int16_t raw, float expo)
{
    float dz = (float)PS2_STICK_DEADZONE / 128.0f;
    float x  = (float)raw / 128.0f;
    if (x >  1.0f) x =  1.0f;
    if (x < -1.0f) x = -1.0f;

    float a = fabsf(x);
    if (a < dz) return 0.0f;

    /* 去掉死区后重新归一化到 0..1，否则会有跳变 */
    a = (a - dz) / (1.0f - dz);
    a = powf(a, expo);
    return (x < 0) ? -a : a;
}

/* ---------------------------------------------------------------------------
 * 手动模式：摇杆 -> 车体速度
 * 符号说明（如果方向反了，改这里最直观，不要去改线）
 *   PS2 左摇杆向上 -> LY 减小（约 -128），我们要 vx 为正（前进） => 取负
 *   PS2 左摇杆向右 -> LX 增大，我们定义 vy 左为正         => 取负
 *   PS2 右摇杆向右 -> RX 增大，希望车顺时针转（ω 为负）   => 取负
 * -------------------------------------------------------------------------*/
static void manual_cmd(float *vx, float *vy, float *wz)
{
    *vx = -stick_map(ps2.ly, PS2_STICK_EXPO);
    *vy = -stick_map(ps2.lx, PS2_STICK_EXPO);
    *wz = -stick_map(ps2.rx, PS2_STICK_EXPO);
}

/* ---------------------------------------------------------------------------
 * 舵机在线标定 / 手动操作（方向键）
 *
 *   点按一下      ：±2°，串口打印角度 —— 用来标定，把数抄进 config.h
 *   按住 > 350ms  ：连续转动 90°/s —— 用来手动操作大臂升降、夹爪开合
 *
 * 为什么要加"按住连续转"：原来只有点按 ±2°，那是给标定用的，
 * 想手动把大臂从收起位(25°)抬到取块位(120°)要按 48 次，根本没法操作。
 * -------------------------------------------------------------------------*/
#define TUNE_STEP_DEG    2.0f      /* 点按步长（度） */
#define TUNE_HOLD_MS     350       /* 按住超过这么久转为连续转动 */
#define TUNE_HOLD_DPS    90.0f     /* 连续转动速度（度/秒） */

typedef struct {
    uint32_t t_down;    /* 按下时刻，0 = 当前没按住 */
    uint32_t t_prev;    /* 上一帧时刻（算 dt 用） */
    uint32_t t_log;     /* 上次打印时刻（防止刷屏） */
    uint16_t key;       /* 当前按住的键，换键要重新计时 */
} TuneHold_t;

static TuneHold_t g_hold[SERVO_NUM];

static void tune_servo(ServoId_t id, uint16_t dec_k, uint16_t inc_k,
                       float *cur, uint32_t now)
{
    TuneHold_t *h = &g_hold[id];
    int dir = 0, just_pressed = 0;
    uint16_t k = 0;

    if      (PS2_Key(inc_k)) { dir = +1; k = inc_k; }
    else if (PS2_Key(dec_k)) { dir = -1; k = dec_k; }

    if (dir == 0 || k != h->key)          /* 松开了，或换了方向键 → 重新计时 */
    {
        h->t_down = (dir == 0) ? 0 : now;
        h->t_prev = now;
        h->t_log  = now;
        h->key    = k;
        if (dir != 0)                     /* 新的一次按下 → 给一次点按微调 */
        {
            *cur += dir * TUNE_STEP_DEG;
            just_pressed = 1;
        }
    }
    else if (now - h->t_down > TUNE_HOLD_MS)
    {
        uint32_t dt = now - h->t_prev;
        if (dt > 100) dt = 100;           /* 卡顿保护：别一次跳太多 */
        *cur += dir * TUNE_HOLD_DPS * (dt / 1000.0f);
    }
    h->t_prev = now;

    if (*cur < 0.0f)   *cur = 0.0f;
    if (*cur > 180.0f) *cur = 180.0f;

    if (dir != 0)
    {
        Servo_SetAngle(id, *cur);
        /* 点按那一下立刻打印；连续转动时每 500ms 打印一次，别刷屏 */
        if (just_pressed || now - h->t_log >= 500)
        {
            h->t_log = now;
            LOG("[TUNE] id=%d %d.%d deg\r\n", (int)id,
                (int)(*cur), (int)((*cur - (int)(*cur)) * 10));
        }
    }
}

/* ---------------------------------------------------------------------------
 * 按键处理
 * -------------------------------------------------------------------------*/
static void handle_keys(uint32_t now)
{
    /* 急停：按一次锁死，再按一次解除 */
    if (PS2_Clicked(KEY_EMERGENCY))
    {
        g_estop = !g_estop;
        if (g_estop) { Chassis_EmergencyStop(); Auto_Abort(); FullAuto_Stop(); LOG("[ESTOP] ON\r\n"); }
        else         { LOG("[ESTOP] OFF\r\n"); }
    }
    if (g_estop) return;

    /* 模式循环：手动 → 半自动 → 全自动 → 手动 …
     * 退出任何一档时都会顺带中止正在跑的宏/全自动，绝不允许"带着动作换模式" */
    if (PS2_Clicked(KEY_MODE_TOGGLE))
    {
        Auto_Abort();
        FullAuto_Stop();
        g_mode = (g_mode + 1) % 3;
        if (g_mode == MODE_FULL) FullAuto_Start();   /* 全自动：进档即开跑 */
        LOG("[MODE] %s\r\n",
            g_mode == MODE_MANUAL ? "MANUAL" :
            g_mode == MODE_SEMI   ? "SEMI-AUTO" : "FULL-AUTO");
    }

    if (PS2_Clicked(KEY_SERVO_STOW))
    {
        Arm_Start(POSE_STOW);
        LOG("[ARM] stow\r\n");
    }

    if (PS2_Clicked(KEY_SERVO_SCOOP))
    {
        static uint8_t down = 0;
        down = !down;
        Arm_ScoopDown(down);
        LOG("[SCOOP] %s\r\n", down ? "down" : "up");
    }

    /* 半自动宏 */
    if (g_mode == MODE_SEMI)
    {
        if (PS2_Clicked(KEY_MACRO_GROUND)) { Auto_Start(MACRO_GROUND); LOG("[MACRO] ground\r\n"); }
        if (PS2_Clicked(KEY_MACRO_RACK))   { Auto_Start(MACRO_RACK);   LOG("[MACRO] rack\r\n"); }
        if (PS2_Clicked(KEY_MACRO_CAVE))   { Auto_Start(MACRO_CAVE);   LOG("[MACRO] cave\r\n"); }
        if (PS2_Clicked(KEY_MACRO_EDGE))   { Auto_Start(MACRO_EDGE);   LOG("[MACRO] edge\r\n"); }
#if USE_VISION
        if (PS2_Clicked(PSB_L2))           { Auto_Start(MACRO_VISION); LOG("[MACRO] vision\r\n"); }
#endif
    }

    /* ---- 舵机在线标定 + 手动操作 ----
     *   点按 = ±2°（标定，串口打印角度，抄进 config.h）
     *   按住 > 350ms = 连续转 90°/s（手动操作：大臂升降 / 夹爪开合 / 前铲） */
    tune_servo(SERVO_ARM,   PSB_PAD_DOWN, PSB_PAD_UP,    &g_tune_arm,   now);
    tune_servo(SERVO_GRIP,  PSB_PAD_LEFT, PSB_PAD_RIGHT, &g_tune_grip,  now);
#if !USE_VISION
    tune_servo(SERVO_SCOOP, PSB_L2,       PSB_R2,        &g_tune_scoop, now);
#else
    tune_servo(SERVO_SCOOP, 0,            PSB_R2,        &g_tune_scoop, now);
#endif

    /* ---- 宏 / 全自动 运行期间，操作手动了摇杆就立刻交还控制权（安全第一）---- */
    if (Auto_Running() || FullAuto_Running())
    {
        float mv = fabsf(stick_map(ps2.ly, 1.0f)) +
                   fabsf(stick_map(ps2.lx, 1.0f)) +
                   fabsf(stick_map(ps2.rx, 1.0f));
        if (mv > 0.35f)
        {
            Auto_Abort();
            if (FullAuto_Running())
            {
                FullAuto_Stop();
                g_mode = MODE_SEMI;               /* 接管后退回半自动，不自动重来 */
                LOG("[FULL] aborted by operator -> SEMI\r\n");
            }
            else
            {
                LOG("[MACRO] aborted by operator\r\n");
            }
        }
    }

    /* ---- 全自动跑完/放弃后，按○键可以再来一轮 ---- */
    if (g_mode == MODE_FULL && !FullAuto_Running())
    {
        if (PS2_Clicked(KEY_MACRO_GROUND))
        {
            FullAuto_Start();
            LOG("[FULL] restart, picked=%d\r\n", FullAuto_Picked());
        }
    }
    (void)now;
}

/* ---------------------------------------------------------------------------
 * LED 状态指示：一眼看出车处于什么状态，比串口实用
 *  慢闪 1Hz   正常
 *  快闪 5Hz   电池电压低
 *  常亮       手柄失联 / 急停
 * -------------------------------------------------------------------------*/
static void led_task(uint32_t now)
{
    static uint8_t level = 0;
    uint32_t period;

    if (g_estop || !ps2.connected) period = 0xFFFFFFFF;
    else if (Sensor_GetVbat() < VBAT_LOW_WARN) period = 100;
    else period = 500;

    if (period == 0xFFFFFFFF)
    {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);   /* 常亮 */
        return;
    }

    if ((uint32_t)(now - g_t_led) >= period / 2)
    {
        g_t_led = now;
        level = !level;
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, level ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/* ---------------------------------------------------------------------------
 * 周期日志：把关键量打到串口，调参时盯这个看
 * -------------------------------------------------------------------------*/
static void log_task(void)
{
    float w[4];
    Chassis_GetWheel(w);
    float vx, vy, wz;
    Chassis_GetCur(&vx, &vy, &wz);

    LOG("V=%.2f | btn=%04X | L(%4d,%4d) R(%4d,%4d)\r\n",
        Sensor_GetVbat(), ps2.btn, ps2.lx, ps2.ly, ps2.rx, ps2.ry);
    LOG("  cmd v=(%+.2f,%+.2f) w=%+.2f | wheel %+.2f %+.2f %+.2f %+.2f\r\n",
        vx, vy, wz, w[0], w[1], w[2], w[3]);
    LOG("  duty %.2f %.2f %.2f %.2f | arm=%.0f grip=%.0f scoop=%.0f\r\n",
        Motor_GetDuty(0), Motor_GetDuty(1), Motor_GetDuty(2), Motor_GetDuty(3),
        Servo_GetAngle(SERVO_ARM), Servo_GetAngle(SERVO_GRIP), Servo_GetAngle(SERVO_SCOOP));
    LOG("  limit L%d R%d | macro=%d | mode=%s\r\n",
        Limit_Left(), Limit_Right(), Auto_Current(),
        g_mode == MODE_MANUAL ? "man" : g_mode == MODE_SEMI ? "semi" : "full");
    if (FullAuto_Running())
        LOG("  FULL %s | picked=%d fail=%d\r\n",
            FullAuto_StateName(), FullAuto_Picked(), FullAuto_Failed());
#if USE_VISION
    LOG("  vis cls=%d (%d,%d) area=%u\r\n", vision.cls, vision.x, vision.y, vision.area);
#endif
}

/* ========================================================================== */
void App_Init(void)
{
    LOG("\r\n==== CRTC2026 ROBOT-%d BOOT ====\r\n", ROBOT_ID);

    Sensor_Init();
    Servo_Init();
    Chassis_Init();
    Arm_Init();
    Auto_Init();
    FullAuto_Init();
    Vision_Init();
    PS2_Init();

    g_mode  = DEFAULT_MODE;
    g_estop = 0;
    g_t_ctrl = g_t_servo = g_t_log = g_t_led = HAL_GetTick();

    /* 上电就是全自动的话直接开跑（一般别这么设，留着给正式比赛用） */
    if (g_mode == MODE_FULL) FullAuto_Start();

    LOG("init done. mode=%s\r\n",
        g_mode == MODE_MANUAL ? "manual" :
        g_mode == MODE_SEMI   ? "semi-auto" : "full-auto");
}

void App_Tick(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 舵机插值 100Hz ---- */
    if ((uint32_t)(now - g_t_servo) >= SERVO_PERIOD_MS)
    {
        float dt = (float)(now - g_t_servo) / 1000.0f;
        g_t_servo = now;
        Servo_Update(dt);
    }

    /* ---- 机构序列器（内部自己计时，随便喂） ---- */
    Arm_Update(now);

    /* ---- 底盘控制 CTRL_PERIOD_MS ---- */
    if ((uint32_t)(now - g_t_ctrl) >= CTRL_PERIOD_MS)
    {
        float dt = (float)(now - g_t_ctrl) / 1000.0f;
        g_t_ctrl = now;

        PS2_Scan();
        Sensor_Update(now);
        Vision_Update(now);
        Motor_SetVoltageComp(Sensor_GetVbat());

        handle_keys(now);

        if (g_estop)
        {
            Chassis_SetTarget(0, 0, 0);
            Chassis_Update(dt);
        }
        else
        {
            float vx, vy, wz;
            if (FullAuto_Running())
            {
                /* 全自动内部会在 FA_MACRO 状态下驱动 Auto_Update，
                   所以这里**不要**再调一次 Auto_Update，否则一个周期推进两步 */
                FullAuto_Update(now);
                FullAuto_GetCmd(&vx, &vy, &wz);
            }
            else if (Auto_Running())
            {
                Auto_Update(now);
                Auto_GetCmd(&vx, &vy, &wz);
            }
            else
            {
                manual_cmd(&vx, &vy, &wz);
            }

            /* 手柄失联保护：绝不保持上一个指令继续跑 */
            if (!ps2.connected) { vx = vy = wz = 0.0f; }

            Chassis_SetTarget(vx, vy, wz);
            Chassis_Update(dt);
        }
    }

    /* ---- 日志 ---- */
    if ((uint32_t)(now - g_t_log) >= LOG_PERIOD_MS)
    {
        g_t_log = now;
        log_task();
    }

    led_task(now);
}

/* ---------------------------------------------------------------------------
 * UART 接收回调（给视觉模块用）
 * 在 stm32f1xx_it.c 或 main.c 的 HAL_UART_RxCpltCallback 里调用本函数，
 * 或者干脆在 main.c 里这样写：
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *       if (huart->Instance == USART1) {
 *           Vision_RxByte(g_uart1_rx);
 *           HAL_UART_Receive_IT(&huart1, &g_uart1_rx, 1);
 *       }
 *   }
 * -------------------------------------------------------------------------*/
void App_UartRxCallback(UART_HandleTypeDef *huart, uint8_t byte)
{
    if (huart->Instance == USART1) Vision_RxByte(byte);
}
