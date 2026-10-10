/* ============================================================================
 * test_arm.c —— 只测舵机与机构（底盘电机不输出）
 *
 * 【为什么单独一个固件】
 *   舵机调试最容易出的事故是"角度超行程"：舵机顶到机械限位还硬撑，
 *   电流飙升 -> 5V 被拉垮 -> 单片机复位。单独测的时候你能一个一个来，
 *   看清楚每个角度对应什么姿态，还不会碰到轮子转动。
 *
 * 【安全前提】
 *   1. 舵机信号线接好之前不要上电（舵机缺信号会抖）
 *   2. 听到"吱吱"声 = 脉宽超行程，立刻往回收，别硬撑
 *   3. MG995 堵转电流 >1A，别用 USB 口 5V 带它，用车上那路降压
 *   4. ★ 本固件用 BSP_TIM1_Init() 开了第 5 路舵机（PA11）。
 *      调车时**别把 micro-USB 插上当电源** —— PA11/PA12 就是 USB 的 D-/D+。
 *
 * 【串口指令（115200，不用回车）】
 *   1  POSE_STOW        全部收起（检录初始态）
 *   2  POSE_READY       待命（臂转正前 + 前铲放下）
 *   3  POSE_PICK_GROUND 地面取块（★ 只用铲兜）
 *   4  POSE_PICK_STEP   台阶面取块（臂 φ=+19°）
 *   5  POSE_STORE       入仓（抬平 → 转正后 → 松爪）
 *   6  POSE_RACK_HOOK   挑物资架黄块（φ=+90°）
 *   7  POSE_CAVE_PROBE  洞窟探杆
 *   8  POSE_PICK_FOCUS  中央焦点黄块（φ=−26°）
 *
 *   ★ 手动模式（会切到手动，之后序列不再覆盖）：
 *     i / k   夹爪角度   +5 / −5
 *     o / l   大臂摆角 φ +5 / −5      ← φ 就是设计口径（0=水平，+ 抬头）
 *     y / h   臂朝向 yaw +10 / −10    ← 0=正前，180=正后
 *     p / ;   前铲角度   +5 / −5
 *     m       打印五路舵机实际角度 + 当前 φ / yaw
 *   空格     停止当前序列，回到收起位
 *
 * 【标定流程（推荐顺序）】
 *   1. 先按 m 看当前角度
 *   2. 用 i/k/o/l/y/h/p/; 把机构摆到想要的姿态
 *   3. 按 m 读出角度，抄进 config.h 里对应的 SERVO_xxx 宏
 *   4. 重烧完整固件，按一次按键验证动作一致
 * ==========================================================================*/

#include "main.h"
#include "bsp.h"
#include "board.h"
#include "config.h"
#include "servo.h"
#include "arm.h"
#include <stdio.h>

static volatile char g_cmd = 0;
static float   g_grip  = 0.0f;                              /* 夹爪角度 */
static float   g_phi   = 0.0f;                              /* 大臂摆角 φ */
static float   g_yaw   = 0.0f;                              /* 臂朝向 */
static float   g_scoop = 0.0f;                              /* 前铲角度 */
static uint8_t g_manual_mode = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        g_cmd = (char)g_uart1_rx;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_rx, 1);
    }
}

static void print_angles(void)
{
    printf("ANG  grip=%.0f arm=%.0f(phi=%+.0f) yaw=%.0f scoop=%.0f extra=%.0f  "
           "(busy=%d)\r\n",
           Servo_GetAngle(SERVO_GRIP), Servo_GetAngle(SERVO_ARM), g_phi,
           Servo_GetAngle(SERVO_YAW), Servo_GetAngle(SERVO_SCOOP),
           Servo_GetAngle(SERVO_EXTRA), Arm_IsBusy());
}

int main(void)
{
    uint32_t t_servo = 0, t_log = 0;

    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_Init();
    BSP_TIM2_Init();
    BSP_TIM1_Init();          /* ★ 第 5 路舵机（PA11）—— 少了这一句，回转舵机不动 */
    BSP_USART1_Init();

    Servo_Init();
    Arm_Init();

    /* 手动模式的初值取收起姿态，避免一上来就甩 */
    g_grip  = SERVO_GRIP_OPEN;
    g_phi   = 0.0f;                       /* φ=0 → 臂水平 → 舵机角 90 */
    g_yaw   = SERVO_YAW_STOW;
    g_scoop = SERVO_SCOOP_TRAVEL;

    printf("\r\n==== ARM TEST (5 servos) ====\r\n");
    printf("SYSCLK = %lu Hz\r\n", SystemCoreClock);
    printf("poses: 1=stow 2=ready 3=pick_ground 4=pick_step 5=store "
           "6=rack 7=cave 8=focus\r\n");
    printf("manual: i/k grip  o/l arm(phi)  y/h yaw  p/; scoop  m=print  "
           "space=stow\r\n");

    t_servo = t_log = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (g_cmd)
        {
            char c = g_cmd;
            g_cmd = 0;
            switch (c)
            {
            case '1': g_manual_mode = 0; Arm_Start(POSE_STOW);         break;
            case '2': g_manual_mode = 0; Arm_Start(POSE_READY);        break;
            case '3': g_manual_mode = 0; Arm_Start(POSE_PICK_GROUND);  break;
            case '4': g_manual_mode = 0; Arm_Start(POSE_PICK_STEP);    break;
            case '5': g_manual_mode = 0; Arm_Start(POSE_STORE);        break;
            case '6': g_manual_mode = 0; Arm_Start(POSE_RACK_HOOK);    break;
            case '7': g_manual_mode = 0; Arm_Start(POSE_CAVE_PROBE);   break;
            case '8': g_manual_mode = 0; Arm_Start(POSE_PICK_FOCUS);   break;
            case ' ': g_manual_mode = 0; Arm_Start(POSE_STOW);         break;

            case 'i': g_manual_mode = 1; g_grip  += 5.0f;  break;
            case 'k': g_manual_mode = 1; g_grip  -= 5.0f;  break;
            case 'o': g_manual_mode = 1; g_phi   += 5.0f;  break;   /* φ 每档 5° */
            case 'l': g_manual_mode = 1; g_phi   -= 5.0f;  break;
            case 'y': g_manual_mode = 1; g_yaw   += 10.0f; break;
            case 'h': g_manual_mode = 1; g_yaw   -= 10.0f; break;
            case 'p': g_manual_mode = 1; g_scoop += 5.0f;  break;
            case ';': g_manual_mode = 1; g_scoop -= 5.0f;  break;

            case 'm': print_angles(); break;
            default: break;
            }

            if (g_manual_mode)
            {
                /* φ / 角度都在各自函数里做限幅，别在这里自己截 */
                if (g_grip > 180.0f) g_grip = 180.0f;
                if (g_grip <   0.0f) g_grip = 0.0f;
                if (g_yaw  > 180.0f) g_yaw  = 180.0f;
                if (g_yaw  <   0.0f) g_yaw  = 0.0f;
                if (g_scoop > 180.0f) g_scoop = 180.0f;
                if (g_scoop <   0.0f) g_scoop = 0.0f;
                if (g_phi  >  90.0f) g_phi  = 90.0f;
                if (g_phi  < -50.0f) g_phi  = -50.0f;

                Servo_SetAngle(SERVO_GRIP,  g_grip);
                Arm_SetPitch(g_phi);                 /* 内部转成 s = 90 + φ */
                Arm_SetYaw(g_yaw);
                Servo_SetAngle(SERVO_SCOOP, g_scoop);
            }
        }

        /* 舵机缓动 10ms */
        if ((uint32_t)(now - t_servo) >= SERVO_PERIOD_MS)
        {
            float dt = (float)(now - t_servo) / 1000.0f;
            t_servo = now;
            Servo_Update(dt);
        }

        Arm_Update(now);

        /* 500ms 打印一次 */
        if ((uint32_t)(now - t_log) >= 500)
        {
            t_log = now;
            print_angles();
        }
    }
}
