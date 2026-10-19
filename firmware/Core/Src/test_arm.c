/* ============================================================================
 * test_arm.c —— 只测舵机与机械臂（底盘电机不输出）
 *
 * 【构型】柱坐标：θ 回转 / Z 升降 / 夹爪 —— 三轴独立，没有逆解。
 *
 * 【为什么单独一个固件】
 *   舵机调试最容易出的事故是"角度超行程"：舵机顶到机械限位还硬撑，
 *   电流飙升 -> 5V 被拉垮 -> 单片机复位。单独测的时候你能一个一个来，
 *   看清楚每个角度对应什么姿态，还不会碰到轮子转动。
 *
 * 【安全前提】
 *   1. 舵机信号线接好之前不要上电（舵机缺信号会抖）
 *   2. 听到"吱吱"声 = 顶到机械限位/脉宽超行程，立刻往回收，别硬撑
 *   3. ★ 柱坐标下最危险的动作是**低位回转**：爪尖最低 64.5 < 甲板面 88.5，
 *      低位硬转会把爪插进甲板/储仓。本固件和 arm.c 都有互锁（会先把升降抬到
 *      最高再转），但**手动单测时也请先升上去再转**，别依赖互锁。
 *   4. ★ 本固件用 BSP_TIM1_Init() 开了第 5 路舵机（PA11）。
 *      调车时**别把 micro-USB 插上当电源** —— PA11/PA12 就是 USB 的 D-/D+。
 *
 * 【串口指令（115200，不用回车）】
 *   1  POSE_STOW           全部收起（检录初始态：升最高 → 转正后）
 *   2  POSE_READY          待命（转正前 + 前铲放下）
 *   3  POSE_PICK_GROUND    地面取块（★ 只用铲兜）
 *   4  POSE_PICK_FOCUS     焦点黄块（爪尖 ~85）
 *   5  POSE_PICK_PLATFORM  高台面白块（爪尖 ~87.5）
 *   6  POSE_PICK_STEP      台阶面白块（爪尖 95，只够下半截）
 *   7  POSE_STORE          入仓（升最高 → 转正后 → 松爪）
 *   8  POSE_RACK_HOOK      挑物资架（★ 用挑杆，臂够不到）
 *   9  POSE_CAVE_PROBE     洞窟探杆
 *
 *   ★ 手动模式（按键后自动切手动，序列不再覆盖）：
 *     i / k   夹爪角度   +5 / −5
 *     o / l   升降       +5mm / −5mm   ← ★ 给的是"爪尖离地高度"，内部查标定表
 *     y / h   回转       +10° / −10°   ← 0=正前，180=正后（带互锁）
 *     p / ;   前铲角度   +5 / −5
 *     m       打印全部实际角度 + 爪尖高度 + 是否被互锁拦
 *   空格     停止当前序列，回到收起位
 *
 * 【标定流程（这就是产 ARM_LIFT 标定表的全部步骤）】
 *   1. 先按下升降到最低（连按 l），按 m 读舵机角 → 填 arm_cal.h 的 ARM_CAL_DEG_0，
 *      同时拿尺子量爪尖离地 → 填 ARM_CAL_TIP_0
 *   2. 每按 o 加 5mm，量一次爪尖高度、读一次角度 → 每隔约 6mm 记一个点
 *   3. 一共记满 6 个点 → 替换 arm_cal.h 的 ARM_CAL_TIP_1..5 / ARM_CAL_DEG_1..5
 *   4. 回转同理：y/h 转到 0° / 90° / 180°，读角度 → 填 ARM_YAW_DEG_0/90/180
 *   5. 重烧完整固件，按一次姿态键验证动作一致
 * ==========================================================================*/

#include "main.h"
#include "bsp.h"
#include "board.h"
#include "config.h"
#include "arm_cal.h"
#include "servo.h"
#include "arm.h"
#include <stdio.h>

static volatile char g_cmd = 0;
static float   g_grip  = 0.0f;                              /* 夹爪角度 */
static float   g_tip   = 0.0f;                              /* ★ 升降：爪尖离地高度 mm */
static float   g_yaw   = 0.0f;                              /* ★ 回转：朝向角 */
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
    printf("ANG grip=%.0f | lift=%.0f(deg) tip=%.1f(mm) | yaw=%.0f%s | scoop=%.0f extra=%.0f | busy=%d\r\n",
           Servo_GetAngle(SERVO_GRIP),
           Arm_LiftDegNow(), Arm_LiftMmNow(),
           Servo_GetAngle(SERVO_YAW), Arm_YawLocked() ? "(LOCK)" : "",
           Servo_GetAngle(SERVO_SCOOP), Servo_GetAngle(SERVO_EXTRA),
           Arm_IsBusy());
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
    g_tip   = ARM_TIP_HOME;               /* 爪尖最高 = 检录/回转安全高度 */
    g_yaw   = SERVO_YAW_STOW;
    g_scoop = SERVO_SCOOP_TRAVEL;

    printf("\r\n==== ARM TEST (cylindrical: yaw + lift + grip) ====\r\n");
    printf("SYSCLK = %lu Hz\r\n", SystemCoreClock);
    printf("poses: 1=stow 2=ready 3=pick_ground 4=focus 5=platform 6=step "
           "7=store 8=rack 9=cave\r\n");
    printf("manual: i/k grip  o/l lift(mm)  y/h yaw  p/; scoop  m=print  space=stow\r\n");
    printf("cal: 爪尖最低 %.1fmm / 最高 %.1fmm，表里的名义值必须实测替换\r\n",
           ARM_GEO_CLAW_TIP_LO, ARM_GEO_CLAW_TIP_HI);

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
            case '1': g_manual_mode = 0; Arm_Start(POSE_STOW);          break;
            case '2': g_manual_mode = 0; Arm_Start(POSE_READY);         break;
            case '3': g_manual_mode = 0; Arm_Start(POSE_PICK_GROUND);   break;
            case '4': g_manual_mode = 0; Arm_Start(POSE_PICK_FOCUS);    break;
            case '5': g_manual_mode = 0; Arm_Start(POSE_PICK_PLATFORM); break;
            case '6': g_manual_mode = 0; Arm_Start(POSE_PICK_STEP);     break;
            case '7': g_manual_mode = 0; Arm_Start(POSE_STORE);         break;
            case '8': g_manual_mode = 0; Arm_Start(POSE_RACK_HOOK);     break;
            case '9': g_manual_mode = 0; Arm_Start(POSE_CAVE_PROBE);    break;
            case ' ': g_manual_mode = 0; Arm_Start(POSE_STOW);          break;

            case 'i': g_manual_mode = 1; g_grip  += 5.0f;  break;
            case 'k': g_manual_mode = 1; g_grip  -= 5.0f;  break;
            case 'o': g_manual_mode = 1; g_tip   += 5.0f;  break;   /* 爪尖 +5mm */
            case 'l': g_manual_mode = 1; g_tip   -= 5.0f;  break;   /* 爪尖 −5mm */
            case 'y': g_manual_mode = 1; g_yaw   += 10.0f; break;
            case 'h': g_manual_mode = 1; g_yaw   -= 10.0f; break;
            case 'p': g_manual_mode = 1; g_scoop += 5.0f;  break;
            case ';': g_manual_mode = 1; g_scoop -= 5.0f;  break;

            case 'm': print_angles(); break;
            default: break;
            }

            if (g_manual_mode)
            {
                /* 各角度都在各自函数里做限幅，别在这里自己截 */
                if (g_grip > 180.0f) g_grip = 180.0f;
                if (g_grip <   0.0f) g_grip = 0.0f;
                if (g_yaw  > 180.0f) g_yaw  = 180.0f;
                if (g_yaw  <   0.0f) g_yaw  = 0.0f;
                if (g_scoop > 180.0f) g_scoop = 180.0f;
                if (g_scoop <   0.0f) g_scoop = 0.0f;
                /* 爪尖高度的上下限 = 标定表的两个端点（曲面端点之外无处可去） */
                if (g_tip > ARM_GEO_CLAW_TIP_HI) g_tip = ARM_GEO_CLAW_TIP_HI;
                if (g_tip < ARM_GEO_CLAW_TIP_LO) g_tip = ARM_GEO_CLAW_TIP_LO;

                Servo_SetAngle(SERVO_GRIP,  g_grip);
                Arm_SetLift(g_tip);                  /* 内部 mm → 舵机角（查标定表） */
                Arm_SetYaw(g_yaw);                   /* 内部带互锁 */
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
