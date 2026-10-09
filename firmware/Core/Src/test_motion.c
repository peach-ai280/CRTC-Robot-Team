/* ============================================================================
 * test_motion.c —— 只测底盘运动（不用手柄，串口发指令）
 *
 * 【为什么要单独一个固件】
 *   第一次带电调试时，最怕"手柄没对频 + 机构乱动 + 车窜出去"三件事一起来。
 *   这个固件把机构、手柄全部拿掉，只留底盘，你可以把车架空（四个轮子离地），
 *   用串口发一个字母看轮子怎么转，确认方向对了再装轮子落地。
 *
 * 【安全前提】
 *   第一次上电务必架空（用两个书本把车架起来，四轮悬空），
 *   确认命令和轮子转向对得上之后，再放到地上。
 *
 * 【串口指令（115200，发完不用回车）】
 *   w / s   前进 / 后退
 *   a / d   左移 / 右移
 *   q / e   原地逆时针 / 顺时针
 *   z       斜向 45° 组合动作（前进 + 右移，检验归一化限幅）
 *   空格    停止
 *   r       自动演示：依次走 前进/后退/左移/右移/左转/右转/斜走，每段 1.5 秒
 *   + / -   速度档位 ±0.1（0.1 ~ 1.0）
 *
 * 【看什么】
 *   打印格式：cmd = 指令速度    wheel = 四个轮的解算值    duty = 实际占空比
 *   纯前进：四个 wheel 必须同号同值
 *   纯左移：LF/RB 为负、RF/LB 为正
 *   纯逆时针：左两轮负、右两轮正
 *   这三条对不上，说明 config.h 里 MOTOR_x_DIR 或车体尺寸填错了。
 * ==========================================================================*/

#include "main.h"
#include "bsp.h"
#include "board.h"
#include "config.h"
#include "mecanum.h"
#include "chassis_diff.h"
#include "motor.h"
#include "sensor.h"
#include <stdio.h>

/* ============================================================================
 * 【A 车 / B 车 用同一个测试固件】
 *   config.h 里 CHASSIS_TYPE 决定编译进哪一套运动学：
 *     CHASSIS_TYPE_MECANUM (0)  -> Chassis_*  （1 号车，麦轮，能横移）
 *     CHASSIS_TYPE_DIFF    (1)  -> Diff_*     （2 号车，橡胶轮，只能前后+转）
 *   两边接口同形，所以下面的 main 一个字都不用改。
 *
 * ⚠ 切 B 车后，手柄的"横移"那一档会失效（物理上就不存在），
 *   串口测试时按 a/d 会**没有反应**，这不是 bug，是橡胶轮的物理限制。
 * ==========================================================================*/
#if (CHASSIS_TYPE == CHASSIS_TYPE_DIFF)
  #define CAR_INIT()        Diff_Init()
  #define CAR_SET(vx,vy,wz) Diff_SetTarget((vx),(vy),(wz))
  #define CAR_UPDATE(dt)    Diff_Update(dt)
  #define CAR_GETWHEEL(w)   Diff_GetWheel(w)
  #define CAR_NAME          "B-DIFF (rubber wheels, tank steer)"
#else
  #define CAR_INIT()        Chassis_Init()
  #define CAR_SET(vx,vy,wz) Chassis_SetTarget((vx),(vy),(wz))
  #define CAR_UPDATE(dt)    Chassis_Update(dt)
  #define CAR_GETWHEEL(w)   Chassis_GetWheel(w)
  #define CAR_NAME          "A-MECANUM (rollers, omni)"
#endif

static volatile char g_cmd = 0;
static float g_throttle = 1.0f;
static uint8_t g_demo = 0;

/* 串口收到一个字节 = 一条指令 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        g_cmd = (char)g_uart1_rx;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_rx, 1);
    }
}

/* 自动演示：每 1500ms 换一个动作 */
static void demo_cmd(uint32_t now, float *vx, float *vy, float *wz)
{
    uint8_t phase = (uint8_t)((now / 1500UL) % 7UL);
    *vx = *vy = *wz = 0;
    switch (phase)
    {
    case 0: *vx =  0.5f; break;   /* 前进 */
    case 1: *vx = -0.5f; break;   /* 后退 */
    case 2: *vy =  0.5f; break;   /* 左移 */
    case 3: *vy = -0.5f; break;   /* 右移 */
    case 4: *wz =  0.5f; break;   /* 逆时针 */
    case 5: *wz = -0.5f; break;   /* 顺时针 */
    default:*vx = 0.5f; *vy = -0.5f; break;  /* 斜走，测归一化 */
    }
    *vx *= g_throttle; *vy *= g_throttle; *wz *= g_throttle;
}

int main(void)
{
    float vx = 0, vy = 0, wz = 0;
    uint32_t t_ctrl = 0, t_log = 0;

    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_Init();
    BSP_TIM4_Init();
    BSP_ADC1_Init();
    BSP_USART1_Init();

    Sensor_Init();
    CAR_INIT();

    printf("\r\n==== MOTION TEST ====\r\n");
    printf("CHASSIS = %s\r\n", CAR_NAME);
    printf("SYSCLK = %lu Hz\r\n", SystemCoreClock);
    printf("cmds: w/s=fwd/back  a/d=strafe  q/e=yaw  z=diag  space=stop  r=demo  +/- =throttle\r\n");

    t_ctrl = t_log = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ---- 解析指令 ---- */
        if (g_cmd)
        {
            char c = g_cmd;
            g_cmd = 0;
            switch (c)
            {
            case 'w': g_demo = 0; vx =  g_throttle; vy = 0; wz = 0; break;
            case 's': g_demo = 0; vx = -g_throttle; vy = 0; wz = 0; break;
            case 'a': g_demo = 0; vx = 0; vy =  g_throttle; wz = 0; break;
            case 'd': g_demo = 0; vx = 0; vy = -g_throttle; wz = 0; break;
            case 'q': g_demo = 0; vx = 0; vy = 0; wz =  g_throttle; break;
            case 'e': g_demo = 0; vx = 0; vy = 0; wz = -g_throttle; break;
            case 'z': g_demo = 0; vx = g_throttle; vy = -g_throttle; wz = 0; break;
            case ' ': g_demo = 0; vx = vy = wz = 0; break;
            case 'r': g_demo = !g_demo; break;
            case '+': g_throttle += 0.1f; if (g_throttle > 1.0f) g_throttle = 1.0f; break;
            case '-': g_throttle -= 0.1f; if (g_throttle < 0.1f) g_throttle = 0.1f; break;
            default: break;
            }
        }

        /* ---- 5ms 控制周期 ---- */
        if ((uint32_t)(now - t_ctrl) >= CTRL_PERIOD_MS)
        {
            float dt = (float)(now - t_ctrl) / 1000.0f;
            t_ctrl = now;

            if (g_demo) demo_cmd(now, &vx, &vy, &wz);

            CAR_SET(vx, vy, wz);
            CAR_UPDATE(dt);
        }

        /* ---- 200ms 打印 ---- */
        if ((uint32_t)(now - t_log) >= LOG_PERIOD_MS)
        {
            t_log = now;
            float w[4];
            CAR_GETWHEEL(w);
            printf("cmd(%+.2f,%+.2f,%+.2f) wheel[%+.2f %+.2f %+.2f %+.2f] duty[%.2f %.2f %.2f %.2f] vbat=%.2f %s\r\n",
                   vx, vy, wz, w[0], w[1], w[2], w[3],
                   Motor_GetDuty(0), Motor_GetDuty(1), Motor_GetDuty(2), Motor_GetDuty(3),
                   Sensor_GetVbat(), g_demo ? "DEMO" : "");
        }
    }
}
