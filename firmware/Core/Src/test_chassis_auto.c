/* ============================================================================
 * test_chassis_auto.c —— 底盘自检（不用手柄、不用串口）
 *
 * 【为什么要这个】
 *   手柄还没通、又没有 USB 转串口模块的时候，也想确认电机接线对不对。
 *   这个固件上电后自己跑一遍：前进 / 后退 / 左移 / 右移 / 逆时针 / 顺时针 / 斜走，
 *   每段 1.5 秒，中间停 0.8 秒，**跑完一轮就永久停住，不会一直跑**。
 *
 * 【⚠ 安全前提，务必照做】
 *   ① 第一次上电一定要把车架起来（两本书垫住底盘，四轮离地）
 *   ② 手放在电源旁边，车一动方向不对就断电
 *   ③ 跑完自动停，想再测就复位（按板子上的 R 键 / 重新插拔 USB）
 *
 * 【LED（PC13，低电平亮）在说什么】
 *   上电前 3 秒：2Hz 快闪   = 倒计时，马上开始，别松手
 *   动作进行中：常亮        = 正在输出速度
 *   动作间隙：  灭          = 停
 *   全部跑完：  常亮不闪    = 结束，已停住
 *
 * 【看什么】
 *   前进：四个轮子都往前（车头方向）
 *   后退：四个轮子都往后
 *   左移：LF/RB 往前、RF/LB 往后（麦轮）
 *   逆时针：左边两轮往后、右边两轮往前
 *   有某一个轮子跟别人反着转 → 改 config.h 里的 MOTOR_x_DIR
 * ==========================================================================*/

#include "main.h"
#include "bsp.h"
#include "board.h"
#include "config.h"
#include "mecanum.h"
#include "chassis_diff.h"
#include "motor.h"
#include "sensor.h"

#if (CHASSIS_TYPE == CHASSIS_TYPE_DIFF)
  #define CAR_INIT()        Diff_Init()
  #define CAR_SET(vx,vy,wz) Diff_SetTarget((vx),(vy),(wz))
  #define CAR_UPDATE(dt)    Diff_Update(dt)
#else
  #define CAR_INIT()        Chassis_Init()
  #define CAR_SET(vx,vy,wz) Chassis_SetTarget((vx),(vy),(wz))
  #define CAR_UPDATE(dt)    Chassis_Update(dt)
#endif

/* --------------------------------------------------------------------------
 * 参数：想改这里，不用动逻辑
 * ------------------------------------------------------------------------*/
#define AUTO_START_DELAY_MS   3000      /* 上电等多久才开始动（给你反应时间） */
#define STEP_MS               1500      /* 每个动作持续多久 */
#define GAP_MS                 800      /* 动作之间停多久 */
#define AUTO_SPEED            0.45f     /* 速度 0~1，第一次建议别超过 0.5 */
#define AUTO_STEPS               7      /* 一共几个动作 */

int main(void)
{
    uint32_t t_ctrl;

    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_Init();
    BSP_TIM4_Init();
    BSP_ADC1_Init();

    Sensor_Init();
    CAR_INIT();

    /* 开机指示：快闪 3 下，肉眼确认固件真的在跑 */
    for (int i = 0; i < 6; i++)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);      /* 灭 */

    uint32_t t0 = HAL_GetTick();
    t_ctrl = t0;

    uint8_t  done = 0;

    while (1)
    {
        uint32_t now     = HAL_GetTick();
        uint32_t elapsed = now - t0;
        float vx = 0.0f, vy = 0.0f, wz = 0.0f;

        /* ---------------- 动作调度 ---------------- */
        if (!done && elapsed >= AUTO_START_DELAY_MS)
        {
            uint32_t k      = elapsed - AUTO_START_DELAY_MS;
            uint32_t cycle  = STEP_MS + GAP_MS;
            uint8_t  s      = (uint8_t)(k / cycle);

            if (s >= AUTO_STEPS)
            {
                done = 1;                       /* 全部跑完，永久停住 */
            }
            else
            {
                /* 只有"动作段"里才输出速度，"间隙段"给 0 */
                if ((k % cycle) < STEP_MS)
                {
                    switch (s)
                    {
                    case 0: vx =  AUTO_SPEED;              break;  /* 前进 */
                    case 1: vx = -AUTO_SPEED;              break;  /* 后退 */
                    case 2: vy =  AUTO_SPEED;              break;  /* 左移 */
                    case 3: vy = -AUTO_SPEED;              break;  /* 右移 */
                    case 4: wz =  AUTO_SPEED;              break;  /* 逆时针 */
                    case 5: wz = -AUTO_SPEED;              break;  /* 顺时针 */
                    default:vx =  AUTO_SPEED; vy = -AUTO_SPEED; break;  /* 斜走 */
                    }
                }
            }
        }

        /* ---------------- 5ms 控制周期 ---------------- */
        if ((uint32_t)(now - t_ctrl) >= CTRL_PERIOD_MS)
        {
            float dt = (float)(now - t_ctrl) / 1000.0f;
            t_ctrl = now;
            CAR_SET(vx, vy, wz);
            CAR_UPDATE(dt);
        }

        /* ---------------- LED ---------------- */
        if (done)
        {
            /* 全部跑完：常亮 */
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        }
        else if (elapsed < AUTO_START_DELAY_MS)
        {
            /* 倒计时：2Hz 快闪 */
            HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                              ((now / 250u) & 1u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
        else
        {
            /* 跑动中：在动就亮，间隙就灭 */
            HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                              (vx != 0.0f || vy != 0.0f || wz != 0.0f)
                              ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
    }
}
