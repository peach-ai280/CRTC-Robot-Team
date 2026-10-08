#ifndef __BOARD_H
#define __BOARD_H

/* ============================================================================
 * board.h —— CRTC2026 「机器人总动员战队」统一硬件引脚映射
 * 适用：STM32F103C8T6 核心板（蓝丸 / Micro_USB 焊接款）
 * 两台车（1号机 / 2号机）使用完全相同的引脚定义，只有 config.h 里的
 *   宏 ROBOT_ID 和机构角度不同。刷机前务必改 ROBOT_ID！
 * ==========================================================================*/

/* ---------------------------------------------------------------------------
 * 1. 外设资源总表（改引脚只需改这里 + CubeMX 里同步，别两处写死）
 * -------------------------------------------------------------------------*/
/*
 *  TIM2  CH1..CH4  -> 4 路舵机 PWM，50Hz（PA0/PA1/PA2/PA3）
 *  TIM4  CH1..CH4  -> 4 路电机 PWM，10kHz（PB6/PB7/PB8/PB9）
 *  SPI1            -> PS2 手柄（SCK=PA5, MISO=PA6, MOSI=PA7, CS=PB0 软件片选）
 *  USART1          -> 调试串口 + 视觉模块（TX=PA9, RX=PA10, 115200）
 *  ADC1 IN9        -> 电池电压检测（PB1，1/3 分压）
 *  GPIO            -> 电机方向 x4、限位开关 x2、LED、按键、蜂鸣器
 *
 *  为什么电机占 TIM4 而不占 TIM1？
 *    TIM1 的 CH1/CH2/CH3 在 PA8/PA9/PA10，会和 USART1 的 TX/RX 撞车；
 *    TIM2/TIM3 又要给舵机和将来的编码器留着。TIM4 的 PB6~PB9 是唯一
 *    四个通道全在一组、且不和任何通信外设冲突的选择。
 * -------------------------------------------------------------------------*/

/* --- PS2 手柄（SPI1）---------------------------------------------------- */
#define PS2_SPI              hspi1          /* CubeMX 里 SPI1 句柄名 */
#define PS2_CS_PORT          GPIOB
#define PS2_CS_PIN           GPIO_PIN_0

/* --- 电机 PWM（TIM4，10kHz）--------------------------------------------- */
/* 轮序约定（车体坐标：x 前，y 左，ω 逆时针为正） */
#define MOTOR_LF   0     /* 左前轮 */
#define MOTOR_RF   1     /* 右前轮 */
#define MOTOR_LB   2     /* 左后轮 */
#define MOTOR_RB   3     /* 右后轮 */

extern void MX_TIM4_Init(void);   /* CubeMX 生成 */
#define MOTOR_PWM_TIM        htim4
#define MOTOR_TIM_CH_LF      TIM_CHANNEL_1   /* PB6 */
#define MOTOR_TIM_CH_RF      TIM_CHANNEL_2   /* PB7 */
#define MOTOR_TIM_CH_LB      TIM_CHANNEL_3   /* PB8 */
#define MOTOR_TIM_CH_RB      TIM_CHANNEL_4   /* PB9 */

/* --- 电机方向（DRV8833 的 IN2 脚，IN1 接 PWM：PHASE/ENABLE 模式）-------- */
/* 详见 motor.c 里的说明：IN1=PWM, IN2=方向。反向时占空比 = 100 - |速度| */
#define MOTOR_DIR_PORT       GPIOB
#define MOTOR_LF_DIR_PIN     GPIO_PIN_12
#define MOTOR_RF_DIR_PIN     GPIO_PIN_13
#define MOTOR_LB_DIR_PIN     GPIO_PIN_14
#define MOTOR_RB_DIR_PIN     GPIO_PIN_15

/* --- 舵机 PWM（TIM2，50Hz）---------------------------------------------- */
#define SERVO_TIM            htim2
#define SERVO_TIM_CH_GRIP    TIM_CHANNEL_1   /* PA0  夹爪  SG90  */
#define SERVO_TIM_CH_ARM     TIM_CHANNEL_2   /* PA1  大臂  MG995 */
#define SERVO_TIM_CH_SCOOP   TIM_CHANNEL_3   /* PA2  前铲  SG90  */
#define SERVO_TIM_CH_EXTRA   TIM_CHANNEL_4   /* PA3  备用  SG90  */

/* --- 传感器 ------------------------------------------------------------- */
#define LIMIT_L_PORT         GPIOB           /* 对中喇叭口 左微动开关 */
#define LIMIT_L_PIN          GPIO_PIN_10     /* 按下 = 低电平（内部上拉） */
#define LIMIT_R_PORT         GPIOB           /* 对中喇叭口 右微动开关 */
#define LIMIT_R_PIN          GPIO_PIN_11

#define VBAT_ADC             hadc1           /* PB1 / ADC1_IN9 */
#define VBAT_ADC_CHANNEL     ADC_CHANNEL_9

/* --- 人机交互 ----------------------------------------------------------- */
#define LED_PORT             GPIOC
#define LED_PIN              GPIO_PIN_13     /* 蓝丸板载 LED，低电平点亮 */
#define KEY_PORT             GPIOB
#define KEY_PIN              GPIO_PIN_5      /* 上拉输入，按下为低 */
#define BUZZ_PORT            GPIOA
#define BUZZ_PIN             GPIO_PIN_8      /* 有源蜂鸣器，可选 */

/* --- 调试/视觉串口 ------------------------------------------------------ */
#define DBG_UART             huart1          /* PA9/PA10, 115200 */

/* ============================================================================
 * 2. 车体物理尺寸（单位：米）—— 打印件装好后用尺子实测填这里
 *    这几个数直接决定麦轮解算对不对，测错会“横移时自己转圈”
 * ==========================================================================*/
/* 以下三个数必须和机械设计一致（见 mechanical/gen_parts.py 的 PARAMS）：
 *   WHEELBASE = 180mm  => 半轴距 90mm
 *   TRACK     = 136mm  => 半轮距 68mm
 *   麦轮外径  60mm     => 半径 30mm
 * 装好后请用尺子复测，填实测值。填错的表现：横移时车会自己转圈。 */
#define ROBOT_HALF_WHEELBASE  0.090f
#define ROBOT_HALF_TRACK      0.068f
#define WHEEL_RADIUS          0.030f

#endif /* __BOARD_H */
