#ifndef __CONFIG_H
#define __CONFIG_H

/* ============================================================================
 * config.h —— 全车唯一「调参入口」
 *
 * 【给新手的话】调试期间你 95% 的时间只会改这个文件。改完重新烧录即可，
 * 不用动任何 .c。每一个宏后面都写了「调大的后果 / 调小的后果 / 什么现象该改它」。
 *
 * 标定顺序（务必按这个顺序，否则会互相干扰）：
 *   第 0 步  ROBOT_ID / PS2 按键极性
 *   第 1 步  MOTOR_x_DIR 轮转向
 *   第 2 步  MOTOR_DEADZONE 死区
 *   第 3 步  SPEED_GAIN_* 速度标定
 *   第 4 步  CHASSIS_ACC_* 加速度斜坡（防滑）
 *   第 5 步  舵机角度 SERVO_*_*
 *   第 6 步  半自动宏时间参数 AUTO_*
 * ==========================================================================*/

/* ---------------------------------------------------------------------------
 * 0. 身份与模式
 * -------------------------------------------------------------------------*/
/* 1 = 1号机（高台机，从 a4/h5 高台出发，走桥抢中央焦点）
 * 2 = 2号机（地面机，从 a1/h8 出发，扫地面能量单元 + 挑物资架/洞窟） */
#define ROBOT_ID                 1

/* 三档模式（SELECT 键循环切换：手动 → 半自动 → 全自动 → 手动…）
 *   MODE_MANUAL 纯手动：摇杆直接开车，不触发任何自动动作
 *   MODE_SEMI   半自动：操作手按 ○/△/□/× 触发一键宏，决策权在人
 *   MODE_FULL   全自动：车自己巡游找块、自动触发宏，人不用管（需组委会认定）
 *
 * ⚠ 递进关系：全自动是**在半自动的宏之上**加一层自动调度（auto_full.c），
 *   并没有重写抓取动作。所以必须先把半自动调稳，再上全自动。 */
#define MODE_MANUAL              0
#define MODE_SEMI                1
#define MODE_FULL                2

/* 上电默认模式。推荐 MODE_SEMI：中期考核靠它拿分，最稳。
 * 想上场测全自动，把这里改成 MODE_FULL，或上电后按 SELECT 切过去。 */
#define DEFAULT_MODE             MODE_SEMI

/* ---------------------------------------------------------------------------
 * 1. 电机与防滑（最重要的一个板块）
 * -------------------------------------------------------------------------*/
/* 轮转向修正：某个轮转反了，把对应位改成 -1。这是最常见的“车原地打转”元凶 */
#define MOTOR_LF_DIR             1
#define MOTOR_RF_DIR            -1
#define MOTOR_LB_DIR             1
#define MOTOR_RB_DIR            -1

/* 电机安装方向修正：如果整车前进/横移方向反了，但四个轮都同向，改这里
 *   FORWARD_SIGN 前后（摇杆向上应该是前进）
 *   STRAFE_SIGN  左右横移
 *   YAW_SIGN     自转方向 */
#define FORWARD_SIGN             1
#define STRAFE_SIGN              1
#define YAW_SIGN                 1

/* 死区补偿（0~1）：TT 马达 + DRV8833 在占空比低于这个值时不转。
 *   现象：推小油门车不动，推到一半突然蹿出去 —— 说明死区调小了，调大。
 *   现象：轻推摇杆车就猛地窜一下 —— 说明死区调大了，调小。
 *   出厂建议 0.18，实测地面（白色地胶）一般落在 0.12~0.26 */
#define MOTOR_DEADZONE           0.18f

/* 死区补偿生效后的最小输出（防止抖动） */
#define MOTOR_MIN_OUT            0.00f

/* 速度增益：把「归一化速度 0~1」映射到「占空比 0~1」
 *   现象：车太慢、爬不上坡 —— 调大
 *   现象：车一给油门就打滑/抬头 —— 调小
 *   注意：>1.0 会被限幅，没意义
 *
 * ⚠ 前进增益和横移增益必须相等，否则斜着走会系统性偏方向。
 *   这一点是 tools/sim/run_sim.py 在模拟器里跑出来的实测结论：
 *   FORWARD=0.85 / STRAFE=0.80 时，推 45° 斜走，实际是 43.26°，偏了 1.74°，
 *   等于走 2 米横移 6 厘米。把两者设成一样，偏差归零。
 *
 * 【真要补偿横移效率低，应该往大调，不是往小调】
 *   设 前进实测速度 vf、横移实测速度 vs（同样满舵、同样 2 米掐表），
 *   横移更慢（vs < vf）说明横移损耗大，要给它**更大**的增益才走得一样快：
 *       新横移增益 = 旧横移增益 × (vf / vs)
 *   例：vf=0.55 m/s，vs=0.48 m/s -> 0.85 × (0.55/0.48) = 0.97
 *   没做过实测之前，先保持两者相等（0.85），这是最不容易出问题的起点。 */
#define SPEED_GAIN_FORWARD       0.85f
#define SPEED_GAIN_STRAFE        0.85f   /* 先和 FORWARD 保持一致，实测后再按上式调 */
#define SPEED_GAIN_YAW           0.45f   /* 自转最容易打滑，给小一点 */

/* 加速度斜坡（防滑核心）：单位 = 归一化速度 / 秒
 *   现象：起步轮子原地空转、车抖 —— 调小 ACC_UP
 *   现象：起步肉、反应慢 —— 调大 ACC_UP
 *   现象：松摇杆车还滑行很远、冲过目标 —— 调大 ACC_DOWN
 *   打滑严重（坡道/地胶脏）时：ACC_UP 降到 1.5 */
#define CHASSIS_ACC_UP           3.0f
#define CHASSIS_ACC_DOWN         6.0f
#define CHASSIS_YAW_ACC_UP       6.0f
#define CHASSIS_YAW_ACC_DOWN     12.0f

/* 起步助力（kick）：为克服静摩擦，起步瞬间额外叠一点占空比，持续 KICK_MS 后线性衰减
 *   现象：车“咔”一下才动、起步一顿 —— 调大 KICK_DUTY
 *   现象：起步猛点头 —— 调小 */
#define MOTOR_KICK_DUTY          0.12f
#define MOTOR_KICK_MS            120

/* 高速时限制自转（防离心打滑）：|ω| 上限 = YAW_LIMIT_BASE - YAW_LIMIT_K * 线速度
 *   现象：边走边转时整车甩尾 —— 调大 YAW_LIMIT_K（限制更狠）
 *   现象：高速行进时想转却转不动 —— 调大 YAW_LIMIT_BASE */
#define YAW_LIMIT_BASE           0.90f
#define YAW_LIMIT_K              0.60f

/* 满油门时车体的实际速度（m/s）与自转速度（rad/s）。
 *   标定方法：地上贴胶带量 2m，全油门直行掐表；自转掐表转 3 圈。
 *   这两个数只影响「横移/自转混合时」的手感比例，不填准也能跑，
 *   但填准之后斜着走才不会莫名其妙画弧。 */
#define CHASSIS_V_MAX            0.60f
#define CHASSIS_W_MAX            4.00f

/* 松摇杆后的处理：0=自由滑行(coast) 1=刹车(brake)
 *   地胶上滑得很远，建议 1；若刹车导致车身点头/齿轮打齿，改 0 */
#define MOTOR_IDLE_BRAKE         1

/* 电压补偿：占空比 *= (VBAT_NOMINAL / Vbat)，限制在 VBAT_COMP_MAX 以内
 *   电量掉到 6.8V 时若不补偿，车会明显变慢。开着它，两局之间速度才一致 */
#define VBAT_COMP_ENABLE         1
#define VBAT_NOMINAL             7.40f
#define VBAT_COMP_MAX            1.25f
#define VBAT_LOW_WARN            6.60f    /* 低于此值蜂鸣器长鸣 + LED 快闪 */
#define VBAT_ADC_DIV             3.0f     /* 分压比：Vbat = 3 * Vadc（20k+10k） */

/* 左右轮不对称补偿（实测：空车直线跑 2m 偏了就调这里，往偏的反方向调）
 *   往左偏 -> 说明右边快 -> 把 RIGHT_TRIM 调小（如 0.95） */
#define MOTOR_LEFT_TRIM          1.00f
#define MOTOR_RIGHT_TRIM         1.00f

/* ---------------------------------------------------------------------------
 * 2. 闭环反馈（可选，需补买硬件）
 * -------------------------------------------------------------------------*/
/* 编码器：TT 马达配霍尔编码器（约 12 元/个，需拆马达后盖或用带编码器款）。
 *   打开后 motor.c 会用 PID 把轮速拉到目标值，防滑效果质变。
 *   没买硬件就保持 0 —— 代码会自动回退到纯开环，不会编译报错。 */
#define USE_ENCODER              0

/* IMU（MPU6050，约 8 元，I2C）。打开后自动模式下有航向保持（走直线不漂） */
#define USE_IMU                  0

#if USE_ENCODER
  #define ENC_PPR                (12.0f * 48.0f)  /* 霍尔 12 线 * 减速比 48（单沿计数） */
  #define PID_KP                 1.6f
  #define PID_KI                 0.35f
  #define PID_KD                 0.02f
  #define PID_I_LIMIT            0.45f
#endif

/* ---------------------------------------------------------------------------
 * 3. 舵机（角度单位：度，0~180）
 * -------------------------------------------------------------------------*/
/* 脉宽标定：MG995 与 SG90 的极限不一样。若舵机到极限位置“吱吱”抖，
 *   说明脉宽超行程，把 MIN/MAX 往中间收（每次收 30us） */
#define SERVO_PWM_MIN_US         500     /* 0  度 */
#define SERVO_PWM_MAX_US         2500    /* 180 度 */

/* 舵机运动速度限制（度/秒）：防止瞬间大电流把 5V 降压模块拉垮导致单片机复位
 *   现象：动舵机时单片机重启/PS2 掉线 —— 调小到 120 */
#define SERVO_SPEED_DPS          240.0f

/* 夹爪（SERVO_GRIP，SG90） */
/* ⚠⚠ 手册 V1.4 把方块从 40mm 改成 30mm（壁厚 7.5mm）。
   下面三个角度是**按 40mm 方块调的旧值，必须重新标定**：
     1) 先 SERVO_GRIP_CLOSED=95 让它夹到底，量两块防滑垫之间还剩多少 mm
     2) 目标：闭到底的净距 = 26~28mm（必须 < 30，否则夹不住）
     3) 从 CLOSED 往回退，找到刚好夹紧 30mm 方块、舵机不发烫的角度，
        写进 SERVO_GRIP_HOLD_CUBE / HOLD_CORE
   典型结果会是 84~88（比旧的 78 更靠 CLOSED），因为方块小了 10mm。 */
#define SERVO_GRIP_OPEN          30
#define SERVO_GRIP_HOLD_CUBE    86      /* 【待标定】夹 30mm 白块（能量单元） */
#define SERVO_GRIP_HOLD_CORE    84      /* 【待标定】夹 30mm 黄块（能量核心） */
#define SERVO_GRIP_CLOSED       95

/* 大臂（SERVO_ARM，MG995）
 *
 * ⚠ 方向必须实车标定，别信下面注释里的"高/低"：
 *   按这组数值的排列顺序（120 → 95 → 78 → 25 → 15），
 *   隐含假设是【角度越大 = 臂末端越低】。但舵机装反了就整体反过来
 *   （新值 = 180 − 旧值），那时注释里的"高/低"也全部要跟着反。
 *
 * ⚠ 特别注意 SERVO_ARM_STOW：它是**检录初始态**，直接决定初始高度超不超 210mm。
 *   原来的注释写成"最矮姿态"是错的——25 靠近 RACK(15) 那一端，按数值序列是偏高的。
 *   装好车先单独标定这一个姿态，量总高，把结果回填到 docs/09 的尺寸表。
 */
#define SERVO_ARM_STOW           25      /* 收起（检录初始态）⚠ 高度必须实测，原注释"最矮"有误 */
#define SERVO_ARM_GROUND         120     /* 下降到地面取块位（按数值序列是低位） */
#define SERVO_ARM_STEP           78      /* 台阶/焦点平台高度取块位 */
#define SERVO_ARM_RACK           15      /* 挑物资架黄块（按数值序列是最高位） */
#define SERVO_ARM_LIFT           95      /* 举起到储仓口高度 */

/* 前铲 / 喇叭口翻板（SERVO_SCOOP，SG90） */
#define SERVO_SCOOP_DOWN         150     /* 放铲（贴地） */
#define SERVO_SCOOP_TRAVEL       60      /* 行驶位（离地 15mm，防止蹭地） */
#define SERVO_SCOOP_UP           20      /* 收起（把块兜住往储仓送） */

/* 备用舵机（SERVO_EXTRA，挑杆/洞窟探杆） */
#define SERVO_EXTRA_IN           0
#define SERVO_EXTRA_OUT          180

/* ---------------------------------------------------------------------------
 * 4. 半自动宏的时间参数（单位：毫秒 / 归一化速度）
 *    【这些是盲走参数，必须实车标定】标定方法见 docs/调参指南.md
 * -------------------------------------------------------------------------*/
/* 宏1：地面方块自动收集（前进 → 对中触发 → 夹 → 抬 → 入仓 → 复位） */
#define AUTO_APPROACH_SPEED      0.35f   /* 慢速靠拢，快了会把块撞飞 */
#define AUTO_APPROACH_TIMEOUT    2500    /* 超时未触发限位则认为没对准，退出 */
#define AUTO_GRIP_SETTLE_MS      220     /* 夹紧后等舵机到位 */
#define AUTO_LIFT_SETTLE_MS      400
#define AUTO_STORE_SETTLE_MS     350

/* ⚠ 等「地面取块序列」跑完的硬上限。必须 **大于** POSE_PICK_GROUND 的实际耗时，
 * 否则每一次取块都会被判定成超时失败 —— 这个坑我踩过：
 * 序列实测约 3970ms，而这里原本写 3000，结果 100% 失败。
 * 现象：夹爪明明夹起来了，车却报失败并把块丢掉。
 * 改法：把车架空，串口看 [MACRO] 那条日志，序列跑完用了多少 ms，把这个值加到它的 1.3 倍。 */
#define AUTO_PICK_HARD_TIMEOUT   5000
#define AUTO_BACKUP_SPEED        0.30f
#define AUTO_BACKUP_MS           300

/* 宏2：一键靠边（横移贴墙，用于从启动区出发后快速贴住场地边框） */
#define AUTO_EDGE_SPEED          0.30f
#define AUTO_EDGE_TIMEOUT        1500

/* 宏3：挑物资架黄块（1号机/2号机通用：抬杆 → 前进 → 下压 → 后退） */
#define AUTO_RACK_LIFT_MS        500
#define AUTO_RACK_PUSH_SPEED     0.25f
#define AUTO_RACK_PUSH_MS        700
#define AUTO_RACK_PULL_SPEED     0.30f
#define AUTO_RACK_PULL_MS        600

/* 宏4：洞窟探杆取块 */
#define AUTO_CAVE_IN_SPEED       0.20f
#define AUTO_CAVE_IN_MS          900
#define AUTO_CAVE_OUT_SPEED      0.35f
#define AUTO_CAVE_OUT_MS         700

/* 限位开关去抖 */
#define LIMIT_DEBOUNCE_MS        30
/* 两个限位都闭合才算“对中”，允许的时间差上限 */
#define LIMIT_BOTH_WINDOW_MS     400

/* ---------------------------------------------------------------------------
 * 4b. 全自动模式（auto_full.c）—— 只在切到 MODE_FULL 时生效
 *
 * 【标定顺序：先标定 4 节的宏参数，再来标这一节】
 *   因为全自动只是"自动按下宏的键"，宏本身不准，全自动必然不准。
 *
 * 【怎么标】把车放到场地上，切到全自动，掐表看它走得多远：
 *   - 撞墙了          → AUTO_CRUISE_FWD_MS 调小
 *   - 一趟只扫了半场  → AUTO_CRUISE_FWD_MS 调大
 *   - 换道间距太大漏块→ AUTO_CRUISE_STRAFE_MS 调小
 *   - 换道太密原地蹭  → AUTO_CRUISE_STRAFE_MS 调大
 * -------------------------------------------------------------------------*/
/* 进入全自动后的预备时间：机构收起 + 让人走开。别设 0，人会来不及躲 */
#define AUTO_FULL_ARM_DELAY_MS   2000

/* 原地搜索（仅 USE_VISION=1 时用得到）：慢转找目标 */
#define AUTO_SEARCH_SPIN_SPEED   0.35f
#define AUTO_SEARCH_SPIN_MS      1200    /* 转这么久还没看到就转入巡游 */

/* 蛇形巡游：
 *   一次"纵向腿"走 AUTO_CRUISE_FWD_MS，一次"横移腿"走 AUTO_CRUISE_STRAFE_MS，
 *   交替进行；走满 LANES 个来回后自动掉头反向，形成往复覆盖。
 *   场地 2400mm，车宽约 160mm，速度 0.35 归一化 —— 下面这值是估算起点，
 *   **必须实车掐表改**。 */
#define AUTO_CRUISE_SPEED        0.35f
#define AUTO_CRUISE_FWD_MS       2600
#define AUTO_CRUISE_STRAFE_SPEED 0.30f
#define AUTO_CRUISE_STRAFE_MS    900
#define AUTO_CRUISE_LANES        4       /* 扫几条车道后掉头 */

/* 收尾：抓完退一步，防止块卡在铲口被拖出来 */
#define AUTO_RECOVER_BACK_SPEED  0.30f
#define AUTO_RECOVER_BACK_MS     400

/* 宏的硬超时（双保险，宏自己也有超时，这是防止整个全自动卡死）。
 * 必须大于整个 MACRO_GROUND 的最坏耗时：
 *   READY 900 + 靠拢 ≤2500 + PICK 5000 + 后退 300 ≈ 8700，所以取 12000。 */
#define AUTO_MACRO_HARD_TIMEOUT  12000

/* ---- 三道保险，建议别乱改 ---- */
/* 总时长上限：一局 5 分钟，留 60s 余量给操作手接管和回程 */
#define AUTO_TOTAL_LIMIT_MS      240000
/* 连续失败这么多次就放弃全自动，停车等人（防止在没块的地方反复空抓） */
#define AUTO_FAIL_LIMIT          3
/* 拿到这么多个就收工（储仓只有 3 格，别贪） */
#define AUTO_MAX_PICK            3

/* ---------------------------------------------------------------------------
 * 5. PS2 手柄
 * -------------------------------------------------------------------------*/
/* 摇杆中位死区（0~128）：手柄摇杆回中不一定精确是 128 */
#define PS2_STICK_DEADZONE       12
/* 手柄失联保护：连续这么多次没收到有效数据就停机（SPI 一次轮询 ~10ms） */
#define PS2_LOST_THRESHOLD       25
/* 手柄失联后的动作：0=刹车不动 1=缓慢停车 */
#define PS2_LOST_ACTION          0

/* 摇杆 -> 速度 的映射曲线：1.0=线性，>1=低速更细腻（推荐 1.4） */
#define PS2_STICK_EXPO           1.4f

/* 按键宏映射（PS2 按键位定义见 ps2.h） */
#define KEY_MACRO_GROUND         PSB_CIRCLE    /* ○ 键：地面自动收集 */
#define KEY_MACRO_RACK           PSB_TRIANGLE  /* △ 键：挑物资架黄块 */
#define KEY_MACRO_CAVE           PSB_SQUARE    /* □ 键：洞窟探杆取块 */
#define KEY_MACRO_EDGE           PSB_CROSS     /* × 键：一键贴边 */
#define KEY_SERVO_STOW           PSB_L1        /* L1：全机构收起（检录位） */
#define KEY_SERVO_SCOOP          PSB_R1        /* R1：前铲放下/抬起切换 */
#define KEY_MODE_TOGGLE          PSB_SELECT    /* SELECT：手动/半自动切换 */
#define KEY_EMERGENCY            PSB_START     /* START：急停（所有输出归零） */

/* ---------------------------------------------------------------------------
 * 6. 视觉（进阶功能，K210 / OpenMV，UART1 115200）
 * -------------------------------------------------------------------------*/
/* 没买视觉模块就保持 0。写成 #ifndef 是为了让编译脚本能临时打开它：
   python tools/build.py vision 会自动加 -DUSE_VISION=1，
   这样你可以不改这个文件就能编出"带视觉"的固件做验证。 */
#ifndef USE_VISION
#define USE_VISION               0
#endif
#if USE_VISION
  #define VISION_BAUD            115200
  #define VISION_TIMEOUT_MS      300    /* 超时未收到目标则忽略视觉 */
  /* 自动对位 P 控制：转向 = KP * 目标横向偏差 */
  #define VISION_TURN_KP         0.0022f
  #define VISION_TURN_MAX        0.40f
  #define VISION_X_DEADZONE      12     /* 画面像素，小于此偏差不纠 */
  #define VISION_APPROACH_SPEED  0.28f
  #define VISION_NEAR_AREA       9000   /* 色块像素面积大于此值认为已到位 */
#endif

/* ---------------------------------------------------------------------------
 * 7. 调度周期
 * -------------------------------------------------------------------------*/
#define CTRL_PERIOD_MS           5      /* 底盘控制周期 5ms（200Hz） */
#define SERVO_PERIOD_MS          10     /* 舵机插值周期 10ms */
#define LOG_PERIOD_MS            200    /* 串口打印周期 */

#endif /* __CONFIG_H */
