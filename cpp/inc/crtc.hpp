#ifndef CRTC_HPP
#define CRTC_HPP

/* ============================================================================
 * crtc.hpp —— C++ 版运动 / 机构 / 视觉 三合一核心
 *
 * 【为什么类都是 POD + init()，而不是写成正统构造函数】
 *   STM32 的启动文件是我自己写的 C 版本，它只做 .data/.bss 拷贝，**不调用
 *   .init_array**（C++ 全局对象的构造函数就挂在那儿）。所以如果你在这里
 *   写 `Chassis chassis;` 并指望构造函数跑，它不会跑 —— 成员全是垃圾值。
 *   统一改成 init() 显式初始化，静态对象零初始化进 .bss，稳。
 *   这也是嵌入式 C++ 的正统写法，不是偷懒。
 *
 * 【编译开关（build_cpp.py 已经加好，这里只是说明）】
 *   -fno-exceptions -fno-rtti -fno-threadsafe-statics
 *   关掉异常和 RTTI，否则会链进 libstdc++ 的一大坨，64KB Flash 顶不住。
 *
 * 【命名约定】
 *   所有类都在 namespace crtc 下；硬件访问全走 namespace crtc::hal，
 *   真机上电版本用 hal_mcu.cpp，模拟器版本用 hal_sim.cpp —— 同一份逻辑，
 *   换个 .cpp 就能在 PC 上跑数值验证。
 * ==========================================================================*/

#include <stdint.h>
#include <stddef.h>

namespace crtc {

/* ---------------------------------------------------------------------------
 * 0. 参数表（默认值 = firmware/Core/Inc/config.h 里的定稿值）
 *    改这里等价于改 config.h，但 C++ 版可以在运行时换参数做扫描对比。
 * -------------------------------------------------------------------------*/
struct Params {
    /* 车体几何（必须和 mechanical/gen_parts.py 的 PARAMS 一致） */
    float halfWheelbase;      /* 半轴距 m（WHEELBASE 180mm -> 0.090） */
    float halfTrack;          /* 半轮距 m（TRACK 136mm -> 0.068）     */
    float wheelRadius;        /* 轮半径 m（30mm）                     */
    float vMax;               /* 满油门线速度 m/s（掐表测 2m 算出来） */
    float wMax;               /* 满油门角速度 rad/s                   */

    /* 速度增益：归一化速度 -> 占空比
     * ⚠ gainForward 必须等于 gainStrafe，否则斜走会系统性偏方向 */
    float gainForward;
    float gainStrafe;
    float gainYaw;            /* A 车（麦轮）自转增益 */
    float gainYawDiff;        /* B 车（橡胶轮）自转增益，差速转向更弱所以比 gainYaw 大 */

    /* 加速度斜坡（防滑第一道防线），单位 归一化/秒 */
    float accUp, accDown;
    float yawAccUp, yawAccDown;

    /* 高速限自转：|w| <= yawLimitBase - yawLimitK * 线速度 */
    float yawLimitBase, yawLimitK;

    /* 电机输出 */
    float    deadzone;        /* 死区补偿 0~1，实测一般 0.12~0.26 */
    float    kickDuty;        /* 起步助力幅值 */
    uint16_t kickMs;          /* 起步助力持续时间 */
    int      dirSign[4];      /* 轮转向修正 ±1，装反了改这里别改线 */
    float    trim[4];         /* 左右轮一致性微调 */
    uint8_t  idleBrake;       /* 松摇杆：1=刹车 0=滑行 */

    /* 电压补偿 */
    uint8_t vbatComp;
    float   vbatNominal;
    float   vbatCompMax;

    /* 舵机 */
    float    servoDps;        /* 限速 度/秒，防 5V 塌陷 */
    uint16_t servoMinUs;      /* 0 度脉宽 */
    uint16_t servoMaxUs;      /* 180 度脉宽 */
};

extern const Params kDefaultParams;

/* TIM4 计数周期：PSC=0, ARR=7199 -> 10kHz，一个周期 7200 个计数 */
const uint16_t kPwmPeriod = 7200U;

/* ---------------------------------------------------------------------------
 * 0.1 运动学内核选择：麦轮 / 差速（两台车各一份，见 chassis.cpp 的 kinematics）
 *
 * ⚠ 这里为什么用工程惯例的 K 前缀 + 纯数值比较，而不是 enum：
 *   最早写成 `enum ChassisKind { kMecanum=0, kDiff=1 };` 加一个 `const` 常量，
 *   然后 `if (kChassisKind == kDiff)`。在 -Os 下编译器把 `enum == enum` 折叠成
 *   了一次**有符号比较**，A 车（值 0）恰好走对，B 车那份固件却仍然跑麦轮公式 ——
 *   编译不报错、符号表里两个内核都在、只有跑数值才看得出来。
 *   现在改成"宏数值直接比"，优化器没有推理空间。
 * -------------------------------------------------------------------------*/
#ifndef CRTC_CHASSIS_KIND
  #define CRTC_CHASSIS_KIND 0    /* 0 = A 车麦轮（默认）；1 = B 车橡胶轮差速 */
#endif

#define CRTC_IS_DIFF()   (CRTC_CHASSIS_KIND == 1)

/* ---------------------------------------------------------------------------
 * 1. 硬件抽象层（真机 / 模拟器两套实现）
 * -------------------------------------------------------------------------*/
namespace hal {
    void     motorCompare(int idx, uint16_t cmp);   /* 写 TIM CCR */
    void     motorDir(int idx, bool high);          /* 方向电平 */
    void     servoPulse(int ch, uint16_t us);       /* 舵机脉宽（微秒） */
    uint32_t millis(void);
    int      uartRead(void);                        /* -1 = 没有新字节 */
    bool     limitHit(int idx);                     /* 限位开关，true=压住 */
    float    vbat(void);                            /* 电池电压 V */
    int      print(const char *fmt, ...);           /* 串口打印 */
}

/* ---------------------------------------------------------------------------
 * 2. 电机输出层：把 -1..1 的目标速度变成 占空比 + 方向电平
 *    死区补偿 -> 起步助力 -> 电压补偿 -> 拆 PWM/DIR
 * -------------------------------------------------------------------------*/
class MotorMixer {
public:
    void init(const Params &p);
    void setVbat(float vbat);
    void set(int idx, float speed, uint32_t nowMs);
    void outputAll(const float s[4], uint32_t nowMs);
    void brakeAll(void);
    void coastAll(void);

    float duty(int idx) const { return duty_[idx]; }
    const Params *params(void) const { return p_; }

private:
    const Params *p_;
    float         duty_[4];
    float         vcomp_;
    uint32_t      kickStart_[4];
    uint8_t       wasIdle_[4];
};

/* ---------------------------------------------------------------------------
 * 3. 底盘：斜坡 -> 限自转 -> 运动学（麦轮逆解 / 差速）-> 整体等比归一化 -> 输出
 *
 *    ⚠ 运动学内核由编译期常量 kChassisKind 决定（见上面 0.1 节）：
 *       kMecanum = A 车（麦轮，全向，vy 有效）
 *       kDiff    = B 车（橡胶轮，差速，vy 在物理上不存在，会被丢弃）
 *    外圈三步（斜坡/限自转/归一化）两台车完全相同，只有第三步不同，
 *    所以这里只切换那一个函数，其余代码零改动。
 * -------------------------------------------------------------------------*/
class Chassis {
public:
    void init(void);
    void init(const Params &p);

    void setTarget(float vx, float vy, float wz);   /* 归一化 -1..1 */
    void update(float dtSec, uint32_t nowMs);       /* 控制周期调用 */
    void stop(void);                                /* 目标归零，靠斜坡滑停 */
    void emergencyStop(void);                       /* 立刻刹车 */

    void setVbat(float vbat) { mixer_.setVbat(vbat); }

    const float *wheels(void) const { return w_; }   /* 四轮解算值 -1..1 */
    const float *cur(void)    const { return cur_; } /* 斜坡后的 vx,vy,wz */
    float yawGeom(void) const;                       /* (a+b)*wmax/vmax */
    float turnK(void) const;                         /* 差速转弯系数 半轮距/轮半径（无单位） */
    MotorMixer &mixer(void) { return mixer_; }

private:
    void ramp(float dt);
    void limitYaw(void);
    void kinematics(void);      /* 按 kChassisKind 分派 */
    void kinematicsMecanum(void);
    void kinematicsDiff(void);
    void normalize(void);

    const Params *p_;
    float tgt_[3], cur_[3], w_[4];
    uint8_t emergency_;
    MotorMixer mixer_;
};

/* ---------------------------------------------------------------------------
 * 4. 单个舵机：限速缓动（防瞬间大电流把 5V 拉塌）
 * -------------------------------------------------------------------------*/
class Servo {
public:
    void init(const Params &p, int channel, float startDeg);
    void setTarget(float deg);
    void update(float dtSec);            /* 按 servoDps 逼近目标 */
    void jump(float deg);                /* 直接到位，不做缓动 */

    float current(void) const { return cur_; }
    float target(void)  const { return tgt_; }
    bool  isMoving(void) const;
    int   channel(void) const { return ch_; }

private:
    void apply(void);                    /* 角度 -> 脉宽 -> hal */

    const Params *p_;
    int   ch_;
    float cur_, tgt_;
};

/* ---------------------------------------------------------------------------
 * 5. 机构序列器：一次只动一个舵机（串行），防 5V 塌陷导致单片机复位
 * -------------------------------------------------------------------------*/
enum Pose {
    POSE_NONE        = 0,
    POSE_STOW        = 1,   /* 收起（检录初始态）⚠ 高度必须实车量，别信"最矮"二字 */
    POSE_READY       = 2,   /* 待命：前铲贴地 */
    POSE_PICK_GROUND = 3,   /* 地面取块（核心动作） */
    POSE_PICK_STEP   = 4,   /* 台阶 / 焦点平台取块 */
    POSE_STORE       = 5,   /* 入仓 */
    POSE_RACK        = 6,   /* 挑物资架黄块 */
    POSE_CAVE        = 7,   /* 洞窟探杆取块 */
    POSE_SCOOP       = 8,   /* 单纯放铲 */
    POSE_COUNT
};

/* 舵机编号，要和 board.h 的 SERVO_* 对应 */
enum ServoId { kGrip = 0, kArm = 1, kScoop = 2, kExtra = 3, kServoCount = 4 };

class ArmSequencer {
public:
    /* 一步动作：动哪个舵机、到多少度、到位后再等多少毫秒。
     * 放在 public 是因为动作脚本数组定义在 arm.cpp 里，
     * 私有嵌套类型在类外访问不到。 */
    struct Step { uint8_t servo; float angle; uint16_t waitMs; };

    void init(Servo *servos, const Params &p);
    void start(Pose pose, uint32_t nowMs);
    void update(uint32_t nowMs);        /* 1~10ms 调一次 */
    bool busy(void) const { return busy_ != 0; }
    Pose current(void) const { return cur_; }

    void grip(float deg);
    void scoopDown(bool down);

private:
    static const Step *sequence(Pose pose, int &len);

    Servo       *sv_;
    const Params *p_;
    Pose         cur_;
    uint8_t      busy_;
    uint8_t      idx_;
    uint32_t     tMark_;
};

/* ---------------------------------------------------------------------------
 * 6. 视觉链路：14 字节定长帧状态机（AA 55 cls x y w h area chk）
 * -------------------------------------------------------------------------*/
struct VisionFrame {
    uint8_t  cls;      /* 0=没目标 1=白块 2=黄块 */
    int16_t  x, y;     /* 目标中心像素坐标（画面中心为 0） */
    uint16_t w, h;     /* 像素宽高 */
    uint16_t area;     /* 像素面积，用来判断远近 */
    uint32_t lastMs;   /* 最后一帧时间 */
    uint8_t  fresh;    /* 本周期收到新帧 */
};

class VisionLink {
public:
    void init(void);
    void feedByte(uint8_t b, uint32_t nowMs);
    void update(uint32_t nowMs);
    bool valid(uint32_t nowMs, uint16_t timeoutMs) const;
    const VisionFrame &frame(void) const { return f_; }

    /* 自动对位：给定期望的横向误差为 0，算出一个转向指令 */
    float turnCommand(uint16_t deadzonePx, float kp, float maxOut) const;

    static const int kFrameLen = 14;

private:
    VisionFrame f_;
    uint8_t     buf_[14];
    uint8_t     len_;
    uint8_t     state_;
};

} /* namespace crtc */

#endif /* CRTC_HPP */
