# firmware_v2 —— 实车烧录的底盘固件（最终版）

> 本目录是 **2026-10-19 真正烧进车上那颗 STM32F103C8T6 的那一版**。
> 仓库里另有 `firmware/`（本队自研的 HAL shim 版，含舵机、机械臂、半自动宏），
> 两者**互不相干、产物分开**，关系见第七节。

## 一、它管什么

只管一条链：**读 PS2 手柄 → 算四个轮子各转多快 → 输出给两块 DRV8833**。

不含舵机、不含机械臂、不含储仓自动收集，也不含视觉。

| 项 | 值 |
|---|---|
| 芯片 | STM32F103C8T6（64KB Flash / 20KB RAM） |
| 时钟 | 内部 HSI 经 PLL 到 64MHz，**不依赖外部晶振** |
| 调度 | 10ms 主循环；TIM4 输出 10kHz PWM |
| PS2 | GPIO 位敲 SPI（不是硬件 SPI），125kHz，低位先发 |
| 依赖 | 无 HAL、无 CubeMX、无标准外设库；只依赖 C 标准库的 `memset` |

源码 6 个文件：

| 文件 | 作用 |
|---|---|
| `src/main.c` | 10ms 主循环、状态 LED、调试变量 |
| `src/board.c` | 时钟、GPIO、TIM4 PWM、SysTick、看门狗 |
| `src/ps2_bus.c` | PS2 位敲收发、模式协商 |
| `src/robot_control.c` | 手柄解析、麦轮混合、启停与急停逻辑 |
| `src/driver_encode.c` | DRV8833 双输入真值表换算（反向时占空比取反） |
| `src/startup.c` | 向量表、复位入口（C 语言写的，不是汇编 `.s`） |

## 二、引脚（实车逐根核对过）

LF=左前，RF=右前，LB=左后，RB=右后，均按**车辆前进方向**定义。

| STM32 引脚 | 接什么 | 方向 | 说明 |
|---|---|---|---|
| PB0 | PS2 接收器 CS / ATT / SEL | 输出 | 软片选，普通 GPIO |
| PA5 | PS2 CLK | 输出 | 125kHz |
| PA6 | PS2 DAT / DO | **上拉输入** | 接收器 → STM32。这一脚是**输入**，配了内部上拉 |
| PA7 | PS2 CMD / DI | 输出 | STM32 → 接收器 |
| PB6 / TIM4_CH1 | 左前 DRV8833 IN1 | PWM | |
| PB12 | 左前 DRV8833 IN2 | 输出 | |
| PB7 / TIM4_CH2 | 右前 DRV8833 IN1 | PWM | |
| PB13 | 右前 DRV8833 IN2 | 输出 | |
| PB8 / TIM4_CH3 | 左后 DRV8833 IN1 | PWM | |
| PB14 | 左后 DRV8833 IN2 | 输出 | |
| PB9 / TIM4_CH4 | 右后 DRV8833 IN1 | PWM | |
| PB15 | 右后 DRV8833 IN2 | 输出 | |
| PC13 | 板载 LED | 输出 | **低电平点亮** |

三个容易接错的地方，都是我们真接错过的：

1. **PS2 的 DI / DO 要对调着接。** STM32 的 `CMD`（输出）接接收器的 `DI`，
   `DAT`（输入）接接收器的 `DO`。按"单片机视角"直连就反了。我们查了三小时才发现。
2. **PS2 接收器供 3.3V，不是 5V。** 接收器不能直插单片机排针，要过转接板。
3. **SWD 四根线不能整排直插。** 板顶是 `3V3/SWIO/SWCLK/GND`，ST-LINK 右列是
   `SWCLK/SWDIO/GND/3.3V`，顺序不同，直插等于把 3.3V 怼到 GND 上。必须四根独立线对丝印插。

DRV8833 是**双输入真值表**：`00`=滑行、`10`=正转、`01`=反转、`11`=刹车。
所以反转不是"改方向脚"，而是**把 PWM 占空比取反**（`compare = period - active`），
靠低电平那一段驱动。`driver_encode.c` 做的就是这件事。

## 三、相对原始包的 6 处改动

这份代码的骨架来自一份现成的裸机工程（原始压缩包 `STM32F103C8_PS2_Firmware`）。
**13 个文件里 9 个字节未动，改了 4 个文件、6 处**，每一处都是烧进车以后实测出来的：

| # | 文件 | 改动 | 为什么 |
|---|---|---|---|
| 1 | `board_config.h` | `BOARD_WATCHDOG_ENABLE` `1` → `0` | 这块 F103C8 的 LSI 不驱动 IWDG：写 `IWDG_PR/RLR` 后 `IWDG_SR` 的 PVU/RVU 永不清零，`board_init()` 卡在等待循环 → 超时返回 false → `main` 死在死循环。表现是**上电后完全没反应**。 |
| 2 | `board_config.h` | `MOTOR_LB_SIGN` / `MOTOR_RB_SIGN` `1` → `-1` | 推前进时前两轮正常、**后两轮反转**。寄存器读回 `TIM4_CCR1..4` 与 `PB12~15` 四路完全同值，说明 MCU 输出没问题、差异在马达线序。用软件取反补偿，等效于把后轮两根线对调。 |
| 3 | `ps2_bus.c` | 采样点从"上升沿后再等半位"改到**上升沿后立即采** | 原来每位读到的是上一位（收到 `FF 39 2D…`，标准应为 `FF 73 5A…`，每个字节正好 `>>1`）。频率没错，是**相位**错了。周期不变，仍 125kHz。 |
| 4 | `robot_control.c` | START 改成**按下沿即生效** | 原逻辑要求"摇杆回中 + 不按 L1 连续 5 帧"才接受 START。实际后果：手上扶着摇杆或按着 L1 时按 START 被完全丢弃，**急停锁存永远解不开**——表现是"怎么按都没反应"，但代码认为自己一切正常。 |
| 5 | `robot_control.c` | 删除"必须一直按住 L1 才出力" | L1 与左摇杆单手没法同时稳住。现在按 START 启用后摇杆直接生效。**安全没有降低**：未按 START 摇杆恒无效、SELECT 急停锁存、100ms 收不到合法帧立即停车，三道都还在。 |
| 6 | `robot_control.h` | `ROBOT_NORMAL_LIMIT` `350→1000`、`ROBOT_SLOW_LIMIT` `180→450` | 原值 35% 占空比，7.4V 上只有约 2.6V，TT 马达带麦轮几乎带不动整车，表现是"推杆了但车不走"。先放开到 100% 把车跑起来，手感之后再收敛。 |

改动全部写在代码注释里，带日期和证据。`git log` 也能对上。

## 四、怎么编、怎么烧、怎么验

用 CubeIDE 自带的 `arm-none-eabi-gcc`，命令行，不进图形界面：

```bash
# 编译（产物在 build/firmware.bin）
python tools/build_v2.py

# 编译 + 逐项核对产物（向量表、入口地址、大小）
python tools/build_v2.py --verify

# 烧录（走 OpenOCD；这块 ST-LINK 是克隆版，CubeProgrammer 握不上手）
python tools/flash.py v2

# 读回内存，看 PS2 帧计数 / 复位原因
python tools/verify_v2.py
python tools/diag_v2.py
```

> ⚠ 这块 ST-LINK 是**克隆版 V2、固件 `V2J29S7`**，`STM32_Programmer_CLI` 一律报
> `DEV_CONNECT_ERR`，**不是线接错了**。要走 OpenOCD 且必须用 `interface/stlink-dap.cfg`
> （`stlink.cfg` 会报 `Infinite eval recursion`）。接线细节见 `docs/硬件接线与焊接指南.md`，
> 参数与调车细节见 `docs/调参指南.md`。

目录里另外保留了 `STM32_PS2_Mecanum.uvprojx` + `.sct`，给装 Keil 的电脑用 AC6 编译；
两条路产物等价，仓库里默认走脚本这条。

## 五、怎么操控

1. 手柄与接收器上电配对，摇杆回中、松开 L1 和方向键。
2. **按一次 START** 启用（上电时一直按着 START 不算，要松手后再按）。
3. 左摇杆前后=前后，左右=横移；右摇杆横轴=转向（左为逆时针）；方向键也能平移。
4. **按住 L2** 切慢速（45% 上限）。
5. **SELECT** 锁定停车，锁存后必须重新按 START 才能恢复。
6. 超过 **100ms** 收不到合法 PS2 帧 → 输出归零并自动取消启用。

输出归零是**滑行停车**，车还有惯性；SELECT 是软件停车，不等于硬件断电。

**状态 LED（PC13，低电平亮）**：

| 现象 | 含义 |
|---|---|
| 常亮 | 已启用，可以开 |
| 500ms 翻转 | 连上了，但还没按 START |
| **250ms 翻转（快闪）** | **没收到合法帧**——查 PA6/PA7 有没有接反 |
| 100ms 翻转 | 急停锁存 |

> 绿灯亮只说明"手柄 ⇢ 接收器"这一段通了；"接收器 ⇢ STM32"是另一段，要另查。
> 判据在 `ps2.raw[9]`：**全 `FF` = 对面根本没驱动数据线**；正常头是 `FF 73 5A`。

## 六、验证到什么程度，没验证什么

**已经验证（有数据）**：

- PS2 连续收帧 **49512 帧 / 8 分 15 秒，无一次复位**，帧头稳定 `FF 73 5A`
- 推杆时读回 `TIM4_CCR1..4` 与 `GPIOB->ODR`，四路信号一致（这也是判定"后轮反转在马达侧"的依据）
- 烧进去的 bin 逐字节核对过极性表：偏移 `0xB64` = `01 01 FF FF`，与前/后轮符号一致
- 向量表 236 字节 = 59 项 × 4；栈顶 `0x20005000`；复位入口 `0x08000A89`（thumb 位正确）
- 6 项独立校验全部通过（`tools/build_v2.py --verify` 可复现）

**没有验证（如实说）**：

| 项 | 原因 |
|---|---|
| **脱机（电池）供电** | 车载降压模块用坏两个、还有一个没找到，替代件 1~2 天到。现在 STM32 由 ST-LINK/USB 供电 |
| **落地行驶** | 拖着 USB 线，车一落地开动就扯断线，所有测试只能把车架起来转轮 |
| 真实车速与占空比曲线 | 需要落地跑才能测；现在的死区、100% 上限都还没收敛 |
| 长时间连续跑 | 只跑过 8 分 15 秒 |
| 无线断连行为 | 有些接收器失联后仍回传旧帧，100ms 超时识别不出来。需要单独试"关手柄"能否真停车 |

**这两条限制（无脱机、只悬空）是供电件缺料造成的，不是控制逻辑没跑通。**
补件到货后按顺序补：电池 + 车载降压 → 脱机空载 → 落地低速 → 落地全速。

## 七、和 `firmware/` 是什么关系

两个**互不相干**的工程，产物分开放，谁也不会覆盖谁：

| | `firmware/` | `firmware_v2/`（本目录） |
|---|---|---|
| 定位 | 本队自研主固件 | **实车烧录版（当前生效）** |
| 底层 | 自己写的 HAL shim（寄存器实现官方函数名） | 直接用寄存器，无 HAL |
| 内容 | 底盘 + PS2 + 舵机 + 机械臂 + 半自动宏 + 视觉串口 | **只有底盘 + PS2** |
| 编译 | `tools/build.py` | `tools/build_v2.py` |
| 烧录 | `tools/flash.py <目标名>` | `tools/flash.py v2` |

`firmware/` 不是废弃代码——它是舵机、机械臂、储仓那套功能的唯一实现，
中期之后要接着往下做就回到它。**但中期考核交的这一版车，跑的是 `firmware_v2`。**

## 八、参数速查

| 参数 | 位置 | 现值 | 含义 |
|---|---|---|---|
| `MOTOR_LF/RF/LB/RB_SIGN` | `board_config.h` | `1 / 1 / -1 / -1` | 单轮极性。**重新焊过马达线就改回 1**，否则后轮又反 |
| `PS2_HALF_BIT_US` | `board_config.h` | `4` | PS2 半位 4µs ≈ 125kHz |
| `BOARD_WATCHDOG_ENABLE` | `board_config.h` | `0` | 本芯片 IWDG 不可用，见第三节第 1 条 |
| `BOARD_MOTOR_PERIOD` | `board_config.h` | `6400` | TIM4 64MHz/6400 = 10kHz |
| `ROBOT_NORMAL_LIMIT` | `robot_control.h` | `1000` | 正常上限，千分比 |
| `ROBOT_SLOW_LIMIT` | `robot_control.h` | `450` | 慢速上限（L2） |
| `ROBOT_SLEW_PER_SECOND` | `robot_control.h` | `1000` | 占空比变化速率，0→100% 约 1s |
| `ROBOT_AXIS_DEADZONE` | `robot_control.h` | `12` | 摇杆回中死区 |
| `ROBOT_LINK_TIMEOUT_MS` | `robot_control.h` | `100` | 失联判据 |

调试时用 Keil Watch 或 `tools/verify_v2.py` 看这几个变量：
`g_robot`（启用/急停状态）、`g_debug`（四轮目标输出）、
`g_ps2_last_bytes` / `g_ps2_last_length`（最近一帧原始数据）。
