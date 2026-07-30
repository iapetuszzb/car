# MSPM0G3507 两轮智能小车

本工程基于 **TI MSPM0G3507（LQFP-64 / PM 封装）**，用于两轮差速小车的八路红外巡线、锐角处理、编码器里程计、MPU6500 姿态测量，以及蓝牙闭环转向和定距离直行。

当前默认运行配置以 `main` 分支和 `mspm0-modules.syscfg` 为准。K210/K230 视觉接收、步进云台、舵机扫描和 HC-SR04 测距代码仍保留，但默认关闭。

## 当前功能

- 八路红外巡线，黑线为高电平，白底为低电平。
- 位置式巡线 PID，并根据偏差大小限制弯道基础速度。
- 最外侧探头见黑后进入全白时，记忆方向并原地急转；重新见黑后恢复普通巡线。
- 双路直流电机 PWM 正反转控制，支持启动增扭和零速卡滞补偿。
- 双编码器 10 ms 测速、累计计数和一维行驶里程显示。
- MPU6500 上电静止标定、Mahony 姿态解算、Yaw 积分和 `1.5 dps` Z 轴角速度死区。
- 蓝牙命令启动/停止巡线、闭环转角、定距离直行和状态查询。
- 闭环转角使用 MPU6500 Yaw 角与角速度反馈，并叠加编码器速度 PI。
- 定距离直行使用编码器距离、速度 PI、加减速和 Yaw 航向修正。
- OLED 显示 Yaw、里程、左右编码器累计值和最近收到的蓝牙 ASCII。
- 板载 USB 调试串口每 100 ms 输出 `roll,pitch,yaw` CSV 数据。

## 模块状态

以下三个开关位于 `main.h`：

```c
#define APP_ENABLE_STEPPER_GIMBAL 0
#define APP_ENABLE_SERVO_SWEEP    0
#define APP_ENABLE_VISION_UART    0
```

HC-SR04 开关位于 `main.c`：

```c
#define HCSR04_MEASUREMENT_ENABLED 0
```

| 模块 | 状态 | 当前配置 |
| --- | --- | --- |
| 八路红外巡线 | 启用 | 普通 PID、弯道降速、锐角原地转向 |
| 电机 PWM | 启用 | 双 H 桥四路 PWM，计数周期 `40000` |
| 编码器 | 启用 | A 相下降沿中断，B 相判断方向 |
| 蓝牙串口 | 启用 | 硬件 UART1，`PA17/PA18`，`9600 8N1` |
| USB 调试串口 | 启用 | 硬件 UART0，`PA10/PA11`，`115200 8N1` |
| MPU6500 | 启用 | I2C1，400 kHz，主循环每 10 ms 读取一次 |
| OLED | 启用 | 软件 I2C，100 ms 刷新，显示旋转 180 度 |
| HC-SR04 | 关闭 | `PB10/PB11` 代码保留，当前不初始化 |
| K210/K230 视觉 UART | 关闭 | 与当前蓝牙 UART 存在资源冲突 |
| 步进云台 | 关闭 | 与 OLED、蓝牙存在引脚冲突 |
| 舵机扫描 | 关闭 | `PB26` 不输出舵机脉冲 |

## 引脚分配

下面的表格由 `mspm0-modules.syscfg`、生成的 `Debug/ti_msp_dl_config.h` 和实际驱动代码交叉核对得到。

### 电机驱动

每个电机使用两路 PWM 控制 H 桥。上层约定正值为车辆前进方向。

| 功能 | MSPM0 引脚 | 外设通道 | 车轮 |
| --- | --- | --- | --- |
| Motor A PWM0 | `PB4` | `TIMA1_CCP0` | 右轮 |
| Motor A PWM1 | `PB1` | `TIMA1_CCP1` | 右轮 |
| Motor B PWM0 | `PA28` | `TIMG7_CCP0` | 左轮 |
| Motor B PWM1 | `PA31` | `TIMG7_CCP1` | 左轮 |

当前电机配置：

```text
Motor A = 右轮
Motor B = 左轮
MOTOR_A_DIRECTION_SIGN = +1
MOTOR_B_DIRECTION_SIGN = +1
Motor B PWM 补偿 = 90%
PWM 硬件周期 = 40000
软件最终输出上限 = +/-32000
```

`MOTOR_A_DIRECTION_SIGN` 和 `MOTOR_B_DIRECTION_SIGN` 用于适配电机接线方向。若更换电机或接线后前进方向相反，应修改对应符号，不要在多层控制逻辑中同时反号。

### 编码器

| 功能 | MSPM0 引脚 | 配置 | 对应车轮 |
| --- | --- | --- | --- |
| Encoder A 相位 A | `PA12` | 下降沿中断、内部下拉 | 右轮 |
| Encoder A 相位 B | `PA13` | 方向判断、内部下拉 | 右轮 |
| Encoder B 相位 A | `PB20` | 下降沿中断、内部下拉 | 左轮 |
| Encoder B 相位 B | `PB13` | 方向判断、内部下拉 | 左轮 |

当前换算参数统一定义在 `Drivers/Encoder/encoder.h`：

```text
电机编码器线数        = 13
减速比                = 30:1
当前解码倍率          = 1（只统计 A 相下降沿）
每个车轮一圈计数      = 13 x 30 x 1 = 390
轮径                  = 65 mm
轮周长                = 204.2035 mm
每个计数对应距离      = 约 0.5236 mm
```

OLED 中的 `odo` 是左右轮绝对行程的平均值，适合显示总行驶距离；它不是带方向的二维 `x/y/yaw` 里程计。蓝牙定距离直行使用同一套共享编码器常量，不需要在 `uart.c` 重复修改参数。

### 八路红外巡线

从车辆前进方向观察，探头由左到右为 `line0` 到 `line7`：

| 探头 | 位置 | MSPM0 引脚 |
| --- | --- | --- |
| `line0` | 最左侧 | `PB16` |
| `line1` | 左侧 2 | `PB0` |
| `line2` | 左侧 3 | `PB6` |
| `line3` | 左中 | `PB7` |
| `line4` | 右中 | `PB8` |
| `line5` | 右侧 3 | `PB15` |
| `line6` | 右侧 2 | `PB17` |
| `line7` | 最右侧 | `PB12` |

电平定义：

```text
黑线 = 1（高电平）
白底 = 0（低电平）
八路全 0 = 丢线/全白
```

### 串口、传感器和人机交互

| 功能 | MSPM0 引脚 | 外设/说明 |
| --- | --- | --- |
| 蓝牙 TX | `PA17` | 芯片 UART1 TX，接蓝牙 RX |
| 蓝牙 RX | `PA18` | 芯片 UART1 RX，接蓝牙 TX，内部上拉 |
| USB 调试 TX | `PA10` | 芯片 UART0 TX，板载 backchannel |
| USB 调试 RX | `PA11` | 芯片 UART0 RX，当前不解析命令 |
| MPU6500 SCL | `PB2` | I2C1 SCL，400 kHz |
| MPU6500 SDA | `PB3` | I2C1 SDA，400 kHz |
| MPU6500 INT | `PB9` | 已配置为上升沿输入；当前姿态读取采用 10 ms 轮询 |
| OLED SCL | `PB19` | 软件 I2C |
| OLED SDA | `PA25` | 软件 I2C |
| 启动巡线按钮 | `PB21` | 内部上拉，按下为低电平 |
| 普通状态 LED | `PA0` | 状态输出 |
| 巡线状态 LED | `PB22` | 启动巡线时翻转 |

SysConfig 中名为 `UART_0` 的实例实际映射到芯片 **UART1**，用于蓝牙；名为 `UART_USB` 的实例实际映射到芯片 **UART0**，用于板载 USB 调试。这两个名字容易混淆，修改引脚时应以 `peripheral.$assign` 和生成头文件为准。

蓝牙模块必须使用 `3.3 V TTL UART`，TX/RX 交叉连接并与 MSPM0 共地。不要将 RS-232 电平直接接入 MCU。

## 关闭模块的预留引脚与冲突

以下引脚只存在于保留代码中，当前不会初始化。重新启用模块前必须先解决冲突，不能只把功能宏改为 `1`。

| 保留模块 | 原代码引脚 | 当前冲突 |
| --- | --- | --- |
| K210/K230 视觉 UART | RX=`PA9`，TX=`PA17` | `PA17` 已用于蓝牙 TX，且两者都使用 UART1 |
| 步进云台 A 轴 | STEP=`PA26`，DIR=`PB19`，EN=`PB18` | `PB19` 已用于 OLED SCL |
| 步进云台 B 轴 | STEP=`PB24`，DIR=`PA18`，EN=`PA27` | `PA18` 已用于蓝牙 RX |
| 云台电压 ADC 预留 | `PA22` | 当前未采样 |
| 舵机扫描 | `PB26` | 当前空闲，功能关闭 |
| HC-SR04 | TRIG=`PB10`，ECHO=`PB11` | 当前空闲，功能关闭 |

如需恢复视觉或云台，建议先在 SysConfig 中重新分配 UART/GPIO，再同步修改驱动宏并重新生成 `ti_msp_dl_config.*`。

## 巡线控制参数

默认巡线参数位于 `main.c`：

```c
P = 2500.0f;
I = 0.0f;
D = 800.0f;
basespeed = 10.0f;
TargetLine = 4.5f;
```

当前 PWM 参数位于 `Control/PID/pid.c`：

```c
#define MAX_DUTY 32000
#define BASE_SPEED_TO_PWM 1500
#define MIN_RUN_PWM 12000
#define START_BOOST_PWM 18000
#define START_BOOST_TICKS 40
#define LINE_STALL_BOOST_PWM 6000
#define LINE_TURN_PWM_LIMIT 18000
#define LINE_CURVE_BASE_PWM 13000
#define LINE_EXTREME_BASE_PWM 10500
```

`basespeed=10` 时，正常直线基础 PWM 为：

```text
10 x 1500 = 15000
```

按键启动巡线时，目标速度从 `basespeed x 0.75` 开始，每 10 ms 增加 `0.04`，约 `0.625 s` 后达到完整的 `basespeed=10`。启动后的前 40 个控制周期将基础 PWM 保持在至少 `18000`；如果某个轮子的编码器在当前 10 ms 周期内完全无计数，则只对该轮临时增加 `6000` PWM 以克服静摩擦。

锐角处理流程：

1. 普通巡线时记录最后一次仅由 `line0` 或 `line7` 指示的外侧方向。
2. 随后八路全白时，停止普通巡线 PID，并按记录方向原地转向。
3. 左侧标记后全白：右轮正转、左轮反转，向左原地转。
4. 右侧标记后全白：右轮反转、左轮正转，向右原地转。
5. 原地转动期间第一次重新检测到任意黑线后，清除锐角状态并恢复普通巡线。

## 蓝牙命令

蓝牙串口配置为 `9600 8N1`。命令不区分大小写，推荐每条命令后发送 `\r\n`；程序也会在短时间无新字符后自动提交当前命令。

| 命令 | 示例 | 功能 |
| --- | --- | --- |
| `start` / `run` | `start` | 启动红外巡线 |
| `stop` / `halt` | `stop` | 停止巡线或当前闭环运动 |
| `left 角度` | `left 30` | 闭环左转 1～360 度 |
| `right 角度` | `right 90` | 闭环右转 1～360 度 |
| `forward 距离` | `forward 1.5` | 闭环直行 0.02～10 m |
| `fwd 距离` | `fwd 0.5` | `forward` 的缩写 |
| `SPD=数值` | `SPD=10` | 设置巡线基础速度，范围 0～30 |
| `SPEED=数值` | `SPEED=10` | `SPD=` 的别名 |
| `BASE=数值` | `BASE=10` | `SPD=` 的别名 |
| `status` / `stat` | `status` | 返回模式、进度、Yaw 和 UART 计数 |
| `help` | `help` | 返回命令提示 |

转角和定距离命令依赖 MPU6500 初始化成功。若当前已有闭环动作尚未完成，应先发送 `stop`；否则会返回：

```text
ERR BUSY_USE_STOP
```

## OLED 与 USB 输出

OLED 每 100 ms 刷新一次，并只重绘发生变化的行。当前四行内容为：

```text
yaw:-12.345
odo:1.234m
A:02345 B:02310
RX:forward 1.0
```

没有最近蓝牙字符时，最后一行显示编码器参数，例如：

```text
CPR:390 X1
```

板载 USB 调试串口为 `115200 8N1`，每 100 ms 输出一次：

```text
roll,pitch,yaw
```

示例：

```text
0.125,-1.430,89.672
```

## 工程结构

```text
Control/
  Menu/                 菜单和历史状态代码
  PID/                  巡线 PID、锐角状态机和通用 PID
Drivers/
  Button/               按键驱动
  Encoder/              编码器计数、测速和里程累计
  IR/                   八路红外巡线
  K210/                 视觉串口代码，当前关闭
  MPU6500/              IMU、标定和姿态解算
  Motor/                双电机 PWM 驱动
  MSPM0/                时钟和中断调度
  OLED/                 软件 I2C OLED
  StepperGimbal/         步进云台代码，当前关闭
  Uart/                  蓝牙协议和闭环运动控制
main.c                   初始化、主循环、OLED 和 USB 输出
main.h                   可选功能开关
mspm0-modules.syscfg     SysConfig 外设与引脚配置
```

## 编译

推荐使用 Code Composer Studio 导入工程根目录。当前验证环境：

```text
TI Arm Clang: 4.0.4.LTS
MSPM0 SDK:    2.10.00.04
SysConfig:    1.26.2
器件:         MSPM0G3507
封装:         LQFP-64 (PM)
```

命令行验证构建：

```powershell
& "D:\ccs\utils\bin\gmake.exe" -C Debug -j 8 all -r -O
```

上电后先保持小车静止，等待 MPU6500 完成零偏标定，再通过 `PB21` 按键或蓝牙 `start` 启动巡线。

## 注意事项

- 第一次调试应架空车轮，确认 `start`、`stop`、前进方向和编码器方向正确后再落地。
- 电机电源、逻辑电源、蓝牙、传感器和调试器必须共地。
- 修改引脚时优先修改 `mspm0-modules.syscfg`，不要直接编辑 `Debug/ti_msp_dl_config.*` 生成文件。
- 左右轮不对称时，先确认 Motor A 为右轮、Motor B 为左轮，再检查编码器方向和 `MOTOR_B_PWM_PERCENT`。
- Yaw 闭环不准时，先检查上电静止标定、IMU 安装方向和 USB 输出数据，再调整 PID。
- 定距离误差较大时，应实测每圈编码器计数和轮胎有效直径，并修改 `Drivers/Encoder/encoder.h`。
- 重新启用视觉、云台或 HC-SR04 前，必须重新检查上面的引脚冲突表。
