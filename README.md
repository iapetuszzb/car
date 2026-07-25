# MSPM0G3507 智能小车控制工程

本工程是基于 **TI MSPM0G3507（LQFP-64）** 的两轮差速智能小车程序，当前主要用于八路红外巡线、锐角弯处理、蓝牙遥控、编码器闭环运动和 MPU6500 姿态测量。

当前版本以 `main` 分支为准。云台和视觉串口相关代码仍保留在工程中，但默认已经关闭，不会初始化或驱动对应模块。

## 现有功能

- 八路红外循迹，黑线输出高电平，白底输出低电平。
- 普通位置式 PID 巡线，当前参数为 `P=2500`、`I=0`、`D=800`。
- 根据偏差大小自动降低弯道基础速度，并加强转向输出。
- 锐角处理：最左或最右探头见黑后若进入全白，保存最后方向并原地急转；重新检测到黑线后恢复普通 PID。
- 双路直流电机 PWM 正反转控制。
- 双编码器测速和里程累计。
- 蓝牙串口命令控制巡线、转角和定距离直行。
- 左转、右转采用 MPU6500 航向角闭环和编码器速度闭环。
- 定距离直行采用编码器距离闭环、速度 PI、加减速控制和航向角修正。
- MPU6500 上电静止标定、姿态解算和 `1.5 dps` 陀螺仪死区抑制。
- OLED 实时显示三位小数的 `yaw` 角和最近收到的蓝牙 ASCII 字符。
- HC-SR04 超声波测距及 Alpha-Beta 滤波代码已启用，但目前只采集数据，尚未参与避障和运动控制。
- 主板 `PB21` 按键可启动巡线模式。

## 当前模块状态

以下开关位于 `main.h`：

```c
#define APP_ENABLE_STEPPER_GIMBAL 0
#define APP_ENABLE_SERVO_SWEEP    0
#define APP_ENABLE_VISION_UART    0
```

| 模块 | 当前状态 | 说明 |
| --- | --- | --- |
| 八路红外巡线 | 启用 | 普通 PID 和锐角原地转向 |
| 蓝牙 UART | 启用 | UART1，9600 baud，8N1 |
| MPU6500 | 启用 | I2C1，400 kHz |
| OLED | 启用 | 软件 I2C，显示方向已旋转 180 度 |
| 编码器闭环 | 启用 | 转角和定距离直行使用 |
| HC-SR04 | 启用 | 仅测距和滤波，暂不控制车辆 |
| K210/K230 视觉接收 | 关闭 | 原有代码保留，当前 UART1 给蓝牙使用 |
| 步进云台 | 关闭 | 原有驱动保留，不初始化引脚 |
| 舵机扫描 | 关闭 | 原有代码保留，不输出控制脉冲 |

## 引脚分配

### 电机驱动

电机驱动采用每个电机两路 PWM 的方式实现正转、反转和停止。

| 功能 | MSPM0 引脚 | 外设 | 备注 |
| --- | --- | --- | --- |
| Motor A 通道 0 | `PB4` | `TIMA1_CCP0` | 右轮 |
| Motor A 通道 1 | `PB1` | `TIMA1_CCP1` | 右轮 |
| Motor B 通道 0 | `PA28` | `TIMG7_CCP0` | 左轮 |
| Motor B 通道 1 | `PA31` | `TIMG7_CCP1` | 左轮 |

代码中的轮子关系固定为：

```text
Motor A = 右轮
Motor B = 左轮
```

由于左右机械阻力和电机差异，当前在 `Drivers/Motor/motor.c` 中对 Motor B 使用 `90%` PWM 补偿。

### 编码器

| 功能 | MSPM0 引脚 | 说明 |
| --- | --- | --- |
| Encoder A 相位 A / 中断 | `PA12` | Motor A，即右轮编码器 |
| Encoder A 相位 B / 方向判断 | `PA13` | Motor A，即右轮编码器 |
| Encoder B 相位 A / 中断 | `PB20` | Motor B，即左轮编码器 |
| Encoder B 相位 B / 方向判断 | `PB13` | Motor B，即左轮编码器 |

当前编码器按 A 相下降沿计数。程序使用 `13` 线编码器和 `30:1` 减速比，换算为：

```text
390 脉冲/轮转
轮径 65 mm
轮周长约 204.2035 mm
```

如果实际减速比、编码器线数或轮径变化，需要同步修改 `Drivers/Uart/uart.c` 中的编码器参数。

### 八路红外寻迹

传感器排列从车辆前进方向观察，由左到右为 `line0` 到 `line7`。

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

当前电平定义：

```text
黑线 = 高电平 1
白底 = 低电平 0
八路全 0 = 丢线/全白
```

### 蓝牙串口

| 功能 | MSPM0 引脚 | 接线 |
| --- | --- | --- |
| UART1 TX | `PA17` | 接蓝牙模块 RX |
| UART1 RX | `PA18` | 接蓝牙模块 TX |
| GND | GND | 与蓝牙模块共地 |

串口配置：

```text
波特率：9600
数据位：8
停止位：1
校验位：无
流控：无
```

`PA18` 接收脚启用了内部上拉。蓝牙模块和 MSPM0 必须共地，并确认串口电平为 `3.3 V TTL`，不要把 RS-232 电平直接接入 MCU。

### MPU6500

| 功能 | MSPM0 引脚 | 说明 |
| --- | --- | --- |
| I2C1 SCL | `PB2` | 400 kHz |
| I2C1 SDA | `PB3` | 400 kHz |
| MPU6500 INT | `PB9` | 上升沿中断输入 |

上电时必须保持小车静止，程序会进行陀螺仪零偏标定。OLED 会先显示 `MPU CAL WAIT` 和 `KEEP CAR STILL`，标定成功后显示 `MPU READY`。

### OLED

| 功能 | MSPM0 引脚 | 说明 |
| --- | --- | --- |
| OLED SCL | `PB19` | 软件 I2C 时钟 |
| OLED SDA | `PA25` | 软件 I2C 数据 |

当前 OLED 初始化使用 `0xA0` 和 `0xC0`，画面相对默认方向旋转了 180 度。显示内容包括：

- 第一行：`yaw:xx.xxx`。
- 其余行：最近收到的蓝牙 ASCII 字符；没有数据时显示 UART 接收计数和错误状态。

### 按键、指示灯和超声波

| 功能 | MSPM0 引脚 | 说明 |
| --- | --- | --- |
| 启动巡线按键 | `PB21` | 内部上拉，按下后启动巡线 |
| 普通 LED | `PA0` | 状态指示 |
| RGB/状态 LED | `PB22` | 启动巡线时翻转 |
| HC-SR04 TRIG | `PB10` | 超声波触发输出 |
| HC-SR04 ECHO | `PB11` | 超声波回波输入 |

## 蓝牙命令

命令不区分大小写。建议每条命令末尾发送换行符 `\r\n`；程序也支持在短时间无新字符后自动提交当前命令。

| 命令 | 示例 | 功能 |
| --- | --- | --- |
| `start` / `run` | `start` | 启动普通红外巡线 |
| `stop` / `halt` | `stop` | 立即停止当前运动和巡线 |
| `left 角度` | `left 30` | 闭环向左转指定角度，范围 1 到 360 度 |
| `right 角度` | `right 90` | 闭环向右转指定角度，范围 1 到 360 度 |
| `forward 距离` | `forward 1.5` | 闭环直行指定距离，单位 m，范围 0.02 到 10 m |
| `fwd 距离` | `fwd 0.5` | `forward` 的缩写 |
| `SPD=速度` | `SPD=10` | 设置巡线基础速度，范围 0 到 30 |
| `status` / `stat` | `status` | 返回运行状态、动作进度、yaw 和 UART 计数 |
| `help` | `help` | 返回命令提示 |

示例：

```text
left 30
right 45
forward 1.0
status
stop
start
```

执行转角或定距离命令时，如果已有动作尚未完成，程序会返回：

```text
ERR BUSY_USE_STOP
```

可以先发送 `stop`，再发送下一条运动命令。若 MPU6500 尚未初始化完成，则会返回：

```text
ERR IMU_NOT_READY
```

## 巡线控制逻辑

默认巡线参数位于 `main.c`：

```c
P = 2500.0f;
I = 0.0f;
D = 800.0f;
basespeed = 10.0f;
TargetLine = 4.5f;
```

PWM 和弯道参数位于 `Control/PID/pid.c`：

```c
#define BASE_SPEED_TO_PWM 1700
#define MIN_RUN_PWM 12000
#define START_BOOST_PWM 17000
#define LINE_TURN_PWM_LIMIT 16000
```

默认 `basespeed=10` 时，直行基础 PWM 约为：

```text
10 x 1700 = 17000
```

锐角处理流程：

1. 正常巡线时，`line0` 或 `line7` 检测到黑线，保存最后外侧方向。
2. 随后若八路全部进入白底，立即停止普通巡线 PID。
3. 左侧标记后全白：右轮正转、左轮反转，向左原地转。
4. 右侧标记后全白：右轮反转、左轮正转，向右原地转。
5. 原地转动期间第一次重新检测到任意黑线，清除锐角标记并恢复普通巡线 PID。

锐角原地转向使用与当前直行相同的基础 PWM，不再单独使用过高的固定转向 PWM。

## 工程结构

```text
Control/
  Menu/                 菜单和状态显示
  PID/                  电机、巡线 PID 和锐角状态机
Drivers/
  Button/               按键
  Encoder/              编码器计数与测速
  IR/                   八路红外寻迹
  K210/                 视觉串口代码，当前关闭
  MPU6500/              IMU 驱动、标定和姿态解算
  Motor/                双电机 PWM 驱动
  MSPM0/                时钟和中断辅助代码
  OLED/                 软件 I2C OLED 驱动
  StepperGimbal/         步进云台代码，当前关闭
  Uart/                  蓝牙 UART、命令解析和闭环运动
main.c                   初始化和主循环
main.h                   当前功能开关
mspm0-modules.syscfg     SysConfig 外设与引脚配置
```

## 编译和烧录

推荐使用 Code Composer Studio 导入工程根目录：

```text
D:\2025-CAR
```

当前本机验证环境：

- CCS 工具链：TI Arm Clang `4.0.4.LTS`
- MSPM0 SDK：`2.10.00.04`
- SysConfig：`1.26.2`
- 目标器件：`MSPM0G3507`
- 封装：`LQFP-64 (PM)`

操作步骤：

1. 在 CCS 中选择 `File -> Import`，导入已有 CCS 工程。
2. 检查 `mspm0-modules.syscfg` 中的器件、UART 和引脚配置。
3. 选择 `Debug` 配置并执行 `Build Project`。
4. 通过板载调试器下载生成的程序。
5. 上电后保持小车静止，等待 MPU6500 标定完成。
6. 使用 `PB21` 按键或蓝牙 `start` 命令启动巡线。

## 使用注意事项

- 调试前先架空车轮，确认左右轮方向、编码器方向和急停命令正确，再落地测试。
- 电机电源与逻辑电源应按驱动板要求接线，所有模块必须共地。
- 蓝牙模块必须使用 `3.3 V TTL UART` 电平。
- 上电标定期间不要移动小车，否则 yaw 零偏会影响闭环转向和直行。
- 如果 `left` 和 `right` 实际方向相反，优先检查电机接线和 IMU 安装方向，再调整控制符号。
- 如果定距离误差较大，先实测每圈编码器脉冲数和轮胎有效直径，再修改换算参数。
- 如果普通巡线左右不对称，确认 Motor A 是右轮、Motor B 是左轮，并检查 `MOTOR_B_PWM_PERCENT` 补偿。
- `Debug/` 是构建输出目录，不应手工修改其中自动生成的 SysConfig 文件；引脚配置应修改工程根目录的 `mspm0-modules.syscfg`。

## 当前版本说明

当前稳定版本对应 Git 提交：

```text
b2278b8 Integrate Bluetooth motion control and sharp-corner tracking
```

后续计划可在现有框架上继续加入 FreeRTOS 任务调度、超声波避障、视觉模块通信以及更完整的人机交互。
