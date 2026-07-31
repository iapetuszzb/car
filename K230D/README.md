# K230D 钢球视觉识别

本目录保存正点原子 K230D BOX 当前使用的钢球识别程序、`best1.kmodel` 模型和 UART 回环测试脚本。

## 当前上板程序

文件：`steel_ball/steel_ball_only_main.py`

主要配置：

```text
开发板：正点原子 K230D BOX
摄像头：普通摄像头
摄像头设备：sensor_id=2
AI 输入画面：640 x 360
模型输入：320 x 320
模型：best1.kmodel
模型大小：3343832 bytes
模型 SHA-256：4996C263452F00B8F30B42BEB97A07B072DBD79FBC881CD996E9F01434015D62
```

模型默认加载路径：

```text
/sdcard/examples/kmodel/best1.kmodel
```

脚本还会尝试 `/sdcard`、`/data` 和 `/flash` 下的备用路径。运行环境需要 CanMV 镜像自带的：

```text
/sdcard/libs/PipeLine.py
/sdcard/libs/YOLO.py
```

## PORT2 串口

当前识别程序通过 K230D BOX PORT2 发送距离画面中心最近的钢球坐标差：

```text
UART：UART2
TX：IO44
RX：IO45
波特率：115200
格式：8N1
```

只向单片机发送时的接线：

```text
K230D PORT2 TX / IO44 -> 单片机 UART RX
K230D GND             -> 单片机 GND
```

通信格式：

```text
X{水平坐标差}Y{垂直坐标差}Z{目标有效}E\r\n
```

坐标定义：

```python
error_x = screen_center_x - ball_center_x
error_y = screen_center_y - ball_center_y
```

示例：

```text
X35Y-12Z1E
X0Y0Z0E
```

程序会选择距离画面中心最近的钢球。单帧临时漏检时，最多 `500 ms` 继续发送最后一次有效坐标并保持 `Z=1`；超过保持时间后发送 `X0Y0Z0E`。画面顶部使用 `LIVE`、`HOLD`、`LOST` 标记当前状态。

## UART 测试

`tools/` 中提供两个独立回环测试：

```text
k230d_port1_uart_loopback_test.py  UART1，IO40/IO41
k230d_port2_uart_loopback_test.py  UART2，IO44/IO45
```

测试时只需要短接对应端口的 TX 和 RX，不要把电源引脚接入串口信号线。屏幕和 CanMV IDE 终端中的 `RX`、`MATCH` 持续增加表示回环成功。

## 与 STM32 实验工程的协议区别

本目录的当前上板程序发送完整 `X/Y/Z/E` 帧。`STM32/k230_v1.0.0` 中的 C06B 单轴平衡实验工程则使用一个有符号 X 像素误差和换行：

```text
-37\n
0\n
42\n
```

两种协议不能直接混用。接入 C06B 工程时，应使用其 `k230d/pixel_error_sender.py`，或者同步修改 STM32 解析器以接收 `X/Y/Z/E`。
