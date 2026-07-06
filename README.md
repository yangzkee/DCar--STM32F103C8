# DCar 官方小车例程 - STM32F103C8

适用于 STM32F103C8 的 DCar / DcarON 小车底盘通信例程，支持 Keil MDK 工程和 macOS Makefile 两种使用方式。

产品与教程页面：

https://differ-tech.pages.dev/portal/view/dcar-fast-motion-control

本例程代码文件按 UTF-8 编码保存。

## 仓库内容

```text
.
├── DFCom_Example/       # 完整工程：Keil 工程 + macOS Makefile/GCC 工程
├── DFCom_PatchOnly/     # 便于移植到已有 STM32 工程的核心通信文件
├── Makefile             # 顶层 Makefile，转发到 DFCom_Example
└── README.md            # 本说明
```

## 硬件接线

### STM32F103C8 与小车底盘

STM32 通过 USART1 与 DCar 小车底盘通信，负责发送运动指令并接收 Odom / VelPos 数据。

| STM32F103C8 引脚 | 连接到小车底盘 | 用途 | 参数 |
|---|---|---|---|
| PA9 / USART1_TX | 小车 UART_RX | STM32 发送控制指令到小车 | 460800, 8N1 |
| PA10 / USART1_RX | 小车 UART_TX | STM32 接收 Odom / 状态回传 | 460800, 8N1 |
| GND | 小车 GND | 共地 | 必接 |

注意：

- TX/RX 需要交叉连接。
- STM32 与小车底盘必须共地。
- 串口电平使用 3.3V TTL。
- 小车底盘和 STM32 开发板建议各自按规范供电，不要从 USB-TTL 给小车电机供电。

### STM32F103C8 与电脑

STM32 通过 USART2 把调试信息和里程计数据打印到电脑串口终端。

| STM32F103C8 引脚 | 连接到 USB-TTL | 用途 | 参数 |
|---|---|---|---|
| PA2 / USART2_TX | USB-TTL RX | 电脑显示 printf / Odom 数据 | 115200, 8N1 |
| PA3 / USART2_RX | USB-TTL TX | 预留接收，当前例程可不接 | 115200, 8N1 |
| GND | USB-TTL GND | 共地 | 必接 |

电脑端打开串口工具，选择 USB-TTL 对应的串口，参数设置为：

```text
115200 baud
8 data bits
No parity
1 stop bit
No flow control
```

## 软件环境

### Keil MDK 版本

用于 Windows / Keil 用户。

必须安装：

1. Keil MDK5，工程使用 ARM Compiler 5 编译。
2. STM32F103C8 对应的 STM32F1 Device Family Pack，例如 `Keil.STM32F1xx_DFP.2.4.1.pack`。
3. Keil 工程中启用 `Use MicroLIB`，否则 `printf` 重定向可能无法正常输出。

ARM Compiler 5 和 STM32F103C8 芯片包获取链接：

https://ucnerk2uhr85.feishu.cn/wiki/BaUMwzR4liGc6FkMES1c64c6nOb?renamingWikiNode=true#share-APaqdPwYPo1PIhxCeiac3fzJnih

打开链接后请耐心等待约 5 秒，页面会自动跳转到对应位置。

打开工程：

```text
DFCom_Example/USER/Template.uvprojx
```

推荐检查：

- Target Device: `STM32F103C8`
- Define: `STM32F10X_MD, USE_STDPERIPH_DRIVER`
- Startup: `startup_stm32f10x_md.s`
- Code Generation: 勾选 `Use MicroLIB`

### macOS Makefile 版本

用于 macOS 终端编译和烧录。

安装工具：

```sh
brew install arm-none-eabi-gcc stlink openocd
```

编译：

```sh
make
```

生成文件：

```text
DFCom_Example/build/dfcom_example.elf
DFCom_Example/build/dfcom_example.bin
DFCom_Example/build/dfcom_example.hex
```

使用 ST-Link 烧录：

```sh
make flash
```

使用 OpenOCD 烧录：

```sh
make flash-openocd
```

如果使用 CMSIS-DAP：

```sh
make OPENOCD_INTERFACE=interface/cmsis-dap.cfg flash-openocd
```

## VelPos 数据显示逻辑

例程启动后会初始化：

```c
uart_init(460800);          // USART1 接小车
usart2_init(115200);        // USART2 接电脑 USB-TTL
Odom_PrintTimer_Init();     // TIM2 10Hz 自动打印
```

随后通过启动诊断函数发送 VelPos 订阅请求：

```c
Cmd_Subscribe_VelPos(ODOM_MODE_CONTINUOUS, ODOM_FREQ_10HZ);
```

启动阶段先发送一次订阅，后续由主循环旁路维护：STM32 上电通常比小车底盘协议服务 ready 更早，如果小车还没准备好，第一次订阅可能被错过；这就是“上电没数据，软复位后有数据”的典型原因。例程现在会在后台约每 2 秒检查一次，如果 VelPos 帧计数没有增长，就补发一次 `Cmd_Subscribe_VelPos()`，避免高频重发把底盘访问状态机反复重置。

VelPos 只用于显示/观测，不能作为运动命令的前置条件。本例程即使暂时收不到 VelPos，也会继续执行前进、后退、左转、右转演示；VelPos 订阅会在后台继续重试，直到数据开始回传。

注意：`Cmd_Subscribe_VelPos()` 这个 API 默认给用户读 VelPos，但底层订阅入口仍按当前底盘固件要求发送 `0x04/0x80`；VelPos 本身是回传帧 `0x6C/0x81`。不要把订阅命令误改成 `0x04/0x81`，否则部分底盘会一直没有数据。

小车收到订阅请求后持续回传 VelPos v2 数据。STM32 使用 USART1 DMA + IDLE 中断接收，`DFCom_RxParse()` 自动解析数据，并写入：

```c
g_odom
g_velpos
g_dcar_state
```

TIM2 每 100ms 自动调用打印函数，通过 USART2 把数据输出到电脑串口终端。

## 当前主函数演示动作

当前 `main.c` 已设置为小幅动作演示：

```c
Cmd_Move_Linear( 1, 0, 5, 0);  delay_ms(1500);  // 前进 1 cm, 等 1.5 秒
Cmd_Move_Linear(-1, 0, 5, 0);  delay_ms(1500);  // 后退 1 cm, 等 1.5 秒
Cmd_Move_Rot( 15, 30);         delay_ms(1500);  // 左转 15 deg, 等 1.5 秒
Cmd_Move_Rot(-15, 30);         delay_ms(1500);  // 右转 15 deg, 等 1.5 秒
```

默认单位模式：

```c
g_dfcom_unit_mode = DFCOM_UNIT_CM;
```

也就是直线单位为 cm / cm/s，角度单位为 deg / deg/s。

## 接口功能清单

不同底盘固件和不同版本可用接口可能不同。Pro 版支持本例程中的全部接口函数。

### 运动控制

```c
void Cmd_Move_Linear(float px, float py, float speed_mps, u8 profile);
void Cmd_Move_LinearWithYaw(float px, float py, float dyaw_rad, float speed_mps, u8 profile);
void Cmd_Move_Rot(float dyaw_rad, float omega_max_rad_s);
void Cmd_Move_Vel(float vx_mps, float vy_mps, float vz_rad_s);
void Cmd_Move_Arc(float radius_m, float dyaw_rad, float speed_mps, u8 profile);
```

说明：

- `Cmd_Move_Linear`: 直线位移。
- `Cmd_Move_LinearWithYaw`: 平移同时旋转。
- `Cmd_Move_Rot`: 原地旋转。
- `Cmd_Move_Vel`: 持续速度控制，需要主动发送 `Cmd_Move_Vel(0, 0, 0)` 停车。
- `Cmd_Move_Arc`: 圆弧运动。
- `profile`: `0` 匀速，`1` 梯形加减速。

### VelPos / Odom 和状态

```c
void Cmd_Subscribe_VelPos(u8 mode, u8 freq_hz);
void Cmd_Subscribe_Odom(u8 mode, u8 freq_hz);
void Cmd_Query_DcarState(void);
u8 WaitMoveDone(u8 cmd_code, u32 timeout_ms);
u8 GetMoveProgress(u8 cmd_code);
```

说明：

- `Cmd_Subscribe_VelPos`: 默认使用，订阅 VelPos v2 简化数据。
- `Cmd_Subscribe_Odom`: 订阅 Odom v4 全量数据。
- `Cmd_Query_DcarState`: 查询小车激活、IMU 校准、控制参数状态。
- `WaitMoveDone`: 等待运动完成，推荐 `timeout_ms = 0`。
- `GetMoveProgress`: 非阻塞读取运动进度。

### 可读取数据

```c
extern volatile OdomData_t g_odom;
extern volatile VelPosData_t g_velpos;
extern volatile DcarState_t g_dcar_state;
extern volatile MoveStatus_t g_move_status[8];
```

常用字段：

```c
g_odom.yaw_rad
g_odom.n_pos_x_m
g_odom.n_pos_y_m
g_odom.b_vel_x_mps
g_odom.b_vel_y_mps
g_odom.gyro_z_rps
g_odom.frame_count

g_velpos.yaw_rad
g_velpos.n_pos_x_m
g_velpos.n_pos_y_m
g_velpos.b_vel_x_mps
g_velpos.b_vel_y_mps
g_velpos.frame_count
```

接收侧数据始终为 SI 单位：

- 位置：m
- 速度：m/s
- 角度：rad
- 角速度：rad/s

### 打印与底层串口

```c
void VelPos_Print(void);
void Odom_Print(void);
void Odom_PrintTimer_Init(void);
void DFCom_RxParse(u8 *data, u16 size);
void uart_init(u32 bound);
void usart2_init(u32 bound);
```

默认自动打印 `VelPos_Print()`。如果需要全量 Odom / IMU 数据，可以在 `DFCom_Print.c` 的 TIM2 中断中启用 `Odom_Print()`。

## 版本兼容说明

- Pro 版：支持本例程中的所有运动控制、Odom、状态查询和进度回传接口。
- 非 Pro 或早期固件：可能只支持部分接口，例如基础速度控制、直线位移或低频 Odom。
- 如果某个接口没有响应，请先确认底盘固件版本、车型版本、是否已激活、IMU 是否已校准，以及 USART1 波特率是否为 460800。

## Release 包说明

Release 中提供两份例程包：

- `DCar-STM32F103C8-Keil.zip`: Keil MDK / ARM Compiler 5 工程版。
- `DCar-STM32F103C8-Makefile.zip`: macOS Makefile / arm-none-eabi-gcc 版本。

编译器安装包、Keil MDK 安装包、芯片 Pack 等第三方工具请从对应官方渠道获取并按授权使用。

如需 ARM Compiler 5 和 STM32F103C8 芯片包，请打开下面链接，打开后耐心等待约 5 秒，页面会自动跳转到对应位置：

https://ucnerk2uhr85.feishu.cn/wiki/BaUMwzR4liGc6FkMES1c64c6nOb?renamingWikiNode=true#share-APaqdPwYPo1PIhxCeiac3fzJnih
