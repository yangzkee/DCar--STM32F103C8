# DCar 官方小车例程 - STM32F103C8 / DFCom v2

适用于 STM32F103C8 的 DCar / DcarON 小车底盘通信例程，现已对齐 DFCom v2 协议。  

产品与教程页面：  
https://differ-tech.pages.dev/portal/view/dcar-fast-motion-control

本例程代码文件按 UTF-8 编码保存。

## 仓库内容

```text
.
├── DFCom_Example/          # STM32 Keil 示例工程（主线）
├── DFCom_PatchOnly/        # 通信核心文件（用于移植）
├── DFCom_Arduino/          # Arduino 库 + 示例（新增）
├── MG520_Firmware_Update... # MG520 固件更新包
└── README.md               # 本说明
```

## 两条起步路径

1. **STM32 正式版（Keil）**：面向工程化开发与上线调试。  
2. **Arduino 快速版**：适合先把动作跑起来、先做教学验证。

两条路径共用同一套协议、坐标系和动作语义。

## 硬件接线（STM32F103C8）

| STM32F103C8 引脚 | 连接到小车底盘 | 用途 | 参数 |
|---|---|---|---|
| PA9 / USART1_TX | 小车 UART_RX | 发送控制指令 | 460800, 8N1 |
| PA10 / USART1_RX | 小车 UART_TX | 接收 ODOM/VelPos 回传 | 460800, 8N1 |
| GND | 小车 GND | 共地 | 必接 |

| STM32F103C8 引脚 | 连接到 USB-TTL | 用途 | 参数 |
|---|---|---|---|
| PA2 / USART2_TX | USB-TTL RX | 串口打印（printf） | 115200, 8N1 |
| PA3 / USART2_RX | USB-TTL TX | 预留接收 | 115200, 8N1 |
| GND | USB-TTL GND | 共地 | 必接 |

- TX/RX 需要交叉连接。
- 小车与开发板建议按规范供电，不要用 USB-TTL 给小车电机供电。
- STM32 与小车底盘通信为 3.3V TTL 电平，注意电平匹配。

## 软件环境

### Keil MDK（推荐，STM32）

1. 安装 Keil MDK5（本工程使用 ARM Compiler 5）。
2. 安装 STM32F1 Device Family Pack，目标芯片 `STM32F103C8`。
3. 打开工程：`DFCom_Example/USER/Template.uvprojx`。
4. 编译前检查：
   - Target Device：`STM32F103C8`
   - Define：`STM32F10X_MD, USE_STDPERIPH_DRIVER`
   - Startup：`startup_stm32f10x_md.s`
   - Code Generation：勾选 `Use MicroLIB`

官方 ARM Compiler / 芯片包来源请按官方授权渠道获取：  
https://ucnerk2uhr85.feishu.cn/wiki/BaUMwzR4liGc6FkMES1c64c6nOb?renamingWikiNode=true#share-APaqdPwYPo1PIhxCeiac3fzJnih

### Arduino（可选）

1. 进入 `DFCom_Arduino/DFCom`。
2. 按 `DFCom_Arduino/README.md` 的方式导入库/示例并编译上传。
3. 先确认串口与接线，再跑示例动作。

## 启动流程（STM32）

1. 用 Keil 打开工程，先进行一次编译。
2. 烧录到 MCU。
3. 打开串口工具连接 USART2（115200 8N1）。
4. 上电后看到初始化输出：

```text
[INIT] Subscribe ODOM + VelPos @ 10Hz...
[INIT] ODOM data flowing ✓
[ODOM] Yaw= ... X(fwd)=... Y(left)=... Vx=... Vy=... Gz=...
```

5. 修改 `DFCom_Example/USER/main.c` 里 `while(1)` 的运动逻辑即可。

## 坐标系与单位（关键）

协议层始终是 SI 单位（用于所有接收数据）：

- 位置：米（m）
- 速度：米每秒（m/s）
- 角度：弧度（rad）
- 角速度：弧度每秒（rad/s）

发送层对新手默认兼容 CM 模式：

```c
g_dfcom_unit_mode = DFCOM_UNIT_CM; // 默认：厘米/度
g_dfcom_unit_mode = DFCOM_UNIT_M;  // 需要 SI 时切换：米/弧度
```

坐标系约定（ROS REP-103）：

```text
        +X (前方, forward)
         ↑
 +Y      │
(左方) ←─┼──→ -Y
left     │
         ↓
        -X
```

- `+X`：小车前方（向前）
- `+Y`：小车左侧（向左）
- `+Yaw`：CCW（逆时针，左转为正）

## API 速查（核心）

### 运动控制

```c
void Cmd_Move_Linear        (float px, float py, float speed_mps, u8 profile);
void Cmd_Move_LinearWithYaw (float px, float py, float dyaw_rad, float speed_mps, u8 profile);
void Cmd_Move_Rot           (float dyaw_rad, float omega_max_rad_s);
void Cmd_Move_Arc           (float radius_m, float dyaw_rad, float speed_mps, u8 profile);
void Cmd_Move_Vel           (float vx_mps, float vy_mps, float vz_rad_s);
```

### 订阅与状态

```c
void Cmd_Subscribe_Odom    (u8 mode, u8 freq_hz);
void Cmd_Subscribe_VelPos  (u8 mode, u8 freq_hz);
void Cmd_Query_DcarState   (void);
u8   WaitMoveDone         (u8 cmd_code, u32 timeout_ms);
u8   GetMoveProgress      (u8 cmd_code);
```

> 建议新手先用 `WaitMoveDone` 验证每一步到位，再进入闭环控制与进度查询。

## 常见问题

**Q: 串口打开但看不到任何输出**
- 检查 PA2/PA3 与 USB-TTL 接线与波特率（115200）
- Keil 是否勾选了 `Use MicroLIB`

**Q: 有初始化输出但没有 ODOM**
- 检查 PA9/PA10 线路、共地与波特率（460800）
- 小车是否上电、是否已激活

**Q: 方向/单位感觉不对**
- 先确认 `+X` 为前方、`+Y` 为左方、`+Yaw` 为 CCW（左转）
- `g_dfcom_unit_mode` 的单位和 `Cmd_Move_*` 入参匹配

## Release 包

当前 Release（v1.0.0）提供：

- `DCar-STM32F103C8-Keil.zip`：STM32 Keil 版完整工程
- `DCar-STM32F103C8-Arduino.zip`：DFCom Arduino 版库与示例
