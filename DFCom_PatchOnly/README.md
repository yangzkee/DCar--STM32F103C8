# DcarON DFCom 客户端例程（精简教学版）

> 给 STM32F103ZET6 用的小车通信库，3 个文件玩转 DcarON。
> **协议版本**：SI 单位 + ROS REP-103 坐标系（与老版本不兼容！）
> **默认入参单位**：厘米 + 度（学生友好；可一键切到 SI 米/弧度）
> 官网：<https://differ-tech.pages.dev/>

## 这是什么

一份**面向大学生教学**的精简版 DcarON 客户端例程：

- 🟢 **入门**：调一行 `Cmd_Move_Linear(0, 50, 30, 2)` 就能让车 +Y 左移 50 cm（30 cm/s）
- 🟡 **进阶**：调完后跟一行 `WaitMoveDone(...)` 等小车到位再下一条
- 🔴 **高手**：直接读 `g_odom.yaw_rad` / `g_odom.n_pos_x_m` 写自己的闭环算法（注意永远是 SI）
- 📺 **数据可视化**：插上 USB-TTL 接到 USART2，PC 终端自动看到 10Hz ODOM 实时数据

## 硬件接线

| STM32F103ZET6 引脚 | 接到 | 波特率 | 用途 |
|---|---|---|---|
| **PA9** (TX) / **PA10** (RX) | DcarON 小车 UART | 460800 | 控制指令 + ODOM 数据回传 |
| **PA2** (TX) / **PA3** (RX) | USB-TTL → 电脑 | 115200 | 终端看 ODOM 数据 |

## 文件清单

```
DFCom_Example/
├── HARDWARE/
│   ├── DFCom.h          ← 总头文件（看这个了解所有 API）
│   ├── DFCom_Tx.c       ← 发送侧（Cmd_Move_* 系列）
│   ├── DFCom_Rx.c       ← 接收侧（解析 ODOM + g_odom 数据）
│   └── DFCom_Print.c    ← TIM2 自动打印 + 本地 tick
├── SYSTEM/
│   ├── usart/usart.c    ← USART1 (小车) + USART2 (printf) 配置
│   ├── usart/usart.h
│   ├── sys/ delay/      ← 标准库辅助（不用动）
├── USER/
│   ├── main.c           ← 入口（看这个学怎么用）
│   ├── Template.uvprojx ← Keil 5 工程
├── STM32F10x_FWLib/     ← ST 标准库
└── CORE/                ← 启动代码
```

## 五分钟上手

1. 用 Keil 5 打开 `USER/Template.uvprojx`
2. **重要**：Options → Target → Code Generation → **勾选 Use MicroLIB**（不然 printf 不工作）
3. 编译、烧录到板子
4. 板子 PA9/PA10 接小车 UART，PA2/PA3 接 USB-TTL
5. PC 打开串口工具（115200 8N1）接 USB-TTL 那个 COM 口
6. 上电，终端就能看到：
   ```
   [INIT] Subscribe ODOM @ 10Hz...
   [INIT] ODOM data flowing ✓ (got 15 frames in 1.5s)
   [ODOM] Yaw=  0.12 X(fwd)=  0.000 Y(left)=  0.000 Vx(fwd)=  0.00 Vy(left)=  0.00 Gz=  0.01 fr=1
   [ODOM] Yaw=  0.15 X(fwd)=  0.024 Y(left)=  0.000 Vx(fwd)=  0.25 Vy(left)=  0.00 Gz=  0.02 fr=2
   ...
   ```
7. 改 `main.c` 里 `while(1)` 里的指令，让小车做你想做的动作

## ★ 坐标系与单位约定（重要！）

协议层底层用 **SI 单位** + **ROS REP-103** 坐标系（与老版本不兼容），
但客户端例程允许用户**用厘米 + 度调用**，由本库内部自动转换：

| 项目 | CM 模式 (默认) | M 模式 (SI) |
|---|---|---|
| 位置 px, py | 厘米 (cm) ─ 例: `50` | 米 (m) ─ 例: `0.5` |
| 速度 speed, vx, vy | 厘米/秒 (cm/s) ─ 例: `30` | 米/秒 (m/s) ─ 例: `0.3` |
| 角度 dyaw | 度 (deg) ─ 例: `90` | 弧度 (rad) ─ 例: `1.5708` |
| 角速度 omega, vz | 度/秒 (deg/s) ─ 例: `60` | 弧度/秒 (rad/s) ─ 例: `1.0` |

**默认是 CM 模式**（学生友好）。想切到 SI: `g_dfcom_unit_mode = DFCOM_UNIT_M;`
详见下面 "切换单位模式" 一节。

**坐标系（ROS REP-103, 永远不变）**：

```
        +X (前方, forward)
         ↑
         │
 +Y      │
(左方) ←─┼──→ -Y
left     │
         │
         ↓
        -X
```

- `+X` = 小车前方 ← `px > 0` / `vx > 0` 让车前进
- `+Y` = 小车左方 ← `py > 0` / `vy > 0` 让车左移
- `+Yaw` = CCW 逆时针（从上往下看，右手定则绕 +Z）← `dyaw > 0` 让车左转

**速度范围参考**：
- CM 模式典型使用：10 ~ 50 cm/s; omega_max 30 ~ 120 deg/s
- M  模式典型使用：0.1 ~ 0.5 m/s; omega_max 0.5 ~ 2.0 rad/s
- 物理上限：取决于车型（标准版 520 电机大约 80 ~ 100 cm/s）

## 切换单位模式

默认是 **厘米 + 度** (CM 模式)，适合学生 / 不熟悉 SI 的用户。

```c
// main.c System_Init() 最后:
g_dfcom_unit_mode = DFCOM_UNIT_CM;   // ★ 默认, 不写也行
Cmd_Move_Linear(0, 50, 30, 2);          // +Y 左移 50 cm, 30 cm/s
Cmd_Move_Rot(90, 60);                // 左转 90°, 最大 60 deg/s
```

如果你想用 **米 + 弧度** (M 模式, 协议原生 SI):

```c
g_dfcom_unit_mode = DFCOM_UNIT_M;
Cmd_Move_Linear(0, 0.5f, 0.3f, 2);      // +Y 左移 0.5 m, 0.3 m/s
Cmd_Move_Rot(1.5708f, 1.0f);         // 左转 π/2 rad, 最大 1.0 rad/s
```

**注意**：这个开关**只影响发送侧** (`Cmd_Move_*`)。接收的 `g_odom` 永远是 SI 原生：

- `g_odom.yaw_rad` 弧度
- `g_odom.n_pos_x_m` 米

想看 cm/deg：`g_odom.n_pos_x_m * 100`, `DFCOM_RAD2DEG(g_odom.yaw_rad)`

## API 速查

> 入参单位看 `g_dfcom_unit_mode`。下面参数名里的 `_mps / _rad` 是 SI 模式下的含义；
> CM 模式入参实际是 `cm / cm-per-s / deg / deg-per-s`，库内部自动转 SI 上线。

### 控制指令（在 `main.c` 里调用）

```c
/* 直线位移（profile: 0=匀速, 1=梯形加减速, 2=高精度距离闭环） */
void Cmd_Move_Linear        (float px,    float py,    float speed_mps, u8 profile);

/* 平移 + yaw 同时（"无头"模式） */
void Cmd_Move_LinearWithYaw (float px,    float py,    float dyaw_rad, float speed_mps, u8 profile);

/* 原地旋转（相对增量，CCW+） */
void Cmd_Move_Rot           (float dyaw_rad, float omega_max_rad_s);

/* 持续速度（遥控用，无完成回传，要主动停） */
void Cmd_Move_Vel           (float vx_mps, float vy_mps, float vz_rad_s);

/* 圆弧 */
void Cmd_Move_Arc           (float radius_m, float dyaw_rad, float speed_mps, u8 profile);

/* 订阅 ODOM / VelPos 数据 */
void Cmd_Subscribe_Odom     (u8 mode, u8 freq_hz);
void Cmd_Subscribe_VelPos   (u8 mode, u8 freq_hz);
```

### ODOM 订阅模式

```c
Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS, 10);  // ★持续模式，10Hz（最常用）
Cmd_Subscribe_VelPos(ODOM_MODE_CONTINUOUS, 10);// 简化帧，按需订阅
Cmd_Subscribe_Odom(ODOM_MODE_ONESHOT,    1);   // 拍一帧快照
Cmd_Subscribe_Odom(ODOM_MODE_STOP,       0);   // 停止推送
```

**重要**：持续模式下小车收到一次订阅就会自己一直推送，**客户端不需要反复发请求**！

### 读 ODOM 数据

```c
extern volatile OdomData_t g_odom;

g_odom.yaw_rad       // 当前航向角（单位 rad，CCW+，范围 [-π, π]；多圈累计需自行 unwrap）
g_odom.n_pos_x_m     // 世界系 +X 前进 位置（米）
g_odom.n_pos_y_m     // 世界系 +Y 左方 位置（米）
g_odom.b_vel_x_mps   // 车体系 Vx 前进 (m/s)
g_odom.b_vel_y_mps   // 车体系 Vy 左方 (m/s)
g_odom.acc_x_mps2    // 加速度 +X 前 (m/s²)
g_odom.gyro_z_rps    // 角速度 Z (rad/s, CCW+)
g_odom.frame_count   // 总帧数（用来判数据流是不是活的）
```

> ✅ **协议层 yaw 现在统一是弧度 (rad)**，与发送层 `dyaw_rad` 单位一致, 闭环不再需要转换。
> 想看度数: `DFCOM_RAD2DEG(g_odom.yaw_rad)` 或 `g_odom.yaw_rad * 57.29578f`。
> 终端 `Odom_Print` 输出的 Yaw 列已经自动转成度方便阅读, 实际数据仍是 rad。

### 等运动完成（可选）

```c
// ★ 第二参 = 0 表示 "永久等待 (不超时)", 这是最安全的写法 ★
WaitMoveDone(CMD_LINEAR, 0)            // 等直线位移指令完成 (永久等)
WaitMoveDone(CMD_LINEAR_WITH_YAW, 0)   // 等无头位移
WaitMoveDone(CMD_ARC, 0)               // 等圆弧
WaitMoveDone(CMD_ROT, 0)               // 等旋转

// CMD_VEL 持续速度没有完成回传，不能用 WaitMoveDone

// 不阻塞的进度查询（适合做 UI 进度条）
u8 prog = GetMoveProgress(CMD_LINEAR);    // 0~254 进度, 255=完成
```

⚠️ **不要轻易给第二参填一个小数字 (e.g. 5000ms)**，万一比小车实际跑这段
所需的物理时间还短，会触发"雪崩吃单" bug：客户端超时返回 → 立刻发下一条
→ MCU 端被打断 → 立刻回 0xFF/0x64 → 客户端被骗"完成" → 又发下一条 →
**一秒疯发 3~5 条，每条几乎没动**。详见 `DFCom_Rx.c::WaitMoveDone` 注释。

## 三个等级的用法示例（默认 CM 模式）

> SI 模式数值见上面 "切换单位模式" 一节。

### 🟢 入门 — 粗暴 delay

```c
while (1) {
    Cmd_Move_Linear(50, 0, 30, 2);       // +X 前进 50 cm, 30 cm/s
    delay_ms(3000);                   // 粗暴等 3 秒
    Cmd_Move_Linear(0, 50, 30, 2);       // +Y 左移 50 cm
    delay_ms(3000);
}
```

### 🟡 进阶 — 等回传

```c
while (1) {
    Cmd_Move_Linear(50, 0, 30, 2);    // +X 前进 50 cm, 30 cm/s
    WaitMoveDone(CMD_LINEAR, 0);   // 等小车真到位 (0 = 永久等待, 推荐)
    Cmd_Move_Linear(0, 50, 30, 2);    // +Y 左移 50 cm
    WaitMoveDone(CMD_LINEAR, 0);
}
```

### 🔴 高手 — 自定义闭环

```c
// 自己实现"走到 (1, 1) 米"（+X 前进 1m, +Y 左移 1m）
// 注意: g_odom 永远是 SI (米/弧度), 不受 g_dfcom_unit_mode 影响
float target_x = 1.0f, target_y = 1.0f;
while (1) {
    float dx = target_x - g_odom.n_pos_x_m;    // +X 误差 (米)
    float dy = target_y - g_odom.n_pos_y_m;    // +Y 误差 (米)
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist < 0.05f) break;                   // 5 cm 容差到位
    /* 目标 0.3 m/s 接近, 在 CM 模式下传给 Cmd_Move_Vel 要乘 100 转 cm/s */
    float scale = 0.3f / dist;                 // 期望 0.3 m/s 接近
    Cmd_Move_Vel(dx * scale * 100.0f, dy * scale * 100.0f, 0);
    delay_ms(100);
}
Cmd_Move_Vel(0, 0, 0);                         // 主动停车
```

## 常见问题

**Q: 终端打开了，看不到任何输出**
- 检查 USART2 (PA2/PA3) 接线对不对
- 检查 USB-TTL 波特率是不是 115200
- 检查 Keil 是不是勾了 Use MicroLIB（printf 不行就是这个问题）

**Q: 终端有 `[INIT] Subscribe ODOM` 但没有 `[ODOM]` 数据**
- 检查 USART1 (PA9/PA10) 接小车的线
- 检查小车有没有上电、有没有激活（未激活的话 OLED 显示"未激活"，需要先用上位机激活）

**Q: 看到 `[ODOM] (no data yet...)` 一直循环**
- 一样是 USART1 接线问题，或者小车没在跑

**Q: 小车不动 / 动作不对**
- 检查小车 USART1 默认是不是 460800（出厂默认是）
- 检查 `Cmd_Move_*` 的参数单位：默认 CM 模式 = `cm/cm-per-s/deg/deg-per-s`，
  切到 SI 模式才是 `m/m-per-s/rad/rad-per-s`
- 检查 px/py 顺序是不是搞反了：**新协议 +X 前 / +Y 左**，跟老版本对调

**Q: 数值很小，传 `0.5` 小车不动**
- 默认是 CM 模式 (`g_dfcom_unit_mode = DFCOM_UNIT_CM`)，`0.5` 被当作 0.5 cm = 5 mm 走完就停了，看着像没动
- 解决方法 1: 数值改 `50`（50 cm）
- 解决方法 2: 切到 SI: `g_dfcom_unit_mode = DFCOM_UNIT_M;` 然后 `0.5` 才是 0.5 m

**Q: 从老例程迁移过来，小车动作全错（前后/左右搞反, 旋转方向反）**
- 这是预期的——新协议 ROS REP-103 跟老协议坐标系完全相反。
- 老 `(px=0, py=0.5)` 前进 → 新写法 `(px=0.5, py=0)` 才是前进。
- 老 `Cmd_Move_Rot(.., +90°)` 右转 → 新 `Cmd_Move_Rot(+90, 60)` 是**左转**（CCW）。

## 协议细节（想改库 / 学协议看）

每个发送指令、每个回传帧的**完整字节布局**都画在源码注释里了：

| 想看什么 | 看哪里 |
|---|---|
| 5 条 Cmd_Move_* 的帧布局 + 字段偏移 + 缩放 | `HARDWARE/DFCom_Tx.c` 每个函数前面的大块注释 |
| 订阅 / 状态查询的协议含义 | `HARDWARE/DFCom_Tx.c::Cmd_Subscribe_Odom / Cmd_Subscribe_VelPos / Cmd_Query_DcarState` |
| Odom v4 payload 布局 | `HARDWARE/DFCom_Rx.c::parse_odom_v4` 顶部注释 |
| ProgressBack / DcarState 解析 | `HARDWARE/DFCom_Rx.c::parse_move_progress / parse_dcar_state` |
| 帧定界 + 校验和 + 分发逻辑 | `HARDWARE/DFCom_Rx.c::DFCom_RxParse` |
| 帧总格式 / 地址校验 | `HARDWARE/DFCom_Tx.c` 顶部、`DFCom_Rx.c` 顶部 |

简要速记：

```
发送：[0xDF][ROBOT_ID=0x01][PC_ID=0x97][A][B][LEN][payload][0xFD][sum_lo][sum_hi]
回传：[0xDF][Target_ID=0x97][ROBOT_ID=0x01][A][B][LEN][payload][0xFD][sum_lo][sum_hi]
校验和：从帧头到 tail 全部累加（u16 LE）
```

**5 个运动指令 cmd 码（class A=0x02）**：

| cmd B | 名 | LEN | payload 字段 |
|---|---|---|---|
| 0x62 | Motion_Velocity     | 12 | Vx + Vy + Vz_rad_s (3 × s32 ×10000) |
| 0x63 | Motion_Rotate       |  8 | dYaw_rad + omega_max_rad_s (2 × s32 ×10000) |
| 0x64 | Motion_Linear       | 12 | Px + Py + Speed (3 × s32 ×10000) |
| 0x65 | Motion_LinearWithYaw| 16 | Px + Py + dYaw_rad + Speed (4 × s32 ×10000) |
| 0x66 | Motion_Arc          | 12 | R + dYaw_rad + Speed (3 × s32 ×10000) |

完整 A/B 对照表见 `DFCom.h` 顶部宏定义。

## 启动诊断（上电就能看到）

例程一启动会打印一段"自检报告"：

```
============================================
  DcarON DFCom Client - STM32F103ZET6
  Protocol: SI + ROS REP-103 (+X fwd, +Y left)
  https://differ-tech.pages.dev/
============================================
[INIT] USART1 = 460800 (to DcarON)
[INIT] USART2 = 115200 (this terminal)
[INIT] Subscribe ODOM @ 10Hz...
[INIT] Query DcarState...
[INIT] DcarState: imu_calibrated=1, finetune=1, par1=1.00 par2=1.00
[INIT] Waiting 1500ms to check ODOM flow...
[INIT] ODOM data flowing ✓ (got 15 frames in 1.5s)
[INIT] ✓ All OK, sending commands...
```

**学生看终端就能立刻知道**：
- USART2 通了吗（看到上面那几行 [INIT] 输出 = 通了）
- USART1 通了吗（看到 `ODOM data flowing ✓` = 通了）
- 小车激活/校准了吗（看到 `imu_calibrated=1` = 激活了）

任何一项 ✗ 都会同时打印"可能原因"清单帮排查。

---

更多文档、上位机软件、视频教程请访问 <https://differ-tech.pages.dev/>。
