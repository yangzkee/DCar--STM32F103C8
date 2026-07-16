# DFLink V3 协议手册（对外版）

- 协议族版本: DFLink V3
- 文档修订: V13-PUBLIC
- 生成日期: 2026-07-16
- 适用对象: 客户端开发、上位机联调、测试与集成同事
- 内容范围: 通用帧、数据编码、控制/校准、运动控制、运行数据查询及对应回传

> 本文件由内部主手册自动生成，作为可直接发送给协作同事的对外版本；协议维护仍以 `DFLink_V3协议手册.md` 为唯一主源。

## 1. 总体设计

DFLink V3 是统一串口帧协议，同一套帧格式承载控制/校准指令、运动控制、运行数据查询及对应状态回传；其中 `A` 表示大类，`B` 表示子命令，`LEN` 表示数据区长度，`payload` 表示数据区内容。协议本身只规定帧组织、字段含义、命令作用和回传格式。

## 2. 地址、方向与通用帧

### 2.1 默认地址

默认地址: 上位机 `0x97` - 战车1 `0x01` - 战车2 `0x02` - 战车3 `0x03` - 战车4 `0x04` - 战车5 `0x05` - 战车6 `0x06` - 云台 `0x68` - 灯带 `0x71`

### 2.2 地址语义

`[1] target_id` 表示目标设备地址，`[2] source_id` 表示发送方地址；例如 `DF 01 97 ...` 表示上位机 `0x97` 发给战车 `0x01`。

### 2.3 通用帧格式与字段说明

<table>
  <tbody>
    <tr>
      <td>字节位</td>
      <td><code>[0]</code></td>
      <td><code>[1]</code></td>
      <td><code>[2]</code></td>
      <td><code>[3]</code></td>
      <td><code>[4]</code></td>
      <td><code>[5]</code></td>
      <td><code>[6..6+N-1]</code></td>
      <td><code>[6+N]</code></td>
      <td><code>[7+N]</code></td>
      <td><code>[8+N]</code></td>
    </tr>
    <tr>
      <td>字段</td>
      <td><code>0xDF</code></td>
      <td><code>target_id</code></td>
      <td><code>source_id</code></td>
      <td><code>A</code></td>
      <td><code>B</code></td>
      <td><code>LEN</code></td>
      <td><code>payload</code></td>
      <td><code>0xFD</code></td>
      <td><code>sumL</code></td>
      <td><code>sumH</code></td>
    </tr>
  </tbody>
</table>

其中 `payload` 为 `N` 字节数据区，`LEN = N`；`0xDF` 为帧头，`0xFD` 为帧尾，`sumL / sumH` 为 16 位校验和低字节在前、高字节在后。

### 2.4 校验和

校验方式为 16 位累加和，累加范围从帧头 `0xDF` 开始，到帧尾 `0xFD` 结束，不包含最后两个校验字节本身。发送时低字节在前，高字节在后。

```c
sum = 0;
for (i = 0; i < LEN + 7; ++i) {
    sum += frame[i];
}
checksum_low  = sum & 0xFF;
checksum_high = (sum >> 8) & 0xFF;
```

## 3. 数据类型与编码

协议表建议统一直接写 `U8 / S8 / U16 / S16 / U32 / F16 / F32`，不要混用“浮点”“百分制小数”“两位小数 short”这类描述。

为什么要做这一步编码:
- 串口线上传的是字节，不是“100.03”这种业务语义。
- 固定长度后，接收端一看到类型就知道应该读几个字节。
- `U16 / U32` 直接按整数原值发送，不做缩放。
- `F16 / F32` 采用定点数，避免不同语言或平台对浮点格式、精度和大小端处理不一致。
- 当前协议里的 `F32` 不是 IEEE 754 单精度浮点，而是“有符号整数按比例缩放”；不要和 `U32` 混看。
- 当前仓库统一按小端序发送，即低字节在前、高字节在后。

### 3.1 基础类型

| 类型名 | 实际存储 | 字节数 | 小端序 | 协议值域 | 说明 |
|---|---|---:|---|---|---|
| `U8` | `uint8_t` | 1 | 不涉及 | `0 ~ 255` | 无符号单字节 |
| `S8` | `int8_t` | 1 | 不涉及 | `-128 ~ 127` | 有符号单字节 |
| `U16` | `uint16_t` | 2 | 是 | `0 ~ 65535` | 无符号两字节 |
| `S16` | `int16_t` | 2 | 是 | `-32768 ~ 32767` | 有符号两字节 |
| `U32` | `uint32_t` | 4 | 是 | `0 ~ 4294967295` | 无符号四字节，原值直传 |
| `F16` | `int16_t / 100` | 2 | 是 | `-327.68 ~ 327.67` | 旧文档常写 `Float16` |
| `F32` | `int32_t / 10000` | 4 | 是 | `-214748.3648 ~ 214748.3647` | 旧文档常写 `Float32` |

### 3.2 先看一个统一示例

| 类型 | 业务值 | 编码动作 | 原始整数 | 十六进制 | 线上字节顺序 | 说明 |
|---|---:|---|---:|---|---|---|
| `U8` | `6` | 不缩放 | `6` | `0x06` | `06` | 单字节直传 |
| `S16` | `10003` | 不缩放 | `10003` | `0x2713` | `13 27` | 作为整数理解 |
| `U32` | `3000` | 不缩放 | `3000` | `0x00000BB8` | `B8 0B 00 00` | 原值直传，常用于时间戳或累计计数 |
| `F16` | `100.03` | `×100` | `10003` | `0x2713` | `13 27` | 和上面字节相同，但要再除以 `100` |
| `F16` | `100.05` | `×100` | `10005` | `0x2715` | `15 27` | 相比 `100.03` 只改低字节 |
| `F32` | `0.3000` | `×10000` | `3000` | `0x00000BB8` | `B8 0B 00 00` | 和上面 `U32=3000` 字节完全相同，但业务含义不同 |
| `F32` | `111.111` | `×10000` | `1111110` | `0x0010F446` | `46 F4 10 00` | 更高精度量常用 |

要点:
- 同样的字节 `13 27`，写成 `S16` 表示 `10003`，写成 `F16` 才表示 `100.03`。
- 同样的字节 `B8 0B 00 00`，写成 `U32` 表示 `3000`，写成 `F32` 才表示 `0.3000`。
- 发送时始终先发低 8 位，再发高 8 位；接收时按同样顺序拼回原始整数。

### 3.3 `F16` 的编码方式

`F16` 不是 IEEE 半精度浮点，而是“有符号 16 位整数，再除以 100”。它本质上是一个带 2 位小数的定点数。

| 实际值 | 原始整数 | 十六进制 | 字节序列 |
|---:|---:|---:|---|
| `5.00` | `500` | `0x01F4` | `F4 01` |
| `-1.25` | `-125` | `0xFF83` | `83 FF` |

例如上位机要发送 `5.00`，程序内部先转成整数 `500`，再按小端序发为 `F4 01`；接收端再按 `500 / 100 = 5.00` 还原。

### 3.4 `F32` 的编码方式与和 `U32` 的区别

`F32` 也不是 IEEE 754 单精度浮点，而是“有符号 32 位整数，再除以 10000”。它适合表示位置、角度、距离这类需要 4 位小数精度的数据。

不要把 `F32` 和 `U32` 混看：
- `U32` = 无符号整数原值直传，不缩放。
- `F32` = 有符号整数先 `×10000` 再发送，接收后 `/10000`。
- 两者都占 4 字节、都走小端序，但业务解释完全不同。

| 实际值 | 原始整数 | 十六进制 | 字节序列 |
|---:|---:|---:|---|
| `0.3000` | `3000` | `0x00000BB8` | `B8 0B 00 00` |

例如上位机要发送位置 `0.3000`，程序内部先转成整数 `3000`，再按小端序发为 `B8 0B 00 00`；接收端再按 `3000 / 10000 = 0.3000` 还原。

### 3.5 发送端 / 接收端实际怎么操作

发送侧伪代码:

```c
// 小端序：低字节在前
u8_val = 6;
payload[0] = u8_val;

u32_raw = 3000;
payload[0] = u32_raw & 0xFF;         // 0xB8
payload[1] = (u32_raw >> 8) & 0xFF;  // 0x0B
payload[2] = (u32_raw >> 16) & 0xFF; // 0x00
payload[3] = (u32_raw >> 24) & 0xFF; // 0x00

f16_raw = (int16_t)(100.03 * 100);   // 10003
payload[0] = f16_raw & 0xFF;         // 0x13
payload[1] = (f16_raw >> 8) & 0xFF;  // 0x27

f32_raw = (int32_t)(111.111 * 10000); // 1111110
payload[0] = f32_raw & 0xFF;          // 0x46
payload[1] = (f32_raw >> 8) & 0xFF;   // 0xF4
payload[2] = (f32_raw >> 16) & 0xFF;  // 0x10
payload[3] = (f32_raw >> 24) & 0xFF;  // 0x00
```

接收侧伪代码:

```c
u32_raw = (payload[3] << 24) | (payload[2] << 16)
        | (payload[1] << 8)  | payload[0];
u32_val = (uint32_t)u32_raw;

s16_raw = (payload[1] << 8) | payload[0];
f16_val = (int16_t)s16_raw / 100.0f;

s32_raw = (payload[3] << 24) | (payload[2] << 16)
        | (payload[1] << 8)  | payload[0];
f32_val = (int32_t)s32_raw / 10000.0f;
```

当前代码对应关系:
- 发送侧大量使用 `BYTE0 / BYTE1 / BYTE2 / BYTE3` 逐字节发包。
- 接收侧使用 `Float16_DateAssmeble(ReadFromUsart[7], ReadFromUsart[6])`、`Float32_DateAssmeble(ReadFromUsart[9], ReadFromUsart[8], ReadFromUsart[7], ReadFromUsart[6])` 拼回原始整数。

## 4. A = 0x01 控制类

**蜂鸣器闭环示例**
下面只用蜂鸣器控制举例；其它控制类命令读法一样，主要变化的是 `B / LEN / payload`。

| 项目 | 发送帧 | 回传帧 | 说明 |
|---|---|---|---|
| 通用格式 | `[0xDF][Target_ID][Robot_ID][0x01][0x28][0x01][Date1][0xFD][sumL][sumH]` | `[0xDF][0x97][0x01][0x46][0x28][0x02][Process][Notice][0xFD][sumL][sumH]` | 发送是 `A=0x01`，回传是 `A=0x46`，`B=0x28` 保持不变 |
| 重点字段 | `A=0x01` `B=0x28` `LEN=0x01` `payload=Date1` | `A=0x46` `B=0x28` `LEN=0x02` `payload=Process+Notice` | `Process` 表示状态/进度，`Notice` 表示结果/提示 |

回传示例:

```text
DF 97 01 46 28 02 FF 00 FD E3 03
```

**控制类功能表**
本卡片列出 `A=0x01` 已整理出的控制类命令、`B` 值、长度和 payload 字段对应关系。

<table>
  <thead>
    <tr>
      <th>功能</th>
      <th>A</th>
      <th>B</th>
      <th>LEN长度</th>
      <th>字段1</th>
      <th>字段2</th>
      <th>字段3</th>
      <th>字段4</th>
      <th>说明</th>
    </tr>
  </thead>
  <tbody>
    <tr><td>蜂鸣器控制<br><code>Beep_Control_F</code></td><td>A<br><code>0x01</code></td><td>B<br><code>0x28</code></td><td>LEN长度<br><code>1</code></td><td>Date1<br><code>U8 / 业务值</code></td><td></td><td></td><td></td><td>0=关,1=开,11~13短叫1~3声,21~23鸣叫300/600/900ms</td></tr>
    <tr><td>IMU校准<br><code>IMU_Cali_F</code></td><td>A<br><code>0x01</code></td><td>B<br><code>0x3C</code></td><td>LEN长度<br><code>0</code></td><td></td><td></td><td></td><td></td><td>校准前离地静止；旧表常见u8=1</td></tr>
    <tr><td>锁头模式<br><code>HeadingLock_F</code></td><td>A<br><code>0x01</code></td><td>B<br><code>0x4F</code></td><td>LEN长度<br><code>1</code></td><td>Enable<br><code>U8 / 0或1</code></td><td></td><td></td><td></td><td>默认1；建议上电阶段设置</td></tr>
  </tbody>
</table>

## 5. A = 0x02 运动控制类

本节以 `DCAR_Classic/客户端例程_DFCom_v2` 和当前 DcarON 固件共同使用的 Classic SI 运动协议为准。旧协议中 `0x62=无头位移`、`0x63=圆弧`、`0x66=旋转`、`0x67=速度` 的定义已经废弃，不能与当前协议混用。

### 5.1 坐标系、单位与编码

- 坐标系使用 ROS REP-103 FLU：`+X=前`、`+Y=左`、`+Z=上`、`+Yaw=顶视逆时针 CCW`。
- 协议线上全部使用 SI：位置 `m`、线速度 `m/s`、角度 `rad`、角速度 `rad/s`。
- 所有运动数值字段均为 `S32` 小端定点数，编码为 `physical_value × 10000`；本文历史字段名 `F32(×10000)` 也表示这种定点 `S32`，不是 IEEE-754 float。
- `DCAR_Classic` 客户端可在函数入口提供 `cm/deg` 或 `mm/deg` 便捷模式，但发送前必须换算成上述 SI 线上格式。
- `Px/Py/dyaw` 均为相对本条命令起点的目标增量，不是世界坐标绝对目标。

### 5.2 当前对外运动命令

| 功能 | A | B | LEN | 字段1 | 字段2 | 字段3 | 字段4 | 字段5 | 说明 |
|---|---|---|---:|---|---|---|---|---|---|
| 持续速度 `Motion_Velocity` | `0x02` | `0x62` | `12` | Vx<br>`F32`<br>`m/s` | Vy<br>`F32`<br>`m/s` | Vz<br>`F32`<br>`rad/s` |  |  | `Vx/Vy` 为车体 X/Y 线速度，单位 `m/s`；`Vz` 为 Yaw 角速度，单位 `rad/s`；发送 `(0,0,0)` 主动停车 |
| 原地旋转 `Motion_Rotate` | `0x02` | `0x63` | `8` | dyaw<br>`F32`<br>`rad` | omega_max<br>`F32`<br>`rad/s` |  |  |  | `dyaw` 为相对转角，单位 `rad`，正值左转、负值右转；`omega_max` 为最大角速度，单位 `rad/s`，取正数 |
| 直线位移 `Motion_Linear` | `0x02` | `0x64` | `12/13` | Px<br>`F32`<br>`m` | Py<br>`F32`<br>`m` | Speed<br>`F32`<br>`m/s` | ProfileByte<br>`U8`<br>可选 |  | `Px/Py` 为相对位移，单位 `m`；`Speed` 为线速度，单位 `m/s` 且应大于 0；`ProfileByte` 为可选轨迹档位；差速底盘会忽略不可达的 `Py` |
| 平移并转向 `Motion_LinearWithYaw` | `0x02` | `0x65` | `16/17` | Px<br>`F32`<br>`m` | Py<br>`F32`<br>`m` | dyaw<br>`F32`<br>`rad` | Speed<br>`F32`<br>`m/s` | ProfileByte<br>`U8`<br>可选 | `Px/Py` 为相对位移，`dyaw` 为相对转角，`Speed` 为线速度；`ProfileByte` 可选，但当前固定按 `CONST` 执行 |
| 圆弧运动 `Motion_Arc` | `0x02` | `0x66` | `12/13` | Radius<br>`F32`<br>`m` | dyaw<br>`F32`<br>`rad` | Speed<br>`F32`<br>`m/s` | ProfileByte<br>`U8`<br>可选 |  | `Radius` 为绝对半径，单位 `m`；`dyaw` 为相对转角，正值左转、负值右转；`Speed` 为线速度；`ProfileByte` 可选但当前固定按 `CONST` 执行 |
| 十字标录入 `Cross_Record` | `0x02` | `0x80` | `1` | Mode<br>`U8` |  |  |  |  | `0=FIRST`、`1=NEXT`；两种模式都立即执行 FULL 对齐，成功后返回锁定瞬间连续 Odom 位姿 |
| 十字标定位 `Cross_Localize` | `0x02` | `0x81` | `1` | Mode<br>`U8` |  |  |  |  | `FIRST` 立即 FULL；`NEXT` 一次性武装紧随其后的 `0x64/0x65` 末段捕获 |

`ProfileByte` 定义：

| 值 | 名称 | 当前效果 |
|---:|---|---|
| `0` | `CONST` | 匀速 |
| `1` | `TRAPEZOID` | 梯形加减速；短帧省略 Profile 时默认取 `1` |
| `2` | `DIST_LOOP` | 高精度里程计距离闭环 |

目前只有 `Motion_Linear (0x64)` 实际开放 Profile 选择；`0x65/0x66` 为兼容 Classic 客户端仍接收可选字节，但当前执行层会忽略它并强制 `CONST`。`0x67` 已释放；`0x68~0x6C` 是历史低层控制槽位，不属于当前 `DCAR_Classic` 对外用户运动接口，不应在新客户端中使用。

### 5.3 十字标 Record / Localize

这两个接口只负责“捕获、锁定、返回连续 Odom 位姿”。MCU 不接收 Marker ID，不保存十字架列表，不计算全局地图，也不隐式清零 Odom。外部处理器按自己的有序数组维护 Marker ID、十字架顺序、相邻坐标变换和 `map↔odom` 修正。

请求格式：

```text
[0xDF][Robot_ID][Host_ID][0x02][0x80或0x81][0x01][Mode][0xFD][sumL][sumH]
```

| API | Mode | MCU 行为 |
|---|---:|---|
| `Cross_Record` | `FIRST=0` | 立即执行 FULL，对齐成功后返回第一个十字标的连续 Odom 位姿 |
| `Cross_Record` | `NEXT=1` | 立即执行 FULL，返回下一个十字标位姿；序号由外部数组顺序决定 |
| `Cross_Localize` | `FIRST=0` | 开机位于首个十字标捕获范围时立即执行 FULL |
| `Cross_Localize` | `NEXT=1` | 一次性武装；3 秒内线上下一条控制帧必须正好是 `Motion_Linear(0x64)` 或 `Motion_LinearWithYaw(0x65)` |

`Localize(NEXT)` 绑定的是 DFLink 接收顺序，不是“稍后任意一条直线”。绑定运动进入终点前 `0.35 m` 后，MCU 开始观察新的光电边沿；命中后按 `350 ms` 半余弦速度曲线柔停并接管，随后自主执行 FULL。若没有检测到边沿，则在该运动的里程计终点兜底进入 FULL。整个组合任务只以 `B=0x81` 返回最终终止帧，不提前发送普通 `0x64/0x65` 的自然完成帧。

成功回包固定为：

```text
[0xDF][Target_ID][Robot_ID][0x6F][原命令0x80/0x81][0x13]
[0xFF][0x00][Mode][X S32LE×10000][Y][Z][Yaw]
[0xFD][sumL][sumH]
```

| payload 偏移 | 字段 | 类型 | 单位 / 含义 |
|---:|---|---|---|
| `0` | Process | `U8` | `0xFF`，本次任务终止 |
| `1` | Notice | `U8` | `0x00`，成功 |
| `2` | Mode | `U8` | 原请求的 `FIRST/NEXT` |
| `3..6` | X | `F32` | 锁定瞬间连续 Odom 世界系 X，m |
| `7..10` | Y | `F32` | 锁定瞬间连续 Odom 世界系 Y，m |
| `11..14` | Z | `F32` | 平面小车固定为 `0 m` |
| `15..18` | Yaw | `F32` | 锁定瞬间连续累计 Yaw，rad，CCW+，不回绕 |

失败或打断仍使用普通 `LEN=2` 终止帧 `[FF][Notice]`，没有有效 pose：

| Notice | 含义 | 外部处理 |
|---:|---|---|
| `0x01` | 参数、忙状态、3 秒超时或 NEXT 紧邻时序不合法 | 不保存 pose，修正调用顺序后重试 |
| `0x02` | 有界搜索内未找到完整十字标 | 回到捕获范围后重试 |
| `0x03` | 找到标线但 FULL 未在精度/超时约束内收敛 | 检查光电、摆放和障碍 |
| `0x0A` | 遥控器接管 | 按人工接管处理 |
| `0x62~0x66` | 被对应的新标准运动命令打断 | 停止等待旧十字任务 |
| `0x80/0x81` | 被新的 Record / Localize 请求打断 | 以新请求为准 |

外部只有在同时满足 `A=0x6F`、`B=0x80/0x81`、`LEN=19`、`Process=0xFF`、`Notice=0x00` 时，才能使用和保存 `X/Y/Z/Yaw`。

### 5.4 运动进度与完成回传

有明确终点的 `0x63~0x66` 使用以下回传：

```text
[0xDF][Target_ID][Robot_ID][0x6F][原命令B][0x02][Process][Notice][0xFD][sumL][sumH]
```

- `B` 直接复用原命令号：旋转 `0x63`、直线 `0x64`、平移并转向 `0x65`、圆弧 `0x66`。
- `Process=0x01~0xFE` 表示执行中，进度百分比为 `(Process-1)/254×100%`。
- `Process=0xFF, Notice=0x00` 表示自然完成。
- `Process=0xFF, Notice=0x01` 表示版本门控或参数非法，任务未启动。
- `Process=0xFF, Notice=0x0A` 表示被遥控接管打断。
- `Process=0xFF, Notice=新命令B` 表示被新的 DFLink 运动指令打断。
- 每个运动任务只发送一次终止帧；不能再用后台任务补发 `FF/FF`，否则可能覆盖客户端已经收到的自然完成状态。
- `Motion_Rotate (0x63)` 第一次进入 yaw 误差 `±3°` 后固定等待 `300 ms`，随后回传 `Process=0xFF, Notice=0x00`；窗口触发后不重置计时，等待期间角度闭环继续精修。
- `Motion_LinearWithYaw (0x65)` 的旋转部分采用相同的 `±3° + 300 ms` 完成判定。
- `Motion_Velocity (0x62)` 没有自然终点；客户端不能用 `WaitMoveDone` 等它完成，应主动发送零速度停车。即使收到即时状态 ACK，也不能把它理解成车辆已经停止或到点。

直线位移执行到约 `50%` 时的回传示例：

```text
DF 97 01 6F 64 02 80 00 FD C9 03
```

### 5.5 常用示例帧

持续速度：`Vx=0.30m/s, Vy=0, Vz=0`

```text
DF 01 97 02 62 0C B8 0B 00 00 00 00 00 00 00 00 00 00 FD A7 03
```

原地左转：`dyaw=π/2rad, omega_max=1.0rad/s`

```text
DF 01 97 02 63 08 5C 3D 00 00 10 27 00 00 FD B1 03
```

直线前进：`Px=0.50m, Py=0, Speed=0.30m/s, Profile=TRAPEZOID`

```text
DF 01 97 02 64 0D 88 13 00 00 00 00 00 00 B8 0B 00 00 01 FD 46 04
```

前进并左转：`Px=0.50m, Py=0, dyaw=π/2rad, Speed=0.30m/s, Profile=TRAPEZOID`

```text
DF 01 97 02 65 11 88 13 00 00 00 00 00 00 5C 3D 00 00 B8 0B 00 00 01 FD E4 04
```

左转圆弧：`Radius=0.30m, dyaw=π/2rad, Speed=0.20m/s, Profile=TRAPEZOID`

```text
DF 01 97 02 66 0D B8 0B 00 00 5C 3D 00 00 D0 07 00 00 01 FD 1D 05
```

## 6. A = 0x04 运行数据访问

**访问规则说明**
本卡片说明 `A=0x04` 访问请求怎么发、`VisitType / FreqCode` 分别代表什么，以及访问项与常见回传之间的关系。这里主表只写“请求帧本身”的字段；`A=0x6C` 回传只在表后单独说明，避免把回传对照误看成请求 payload 里的第三个字段。

本类用于请求本机回传某一类运行数据。对外手册只保留电池、欧拉角、Odom 和 VelPos 四项访问；这些访问请求都使用统一的 2 字节 payload:

| 字段 | 类型 | 值域 | 说明 |
|---|---|---|---|
| `VisitType` | `U8` | `0x01 / 0x02` | `0x01=开始连续访问`, `0x02=单次访问 / 停止之前的连续访问` |
| `FreqCode` | `U8` | `0x01/0x05/0x0A/0x14/0x19/0x32/0x64` | 请求字节值，不是线性递增 Hz |

说明: 从本版开始，协议里的 `A / B / 地址 / 枚举字节值` 统一按十六进制书写；例如 `FreqCode=0x0A` 表示字节值 `0x0A`，对应 `100 Hz`，不再写成十进制 `10` 让读者自己换算。这些值来自当前仓库 `FD_Visit.h / FD_Visit.c` 的请求口径，不是 `1Hz / 2Hz / 3Hz` 这种线性编码。

停止规则: 如果某一项已经发过 `VisitType=0x01` 的连续访问，例如已经按 `10 Hz / 100 Hz` 持续回传了，想关闭时，对同一 `B` 再发一次 `VisitType=0x02` 的单次访问即可。可以把它理解为“再发一次单次访问帧，把之前的连续访问顶掉并关闭”。

频率码定义:

| 频率码 | 含义 |
|---:|---|
| `0x01` | 10 Hz |
| `0x05` | 50 Hz |
| `0x0A` | 100 Hz |
| `0x14` | 200 Hz |
| `0x19` | 250 Hz |
| `0x32` | 500 Hz |
| `0x64` | 1000 Hz |

示例: 欧拉角访问连续开始 / 停止

开始连续访问 (`VisitType=0x01`, `FreqCode=0x0A`，即 `100 Hz`；设备随后会按 `100 Hz` 持续回传)

```text
DF 01 97 04 6A 02 01 0A FD EF 02
```

关闭连续访问（对同一 `B=0x6A` 再发一次单次访问，`VisitType=0x02`）

```text
DF 01 97 04 6A 02 02 0A FD F0 02
```

<br>

**访问命令表**
本卡片列出 `A=0x04` 各访问项的 `B` 值与请求参数；下面表格中的“数据段”只有 `VisitType + FreqCode` 两个字节，没有第三个隐藏字段。

<table>
  <thead>
    <tr>
      <th>功能</th>
      <th>A</th>
      <th>B</th>
      <th>LEN长度</th>
      <th>字段1</th>
      <th>字段2</th>
      <th>说明</th>
    </tr>
  </thead>
  <tbody>
    <tr><td>电池访问<br><code>Battry_V</code></td><td>A<br><code>0x04</code></td><td>B<br><code>0x64</code></td><td>LEN长度<br><code>2</code></td><td>VisitType<br><code>U8 / 0x01或0x02</code></td><td>FreqCode<br><code>U8 / 频率码字节</code></td><td>电池相关；回传映射见表后冲突说明</td></tr>
    <tr><td>欧拉角访问<br><code>EuraAngle_V</code></td><td>A<br><code>0x04</code></td><td>B<br><code>0x6A</code></td><td>LEN长度<br><code>2</code></td><td>VisitType<br><code>U8 / 0x01或0x02</code></td><td>FreqCode<br><code>U8 / 频率码字节</code></td><td>欧拉角</td></tr>
    <tr><td>全量里程计访问<br><code>Odom_V</code> / <code>Time_V</code><br>（旧 Time_V 扩展）</td><td>A<br><code>0x04</code></td><td>B<br><code>0x80</code></td><td>LEN长度<br><code>2</code></td><td>VisitType<br><code>U8 / 0x00 / 0x01 / 0x02</code></td><td>FreqCode<br><code>U8 / 频率码字节</code></td><td>全量里程计 Odom (Pro 级)：包含姿态 + 原始 IMU + 机体/世界速度 + 世界位置 + 时间戳</td></tr>
    <tr><td>速度位置访问<br><code>VelPos_V</code></td><td>A<br><code>0x04</code></td><td>B<br><code>0x81</code></td><td>LEN长度<br><code>2</code></td><td>VisitType<br><code>U8 / 0x00 / 0x01 / 0x02</code></td><td>FreqCode<br><code>U8 / 频率码字节</code></td><td>速度+位置简化包 VelPos (标准级)：Yaw + 机体/世界速度 + 世界位置 + 时间戳</td></tr>
  </tbody>
</table>

### 6.0 VisitType 含义（v2 扩展）

| 值 | 名称 | 含义 |
|---|---|---|
| `0x00` | **Stop / 显式退订**（v2 新增） | 通道立即停发，`Visit_All.X.Visit_Type` 置 0 |
| `0x01` | **Continuous / 持续订阅** | 车端按 FreqCode 持续推送 |
| `0x02` | **Once / 一次性** | 车端发 1 帧后自动置 0 停发 — 兼容旧版"退订"写法 |
| 其它 | 协议错误 | 车端忽略，不修改订阅状态 |

> **历史兼容**：旧固件不识别 `0x00`，会落入 else 分支后被丢弃（无副作用）。新固件同时接受 `0x00` 显式退订和 `0x02` 一次性退订，向后兼容。

### 6.1 回传对照说明

`A=0x04` 是访问请求，不是回传。当前对外保留项中，欧拉角、Odom、VelPos 都使用 `A=0x6C` 且保持同一个 `B`；电池项是历史例外。

常见稳定对照:
- `B=0x6A` 欧拉角访问，常见回传为 `A=0x6C, B=0x6A`
- `B=0x80` 全量里程计 Odom 访问（v2 扩展自旧 Time_V），常见回传为 `A=0x6C, B=0x80`
- `B=0x81` 速度+位置简化包 VelPos 访问，常见回传为 `A=0x6C, B=0x81`

电池项冲突说明:
- 当前 Odom v4 (`0x80`) 和 VelPos v2 (`0x81`) 都不包含电池字段，因此对外手册继续保留 `Battry_V (B=0x64)` 这一项。
- 当前仓库里 `Battry_V (B=0x64)` 的发送端实现没有呈现稳定独立回传码。
- 现有代码中 `DF_Alarm.c` 的电池发送与 `DF_CoreState.c` 的 `N_Pos` 发送都出现了 `A=0x6C, B=0x73`。
- 这说明仓库历史实现里存在编码冲突；因此本手册不再把电池回传写成主表里的稳定映射，避免误导联调。

## 7. 回传合集与索引关系

对外版本保留三条主闭环；发送和回传优先使用同一个 `B` 对齐业务:

| 发送 A | 发送大类 | 回传 A | 回传大类 | 关系说明 |
|---|---|---|---|---|
| `0x01` | 控制 / 校准 | `0x46` | 控制类回传 | `B` 继承原控制功能码 |
| `0x02` | 运动控制 | `0x6F` | 运动控制类回传 | `B` 继承原运动命令号 |
| `0x04` | 运行数据访问 | `0x6C` | 运行数据访问回传 | 查询和回传使用同一数据项 `B` |

### 7.1 A = 0x46 控制类回传

本类对应 `A=0x01` 控制类，主要用于校准进度、模式状态和简单控制状态回传。

通用格式:

```text
[0xDF][Target_ID][Robot_ID][0x46][B][0x02][Process][Notice][0xFD][sumL][sumH]
```

<table>
  <thead>
    <tr>
      <th>回传项</th>
      <th>A</th>
      <th>B</th>
      <th>LEN长度</th>
      <th>字段1</th>
      <th>字段2</th>
      <th>说明</th>
    </tr>
  </thead>
  <tbody>
    <tr><td rowspan="2">控制类回传通用格式</td><td>A</td><td>B</td><td>LEN长度</td><td>Process</td><td>Notice</td><td rowspan="2">`B` 对应具体控制功能码；对外保留 `0x28 / 0x3C / 0x4F`</td></tr>
    <tr><td><code>0x46</code></td><td><code>功能码</code></td><td><code>2</code></td><td><code>U8</code></td><td><code>U8</code></td></tr>
  </tbody>
</table>

常见 `B`:

| B | 含义 |
|---|---|
| `0x28` | 蜂鸣器控制状态 |
| `0x3C` | IMU 校准状态 |
| `0x4F` | 锁头模式状态 |

### 7.2 A = 0x6C 运行数据访问回传

本类对应 `A=0x04` 运行数据访问；对外手册只保留欧拉角、Odom 和 VelPos 三种稳定回传。

| 功能 | A | B | LEN长度 | 字段1 | 字段2 | 字段3 | 字段4 | 说明 |
|---|---|---|---|---|---|---|---|---|
| 欧拉角回传 | `0x6C` | `0x6A` | `6` | `Roll = F16` | `Pitch = F16` | `Yaw = F16` | `—` | `F16 * 3 / deg` |
| **Odom v4 全量回传** | `0x6C` | `0x80` | **51** | `Ver(0x04) + 姿态3×S16(rad)` | `Acc3×S16 + Gyro3×S16(rad/s)` | `BVel3×S16 + NVel3×S16 + NPos3×S32` | `Time u32×2` | Pro 全量里程计帧（替代旧 Time_V） |
| **VelPos v2 简化回传** | `0x6C` | `0x81` | **35** | `Ver(0x02) + Yaw S16(rad)` | `BVel3×S16` | `NVel3×S16 + NPos3×S32` | `Time u32×2` | 标准速度+位置帧 |

#### Odom v4 (0x80) 字节布局

```
byte[6]      Version (= 0x04)
byte[7..12]  Roll/Pitch/Yaw   3×s16 / 10000   rad
byte[13..18] Acc_X/Y/Z        3×s16 / 400     m/s²
byte[19..24] Gyro_X/Y/Z       3×s16 / 1800    rad/s
byte[25..30] B_VelX/Y/Z       3×s16 / 5000    m/s
byte[31..36] N_VelX/Y/Z       3×s16 / 5000    m/s
byte[37..48] N_PosX/Y/Z       3×s32 / 1000    m
byte[49..56] Time_ms / us     2×u32           ms, μs
```

**Yaw 数据源**：来自 `g_state.single_yaw_rad`（由统一里程计 `g_odom` 桥接，内部连续累计，发送前 wrap 到 `[-π, π]`）。Roll/Pitch 来自 `g_state.roll_rad/pitch_rad`。

因此客户端收到的 `Odom.yaw` / `VelPos.yaw` 是当前航向角，不是多圈连续累计角；如需统计多圈旋转，客户端应对相邻帧在 `±π` 跳变处自行 unwrap。世界系位置 `N_PosX/Y/Z` 仍是累计位置，不受 Yaw wrap 影响。

#### VelPos v2 (0x81) 字节布局

```
byte[6]      Version (= 0x02)
byte[7..8]   Yaw                s16 / 10000    rad
byte[9..14]  B_VelX/Y/Z         3×s16 / 5000   m/s
byte[15..20] N_VelX/Y/Z         3×s16 / 5000   m/s
byte[21..32] N_PosX/Y/Z         3×s32 / 1000   m
byte[33..40] Time_ms / us       2×u32          ms, μs
```

#### Odom / VelPos 定点 Scale（DF_link.h）

| 宏 | 值 | 适用字段 | 量程 |
|---|---|---|---|
| `DFLINK_SCALE_ANGLE_RAD` | 10000 | Roll/Pitch/Yaw | ±3.276 rad |
| `DFLINK_SCALE_ACC_MPS2` | 400 | Acc_X/Y/Z | ±81.92 m/s² |
| `DFLINK_SCALE_GYRO_RAD` | 1800 | Gyro_X/Y/Z | ±18.20 rad/s |
| `DFLINK_SCALE_VEL_MPS` | 5000 | B_Vel*/N_Vel* | ±6.55 m/s |
| `DFLINK_SCALE_POS_M` | 1000 | N_Pos* | ±2147 km |

> 上下位机解析时**必须**共用同一份 scale 定义，否则会出现 s16/s32 错位 bug（历史教训：早期 N_Pos 上位机用了 s16/100，实际下位机发的是 s32/10000，导致位置数据始终接近 0）。

### 7.3 A = 0x6F 运动控制类回传

本类对应 `A=0x02` 运动控制类，用于回传运动任务的进度、完成状态和打断状态；可以把它理解成和发送运动命令左右对称的一组回传。

通用格式:

```text
[0xDF][Target_ID][Robot_ID][0x6F][B][0x02][Process][Notice][0xFD][sumL][sumH]
```

<table>
  <thead>
    <tr>
      <th>回传项</th>
      <th>A</th>
      <th>B</th>
      <th>LEN长度</th>
      <th>字段1</th>
      <th>字段2</th>
      <th>说明</th>
    </tr>
  </thead>
  <tbody>
    <tr><td rowspan="2">运动控制类回传通用格式</td><td>A</td><td>B</td><td>LEN长度</td><td>Process</td><td>Notice</td><td rowspan="2">`B` 直接复用运动命令号</td></tr>
    <tr><td><code>0x6F</code></td><td><code>0x62~0x66 / 0x80 / 0x81</code></td><td><code>2</code></td><td><code>U8</code></td><td><code>U8</code></td></tr>
  </tbody>
</table>

`B` 的含义:

| B | 对应运动 |
|---|---|
| `0x62` | 持续速度；没有自然到点语义，客户端不等待其完成 |
| `0x63` | 原地旋转 |
| `0x64` | 直线位移 |
| `0x65` | 平移并转向 |
| `0x66` | 圆弧运动 |
| `0x80` | 十字标录入；成功时改用 `LEN=19` 扩展位姿终止帧 |
| `0x81` | 十字标定位；成功时改用 `LEN=19` 扩展位姿终止帧 |

`Process / Notice` 取值:

| `Process` | `Notice` | 含义 |
|---|---|---|
| `0x01 ~ 0xFE` | `0` | 正在执行，`Process` 表示进度 |
| `0xFF` | `0` | 正常完成 |
| `0xFF` | `0x01` | 版本门控或参数非法，任务未启动 |
| `0xFF` | `0x02` | 十字标有界搜索内未找到完整标记 |
| `0xFF` | `0x03` | 十字标已发现，但 FULL 未收敛 |
| `0xFF` | `0x0A` | 被遥控接管打断 |
| `0xFF` | `0x62 ~ 0x66` | 被新的 DFLink 运动指令打断 |
| `0xFF` | `0x80 / 0x81` | 被新的十字标录入 / 定位请求打断 |

每个运动任务只发送一次终止帧；`0x62` 持续速度没有自然完成帧。十字标 `0x80/0x81` 只有成功时使用 `LEN=19`，字段布局见 5.3；失败和打断仍使用本节 `LEN=2` 格式。

进度换算:

```text
百分比 = (Process - 1) / 254 * 100%
```

## 8. 示例帧速查

持续速度：`Vx=0.30m/s`

```text
DF 01 97 02 62 0C B8 0B 00 00 00 00 00 00 00 00 00 00 FD A7 03
```

原地左转：`dyaw=π/2rad, omega_max=1.0rad/s`

```text
DF 01 97 02 63 08 5C 3D 00 00 10 27 00 00 FD B1 03
```

直线前进：`Px=0.50m, Speed=0.30m/s, Profile=1`

```text
DF 01 97 02 64 0D 88 13 00 00 00 00 00 00 B8 0B 00 00 01 FD 46 04
```

前进并左转：`Px=0.50m, dyaw=π/2rad, Speed=0.30m/s, Profile=1`

```text
DF 01 97 02 65 11 88 13 00 00 00 00 00 00 5C 3D 00 00 B8 0B 00 00 01 FD E4 04
```

圆弧运动：`Radius=0.30m, dyaw=π/2rad, Speed=0.20m/s, Profile=1`

```text
DF 01 97 02 66 0D B8 0B 00 00 5C 3D 00 00 D0 07 00 00 01 FD 1D 05
```

## 9. 使用说明

- 本文档只描述对外客户端联调需要的协议，不代表固件内部全部 DFLink 能力。
- 新增或调整协议时，先更新内部主手册，再重新生成本文件，避免两份定义分叉。
- A、B、LEN、字段类型、单位和小端序必须按本文档执行，不要从旧 Excel 或历史示例反推。
