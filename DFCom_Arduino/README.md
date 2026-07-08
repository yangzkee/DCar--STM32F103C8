# DcarON Arduino 客户端 (DFCom for Arduino)

把 DcarON 小车的 DFCom_v2 协议移植到 **Arduino Uno / Nano (ATmega328P)**，
对象式 API、学生友好单位（厘米 + 度），用你熟悉的 **Arduino IDE** 就能玩。

> 协议、坐标系、scale 与 STM32 版 DFCom_v2 **完全一致**，只是换了平台。

---

## 目录

```
DFCom_Arduino/
├── DFCom/                         ← Arduino 库 (给 IDE 用)
│   ├── library.properties
│   ├── src/DFCom.h  DFCom.cpp
│   └── examples/DcarON_Square/    ← 示例: 走方块
├── SingleFile/
│   └── DcarON_Square_AllInOne/    ← 单文件版 (零安装, 双击就传)
└── README.md
```

两种用法，按学生水平选其一：

### 用法 A — 库 (推荐, 干净可复用)
1. 把 `DFCom/` 打包成 `DFCom.zip`（或直接拷到 `Documents/Arduino/libraries/`）
2. Arduino IDE：`项目 → 加载库 → 添加 .ZIP 库`，选 `DFCom.zip`
3. `文件 → 示例 → DFCom → DcarON_Square` 打开示例
4. 选板子 `Arduino Uno`（或 Nano）→ 点「上传」

### 用法 B — 单文件 (零安装, 纯小白)
直接双击 `SingleFile/DcarON_Square_AllInOne/DcarON_Square_AllInOne.ino`，
选板子 → 上传。一个文件搞定，连库都不用装。

---

## 接线 (Uno/Nano 5V ↔ 车 USART3 3.3V)

```
Arduino D1(TX) → [电平转换 5V→3.3V] → 车 USART3_RX (PB11)
Arduino D0(RX) ←──────── 直连 ─────── 车 USART3_TX (PB10)
Arduino GND   ←─────────────────────  车 GND
(可选调试) Arduino D3 → USB-TTL 的 RX，9600 看打印
```

> ⚠️ **D1(TX)→车 RX 这条线必须加电平转换/分压**（5V 直怼 3.3V 引脚有风险）。
> 车→Arduino 方向 3.3V 可直读，不用转。
> ⚠️ 上传程序时 D0/D1 被 USB 占用，**上传完再接小车**（或上传时先拔开 D0/D1）。

---

## API 速查 (默认单位: 厘米 + 度)

```cpp
DFCom.begin();                       // 连小车 (115200)
DFCom.subscribeOdom(10);             // 订阅全量 Odom 10Hz (≤50Hz)
DFCom.subscribeVelPos(10);           // 订阅简化 VelPos 10Hz (按需打开)

DFCom.moveLinear(px, py, speed, 2);  // 直线: +X前进 +Y左移, profile=2 高精度
DFCom.moveRot(dyaw);                 // 原地转: +左转(CCW), 度
DFCom.moveArc(radius, dyaw, speed);  // 圆弧
DFCom.moveVel(vx, vy, vz);           // 持续速度 (不自动停)
DFCom.stop();                        // 停车
DFCom.waitDone();                    // 等上一条真到位

DFCom.update();                      // loop 里调, 收里程计 (waitDone 内部已自动调)
DFCom.xCm();  DFCom.yCm();  DFCom.yawDeg();   // 当前位置/朝向
DFCom.imuOK();                       // 小车是否校准好

DFCom.useMeters();                   // 想用米+弧度 (SI 原生) 就调这个
```

---

## ⚠️ 两个前置 / 注意

1. **小车固件需要一个小改动**：里程计回传默认走 UART4/USART1（460800，Arduino 读不了）。
   已在车固件 `DF_Bytes_Send` 里加了 **USART3 @115200 回传镜像**，把固件重烧到小车后，
   Arduino 才能在 USART3 上收到 odom / 完成回传。（指令方向不用改，USART3 RX 天生就解析 DFLink。）

2. **里程计订阅 ≤50Hz**：USART3 @115200 每帧约 4.4ms，订阅太快会挤占回传。教学用 10Hz 足够。

3. **SoftwareSerial 调试要轻**：它打印时会短暂屏蔽中断，可能丢硬件串口字节。
   所以例程把打印放在 `waitDone()` 之外、低频短消息。别在订阅 100Hz 时狂打。

---

## 资源占用 (ATmega328P: 32KB flash / 2KB RAM)

| 版本 | Flash | RAM |
|---|---|---|
| 库 + 示例 | 8.4KB (26%) | 573B (27%) |
| 单文件 | 6.3KB (19%) | 428B (20%) |

余量充足。
