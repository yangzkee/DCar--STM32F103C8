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
DFCom.begin();                       // 连小车；先停车两次并隔离旧运动回传
DFCom.subscribeOdom(10);             // 订阅全量 Odom 10Hz (≤50Hz)
DFCom.subscribeVelPos(10);           // 订阅简化 VelPos 10Hz (按需打开)

DFCom.moveLinear(px, py, speed, 2);  // 直线: +X前进 +Y左移, profile=2 高精度
DFCom.moveRot(dyaw);                 // 原地转: +左转(CCW), 度
DFCom.moveArc(radius, dyaw, speed);  // 圆弧
DFCom.moveVel(vx, vy, vz);           // 持续速度 (不自动停)
DFCom.stop();                        // 停车
DFCom.waitDone(10000);               // 到位即返回；否则最多执行 10 秒

DFCom.update();                      // loop 里调, 收里程计 (waitDone 内部已自动调)
DFCom.xCm();  DFCom.yCm();  DFCom.yawDeg();   // 当前位置/朝向
DFCom.imuOK();                       // 小车是否校准好

DFCom.useMeters();                   // 想用米+弧度 (SI 原生) 就调这个
```

`waitDone(timeoutMs)` 的参数是这条任务允许执行的最长时间，不是固定延时：
任务提前到位就立即返回 `true`；到达上限仍未完成则返回 `false`，随后发送的
下一条运动指令可以按协议合法打断它。`timeoutMs == 0` 表示永久等待，链路
失联时也不会自行返回。

库内部按发送顺序使用 A/B 两个任务槽。相邻任务不会共用同一个槽，上一任务
随后到达的唯一终止帧只负责释放旧槽，不会让当前任务提前结束。`waitDone`
入口也不会再次清槽，因此发送后快速返回的完成帧不会丢失。

`begin()` 会先进入运动回传隔离态，立即发送零速度，等待 DCar 供电/串口稳定
后再次发送零速度，最后清空本地任务槽并开启新的运动会话。

---

## ⚠️ 两个前置 / 注意

1. **小车固件需要一个小改动**：里程计回传默认走 UART4/USART1（460800，Arduino 读不了）。
   已在车固件 `DF_Bytes_Send` 里加了 **USART3 @115200 回传镜像**，把固件重烧到小车后，
   Arduino 才能在 USART3 上收到 odom / 完成回传。（指令方向不用改，USART3 RX 天生就解析 DFLink。）

2. **里程计订阅 ≤50Hz**：USART3 @115200 每帧约 4.4ms，订阅太快会挤占回传。教学用 10Hz 足够。

3. **SoftwareSerial 调试要轻**：它打印时会短暂屏蔽中断，可能丢硬件串口字节。
   所以例程把打印放在 `waitDone()` 之外、低频短消息。别在订阅 100Hz 时狂打。

---

## 发布前离线回归

仓库内的主机测试会直接编译 Arduino 库源码和单文件源码，并验证快速完成、
同类任务旧终止帧、最大等待时间和启动双停车：

```sh
sh tests/arduino/run_arduino_tests.sh
```

最终 Flash/RAM 占用以目标 Arduino IDE / `arduino-cli` 的 AVR 编译报告为准。
