/******************************************************************************
 * DcarON_Square —— DcarON 小车走方块 (Arduino Uno/Nano 版)
 * ----------------------------------------------------------------------------
 * 接线:
 *   Arduino D1(TX) → [电平转换 5V→3.3V] → 车 USART3_RX (PB11)
 *   Arduino D0(RX) ←──────直连──────────── 车 USART3_TX (PB10)
 *   Arduino GND   ←──────────────────────  车 GND
 *   (可选调试) Arduino D3 → USB-TTL 的 RX,  9600 看打印
 *
 * 默认单位: 厘米 + 度。坐标系: +X 前进 / +Y 左移 / +yaw 左转(CCW)。
 ******************************************************************************/
#include <DFCom.h>
#include <SoftwareSerial.h>

/* 可选调试串口: D2=RX(没用到), D3=TX → 接 USB-TTL 看打印。不接也能跑。 */
SoftwareSerial dbg(2, 3);

void setup()
{
    dbg.begin(9600);
    dbg.println(F("== DcarON Arduino DFCom =="));

    DFCom.begin(115200);        // 连小车，并先停车/清理旧运动会话
    DFCom.setDebug(dbg);
    DFCom.subscribeOdom(10);    // 订阅里程计 10Hz (≤50Hz)

    /* 可选: 问小车校准好没 (300ms 等回传) */
    DFCom.queryState();
    uint32_t t0 = millis();
    while (!DFCom.state.received && (millis() - t0) < 300) DFCom.update();
    if (DFCom.imuOK())            dbg.println(F("car ready (IMU calibrated)"));
    else if (DFCom.state.received) dbg.println(F("WARN: IMU NOT calibrated"));
    else                          dbg.println(F("WARN: no reply from car (check wiring/baud)"));
}

void loop()
{
    /* 走一个边长 50cm 的方块 (前进 + 左转 90°, 四次) */
    for (uint8_t i = 0; i < 4; i++) {
        DFCom.moveLinear(50, 0, 30, 2); // 前进 50cm, 30cm/s, Profile 2
        DFCom.waitDone(10000);         // 到位即返回，10s 为链路兜底
        DFCom.moveRot(90);             // 原地左转 90°
        DFCom.waitDone(10000);
    }

    /* 一圈走完, 打印里程计 (注意: 打印放在 waitDone 之外, 不干扰收帧) */
    DFCom.update();
    dbg.print(F("pos x=")); dbg.print(DFCom.xCm());
    dbg.print(F("cm y=")); dbg.print(DFCom.yCm());
    dbg.print(F("cm yaw=")); dbg.print(DFCom.yawDeg());
    dbg.println(F("deg"));

    delay(1000);
}
