/******************************************************************************
 * DFCom.h  ——  Arduino (ATmega328P) 版 DcarON 小车通信库
 * ----------------------------------------------------------------------------
 * 把 STM32 版 DFCom_v2 协议移植到 Arduino Uno/Nano, 对象式 API, 学生友好。
 * 官网: https://differ-tech.pages.dev/
 *
 *  ╔══════════════════════════════════════════════════════════════════════╗
 *  ║  接线 (Arduino Uno/Nano 5V  ↔  小车 USART3 3.3V):                      ║
 *  ║    Arduino D1(TX) → [电平转换 5V→3.3V] → 车 USART3_RX (PB11)           ║
 *  ║    Arduino D0(RX) ←────── 直连 ──────── 车 USART3_TX (PB10)            ║
 *  ║    Arduino GND   ←──────────────────── 车 GND                          ║
 *  ║  波特率固定 115200。调试想看打印 → 用 SoftwareSerial (见例程)。        ║
 *  ╚══════════════════════════════════════════════════════════════════════╝
 *
 *  坐标系 (ROS REP-103):  +X 前进 · +Y 左移 · +Yaw 逆时针(CCW) 为正
 *
 *  默认单位 = 厘米 + 度 (学生友好)。想用米+弧度: DFCom.useMeters();
 *
 *  最简用法:
 *    #include <DFCom.h>
 *    void setup(){ DFCom.begin(); DFCom.subscribeOdom(10); }
 *    void loop(){
 *      DFCom.moveLinear(50, 0, 30, 2); DFCom.waitDone(10000); // 前进 50cm
 *      DFCom.moveRot(90);             DFCom.waitDone(10000); // 左转 90°
 *    }
 ******************************************************************************/
#ifndef DFCOM_H
#define DFCOM_H

#include <Arduino.h>

/* ---- ODOM 订阅 ---- */
#define DFCOM_MODE_CONTINUOUS 0x01   /* 持续推送 (最常用) */
#define DFCOM_MODE_ONESHOT    0x02   /* 只发一帧 */
#define DFCOM_MODE_STOP       0x00   /* 停止推送 */

/* ---- 指令码 (waitDone 用) ---- */
#define DFCOM_CMD_VEL     0x62
#define DFCOM_CMD_ROT     0x63
#define DFCOM_CMD_LINEAR  0x64
#define DFCOM_CMD_LWY     0x65   /* LinearWithYaw */
#define DFCOM_CMD_ARC     0x66

/* ---- 单位模式 ---- */
#define DFCOM_UNIT_CM 0   /* 默认: 厘米 + 度 */
#define DFCOM_UNIT_M  1   /* 米 + 弧度 (SI 原生) */
#define DFCOM_UNIT_MM 2   /* 毫米 + 度 */

#define DFCOM_RAD2DEG(r) ((r) * 57.29578f)
#define DFCOM_DEG2RAD(d) ((d) * 0.01745329f)

/* 里程计数据 (全量 DF_odom, SI 原生单位: m / m/s / rad / rad/s) */
struct DFOdom {
    float roll, pitch, yaw;          /* rad, yaw CCW+ */
    float accX, accY, accZ;          /* m/s² */
    float gyroX, gyroY, gyroZ;       /* rad/s */
    float bVelX, bVelY, bVelZ;       /* 车体系速度 m/s */
    float nVelX, nVelY, nVelZ;       /* 世界系速度 m/s */
    float posX, posY, posZ;          /* 世界系位置 m (开机累积) */
    uint32_t carTimeS, carTimeUs;
    uint32_t frameCount;
    uint8_t  fresh;
};

/* 简化里程计 (DF_vel_pose) */
struct DFVelPos {
    float yaw;
    float bVelX, bVelY, bVelZ;
    float nVelX, nVelY, nVelZ;
    float posX, posY, posZ;
    uint32_t carTimeS, carTimeUs;
    uint32_t frameCount;
    uint8_t  fresh;
};

/* 小车激活/校准状态 */
struct DFState {
    uint8_t received;        /* 1 = 收到过回传 (链路活) */
    uint8_t imuCalibrated;   /* 1 = IMU 已校准 (可用) */
    uint8_t isFinetune;
    int16_t gyroZoffX100;
    int16_t imuParam2X10000;
    int16_t imuTempX10000;
    int16_t ctrlPar1X100;
    int16_t ctrlPar2X100;
};

class DFCom_t {
public:
    /* ---- 初始化（内部会先停车并隔离旧运动回传） ---- */
    void begin(long baud = 115200, HardwareSerial &port = Serial);
    void setDebug(Stream &dbg) { _dbg = &dbg; }   /* 可选: 接 SoftwareSerial 看打印 */

    /* ---- 单位 (默认厘米+度) ---- */
    void useCentimeters() { _unit = DFCOM_UNIT_CM; }
    void useMeters()      { _unit = DFCOM_UNIT_M;  }
    void useMillimeters() { _unit = DFCOM_UNIT_MM; }

    /* ---- 运动指令 (单位随当前模式; 默认 cm + deg) ----
     *   +X 前进(px/vx>0) · +Y 左移(py/vy>0) · +yaw 左转(dyaw>0, CCW)
     *   profile: 0=匀速  1=梯形加减速  2=高精度距离闭环 */
    void moveLinear(float px, float py, float speed, uint8_t profile = 2);
    void moveLinearWithYaw(float px, float py, float dyaw, float speed, uint8_t profile = 1);
    void moveRot(float dyaw, float omegaMax = 60);   /* 默认 60 deg/s */
    void moveVel(float vx, float vy, float vz);       /* 持续速度, 不自动停 */
    void moveArc(float radius, float dyaw, float speed, uint8_t profile = 1);
    void stop() { moveVel(0, 0, 0); }                 /* 主动停车 */

    /* ---- 里程计订阅 / 状态查询 ---- */
    void subscribeOdom(uint16_t freqHz = 10, uint8_t mode = DFCOM_MODE_CONTINUOUS);
    void subscribeVelPos(uint16_t freqHz = 10, uint8_t mode = DFCOM_MODE_CONTINUOUS);
    void queryState();

    /* ---- 必须在 loop() 里(或 waitDone 内部)调, 收+解析回传 ---- */
    void update();

    /* ---- 等运动完成 (阻塞, 内部自动 update) ----
     *   waitDone()            等最近一条运动指令
     *   waitDone(cmd, tmo_ms) 指定指令; tmo=0 表示永久等待
     *   tmo>0 是最长执行时间: 提前到位就提前返回, 到时可发下一条打断
     *   返回 true=自然完成, false=超时/中断/发送失败 */
    bool waitDone(uint32_t timeoutMs = 0) { return waitDone(_lastCmd, timeoutMs); }
    bool waitDone(uint8_t cmdCode, uint32_t timeoutMs);
    uint8_t progress(uint8_t cmdCode);

    /* ---- 数据 (SI 原生, 直接读) ---- */
    DFOdom   odom;
    DFVelPos velpos;
    DFState  state;

    /* ---- 友好取值 (换成 cm / 度) ---- */
    float xCm()    { return odom.posX * 100.0f; }
    float yCm()    { return odom.posY * 100.0f; }
    float yawDeg() { return DFCOM_RAD2DEG(odom.yaw); }
    bool  imuOK()  { return state.received && state.imuCalibrated; }

private:
    HardwareSerial *_ser = &Serial;
    Stream *_dbg = nullptr;
    uint8_t _unit = DFCOM_UNIT_CM;
    uint8_t _lastCmd = DFCOM_CMD_LINEAR;

    struct MoveSlot {
        uint8_t cmd;
        uint8_t progress;
        uint8_t notice;
        uint8_t fresh;
    };

    /* 两槽按本地发送顺序交替；上一任务的尾帧只进入丢弃槽。 */
    MoveSlot _moveSlots[2];
    uint8_t _currentSlot = 0xFF;
    uint8_t _discardSlot = 0xFF;
    uint8_t _discardCmd = 0;
    uint8_t _motionSessionActive = 0;
    uint8_t _sendFailed = 0;
    uint8_t _failedCmd = 0;

    /* 字节流解析状态机 */
    uint8_t _buf[72];
    uint8_t _idx = 0;
    uint8_t _st = 0;       /* 0=找头 1=收头部 2=收剩余 */
    uint8_t _need = 0;     /* 整帧总长 */

    bool _frame(uint8_t A, uint8_t B, const uint8_t *payload, uint8_t len);  /* 建帧+校验+发送 */
    void _feed(uint8_t byteIn);                                              /* 字节流解析 */
    void _dispatch(uint8_t A, uint8_t B, const uint8_t *p, uint8_t len);
    static bool _isBoundedMotion(uint8_t cmd);
    void _clearMoveSlot(uint8_t slot);
    void _releaseDiscard();
    void _retireCurrent();
    uint8_t _nextMoveSlot() const;
    void _resetMotionSession(uint8_t active);
    void _motionSent(uint8_t cmd);
    void _motionSendFailed(uint8_t cmd);
    void _routeMotionProgress(uint8_t cmd, uint8_t progress, uint8_t notice);
    float _toM(float v);     /* 长度入参 → 米 (按单位模式) */
    float _toRad(float v);   /* 角度入参 → 弧度 (按单位模式) */
};

extern DFCom_t DFCom;

#endif /* DFCOM_H */
