/******************************************************************************
 * DcarON_Square_AllInOne —— 单文件版 (零安装, 双击打开直接上传)
 * ============================================================================
 *  ★★★ 你只需要看 / 改最底下的 setup() 和 loop() ★★★
 *  中间这一大段是 DcarON 通信库 (协议组帧/解析), 不用懂、不用改。
 * ============================================================================
 *  接线 (Arduino Uno/Nano 5V ↔ 车 USART3 3.3V):
 *    D1(TX) → [电平转换 5V→3.3V] → 车 USART3_RX(PB11)
 *    D0(RX) ←──── 直连 ─────────── 车 USART3_TX(PB10)
 *    GND   ←──────────────────────  车 GND
 *    (可选) D3 → USB-TTL RX, 9600 看打印
 *  默认单位: 厘米 + 度。坐标: +X 前进 / +Y 左移 / +yaw 左转(CCW)。
 ******************************************************************************/
#include <SoftwareSerial.h>

/* ===========================  ↓↓↓ 协议库 (不用看) ↓↓↓  ====================== */
#define DF_HEAD 0xDF
#define DF_TAIL 0xFD
#define DF_ROBOT 0x01
#define DF_PC    0x97
#define SC_SI 10000.0f
#define DEG2RAD 0.01745329f
#define MOVE_SLOT_NONE 0xFF

/* 里程计 (SI 原生: m / rad) */
float odomYaw = 0, odomPosX = 0, odomPosY = 0;   /* yaw(rad), 世界系位置(m) */
float odomVx = 0, odomVy = 0;                    /* 车体系速度(m/s) */
uint8_t odomFresh = 0;
/* 小车状态 */
uint8_t carReceived = 0, carImuOK = 0;

/* 运动任务槽: 相邻任务按发送顺序在 A/B 两槽间交替。 */
struct DcarMoveSlot {
    uint8_t cmd;
    uint8_t progress;
    uint8_t notice;
    uint8_t fresh;
};
static DcarMoveSlot mvSlots[2];
static uint8_t dfLastCmd = 0x64;
static uint8_t dfCurrentSlot = MOVE_SLOT_NONE;
static uint8_t dfDiscardSlot = MOVE_SLOT_NONE;
static uint8_t dfDiscardCmd = 0;
static uint8_t dfMotionSessionActive = 0;
static uint8_t dfSendFailed = 0;
static uint8_t dfFailedCmd = 0;

static void dfPutS32(uint8_t *b, int32_t v){ b[0]=v; b[1]=v>>8; b[2]=v>>16; b[3]=v>>24; }
static int16_t  dfS16(const uint8_t *p){ return (int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
static int32_t  dfS32(const uint8_t *p){ return (int32_t)((uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24)); }

/* 建帧 + 校验 + 发送 */
static bool dfFrame(uint8_t A, uint8_t B, const uint8_t *pl, uint8_t len){
    uint8_t f[40], n=0;
    f[n++]=DF_HEAD; f[n++]=DF_ROBOT; f[n++]=DF_PC; f[n++]=A; f[n++]=B; f[n++]=len;
    for(uint8_t i=0;i<len;i++) f[n++]=pl[i];
    f[n++]=DF_TAIL;
    uint16_t sc=0; for(uint8_t i=0;i<(uint8_t)(len+7);i++) sc+=f[i];
    f[n++]=sc&0xFF; f[n++]=(sc>>8)&0xFF;
    return Serial.write(f,n)==n;
}

static bool dfIsBoundedMotion(uint8_t cmd){
    return cmd>=0x63 && cmd<=0x66;
}
static void dfClearMoveSlot(uint8_t slot){
    if(slot>1) return;
    mvSlots[slot].cmd=0;
    mvSlots[slot].progress=0;
    mvSlots[slot].notice=0;
    mvSlots[slot].fresh=0;
}
static void dfReleaseDiscard(){
    if(dfDiscardSlot<=1) dfClearMoveSlot(dfDiscardSlot);
    dfDiscardSlot=MOVE_SLOT_NONE;
    dfDiscardCmd=0;
}
static void dfRetireCurrent(){
    uint8_t slot=dfCurrentSlot;
    if(slot>1) return;

    uint8_t cmd=mvSlots[slot].cmd;
    bool terminal=mvSlots[slot].fresh && mvSlots[slot].progress==0xFF;
    if(terminal){
        dfClearMoveSlot(slot);
    }else{
        if(dfDiscardSlot<=1 && dfDiscardSlot!=slot) dfReleaseDiscard();
        dfClearMoveSlot(slot);
        dfDiscardSlot=slot;
        dfDiscardCmd=cmd;
    }
    dfCurrentSlot=MOVE_SLOT_NONE;
}
static uint8_t dfNextMoveSlot(){
    if(dfCurrentSlot<=1) return dfCurrentSlot^1U;
    if(dfDiscardSlot<=1) return dfDiscardSlot^1U;
    return 0;
}
static void dfResetMotionSession(uint8_t active){
    dfClearMoveSlot(0);
    dfClearMoveSlot(1);
    dfCurrentSlot=MOVE_SLOT_NONE;
    dfDiscardSlot=MOVE_SLOT_NONE;
    dfDiscardCmd=0;
    dfSendFailed=0;
    dfFailedCmd=0;
    dfMotionSessionActive=active;
}
static void dfMotionSent(uint8_t cmd){
    if(cmd!=0x62 && !dfIsBoundedMotion(cmd)) return;

    dfSendFailed=0;
    dfFailedCmd=0;
    if(!dfMotionSessionActive) return;
    if(cmd==0x62){
        dfRetireCurrent();
        return;
    }

    uint8_t next=dfNextMoveSlot();
    if(dfDiscardSlot==next) dfReleaseDiscard();
    dfRetireCurrent();
    dfClearMoveSlot(next);
    mvSlots[next].cmd=cmd;
    dfCurrentSlot=next;
}
static void dfMotionSendFailed(uint8_t cmd){
    if(dfMotionSessionActive && dfIsBoundedMotion(cmd)){
        dfSendFailed=1;
        dfFailedCmd=cmd;
    }
}
static void dfRouteMotionProgress(uint8_t cmd, uint8_t progress, uint8_t notice){
    if(!dfMotionSessionActive || !dfIsBoundedMotion(cmd)) return;

    if(dfDiscardSlot<=1 && cmd==dfDiscardCmd){
        if(progress==0xFF) dfReleaseDiscard();
        return;
    }
    if(dfCurrentSlot<=1 && mvSlots[dfCurrentSlot].cmd==cmd){
        mvSlots[dfCurrentSlot].progress=progress;
        mvSlots[dfCurrentSlot].notice=notice;
        mvSlots[dfCurrentSlot].fresh=1;
    }
}

/* ---- 运动指令 (入参: cm + deg) ---- */
void dcarMoveLinear(float px_cm, float py_cm, float spd_cms, uint8_t profile=2){
    dfLastCmd=0x64;
    if(spd_cms<=0){ dfMotionSendFailed(0x64); return; }
    if(profile>2) profile=2;
    uint8_t p[13];
    dfPutS32(&p[0],(int32_t)(px_cm*0.01f*SC_SI));
    dfPutS32(&p[4],(int32_t)(py_cm*0.01f*SC_SI));
    dfPutS32(&p[8],(int32_t)(spd_cms*0.01f*SC_SI));
    p[12]=profile;
    if(dfFrame(0x02,0x64,p,13)) dfMotionSent(0x64);
    else dfMotionSendFailed(0x64);
}
void dcarMoveRot(float dyaw_deg, float omega_dps=60){
    dfLastCmd=0x63;
    if(omega_dps<=0){ dfMotionSendFailed(0x63); return; }
    uint8_t p[8];
    dfPutS32(&p[0],(int32_t)(dyaw_deg*DEG2RAD*SC_SI));
    dfPutS32(&p[4],(int32_t)(omega_dps*DEG2RAD*SC_SI));
    if(dfFrame(0x02,0x63,p,8)) dfMotionSent(0x63);
    else dfMotionSendFailed(0x63);
}
void dcarMoveArc(float r_cm, float dyaw_deg, float spd_cms, uint8_t profile=1){
    dfLastCmd=0x66;
    if(spd_cms<=0){ dfMotionSendFailed(0x66); return; }
    if(profile>2) profile=2;
    uint8_t p[13];
    dfPutS32(&p[0],(int32_t)(r_cm*0.01f*SC_SI));
    dfPutS32(&p[4],(int32_t)(dyaw_deg*DEG2RAD*SC_SI));
    dfPutS32(&p[8],(int32_t)(spd_cms*0.01f*SC_SI));
    p[12]=profile;
    if(dfFrame(0x02,0x66,p,13)) dfMotionSent(0x66);
    else dfMotionSendFailed(0x66);
}
void dcarMoveVel(float vx_cms, float vy_cms, float vz_dps){
    uint8_t p[12];
    dfPutS32(&p[0],(int32_t)(vx_cms*0.01f*SC_SI));
    dfPutS32(&p[4],(int32_t)(vy_cms*0.01f*SC_SI));
    dfPutS32(&p[8],(int32_t)(vz_dps*DEG2RAD*SC_SI));
    if(dfFrame(0x02,0x62,p,12)) dfMotionSent(0x62);
}
void dcarStop(){ dcarMoveVel(0,0,0); }
void dcarSubscribeOdom(uint16_t freqHz=10){
    uint8_t fc = (freqHz==50)?5:(freqHz==100)?10:(freqHz==200)?20:(freqHz==250)?25:(freqHz==500)?50:1;
    uint8_t p[2]={0x01,fc}; dfFrame(0x04,0x80,p,2);
}
void dcarSubscribeVelPos(uint16_t freqHz=10){
    uint8_t fc = (freqHz==50)?5:(freqHz==100)?10:(freqHz==200)?20:(freqHz==250)?25:(freqHz==500)?50:1;
    uint8_t p[2]={0x01,fc}; dfFrame(0x04,0x81,p,2);
}
void dcarQueryState(){ dfFrame(0x0B,0x40,(const uint8_t*)0,0); }

/* ---- 接收: 字节流解析 (loop / waitDone 里调) ---- */
static uint8_t dfBuf[72], dfIdx=0, dfSt=0, dfNeed=0;
static void dfDispatch(uint8_t A, uint8_t B, const uint8_t *p, uint8_t len){
    if(A==0x6C && B==0x80 && len>=51 && p[0]==0x04){
        odomYaw  = dfS16(&p[5])/10000.0f;
        odomVx   = dfS16(&p[19])/5000.0f;
        odomVy   = dfS16(&p[21])/5000.0f;
        odomPosX = dfS32(&p[31])/1000.0f;
        odomPosY = dfS32(&p[35])/1000.0f;
        odomFresh=1;
    } else if(A==0x6C && B==0x81 && len>=35 && p[0]==0x02){
        odomYaw  = dfS16(&p[1])/10000.0f;
        odomVx   = dfS16(&p[3])/5000.0f;
        odomVy   = dfS16(&p[5])/5000.0f;
        odomPosX = dfS32(&p[15])/1000.0f;
        odomPosY = dfS32(&p[19])/1000.0f;
        odomFresh=1;
    } else if(A==0x6F && len>=2){
        dfRouteMotionProgress(B,p[0],p[1]);
    } else if(A==0x8C && B==0x40 && len>=12){
        carImuOK=p[0]; carReceived=1;
    }
}
static void dfFeed(uint8_t b){
    if(dfSt==0){ if(b==DF_HEAD){ dfBuf[0]=DF_HEAD; dfIdx=1; dfSt=1; } }
    else if(dfSt==1){
        dfBuf[dfIdx++]=b;
        if(dfIdx==6){
            uint8_t len=dfBuf[5];
            if(dfBuf[1]!=DF_PC||dfBuf[2]!=DF_ROBOT||len>60){ dfSt=0; dfIdx=0; if(b==DF_HEAD){dfBuf[0]=DF_HEAD;dfIdx=1;dfSt=1;} }
            else { dfNeed=(uint8_t)(6+len+3); dfSt=2; }
        }
    } else {
        if(dfIdx<sizeof(dfBuf)) dfBuf[dfIdx++]=b;
        if(dfIdx>=dfNeed){
            uint8_t len=dfBuf[5], tp=(uint8_t)(6+len);
            if(dfBuf[tp]==DF_TAIL){
                uint16_t sc=0; for(uint8_t i=0;i<(uint8_t)(len+7);i++) sc+=dfBuf[i];
                uint16_t rec=(uint16_t)dfBuf[tp+1]|((uint16_t)dfBuf[tp+2]<<8);
                if(sc==rec) dfDispatch(dfBuf[3],dfBuf[4],&dfBuf[6],len);
            }
            dfSt=0; dfIdx=0;
        }
    }
}
void dcarUpdate(){ while(Serial.available()) dfFeed((uint8_t)Serial.read()); }

/* ---- 等运动完成 (阻塞, 内部自动收; 不在 wait 入口清槽) ---- */
bool dcarWaitDone(uint32_t timeoutMs=0){
    bool sendFailed=dfSendFailed && dfFailedCmd==dfLastCmd;
    uint8_t slot=(dfCurrentSlot<=1 && mvSlots[dfCurrentSlot].cmd==dfLastCmd)
                     ? dfCurrentSlot
                     : MOVE_SLOT_NONE;
    if(sendFailed || slot==MOVE_SLOT_NONE) return false;

    uint32_t start=millis();
    for(;;){
        dcarUpdate();
        if(mvSlots[slot].fresh && mvSlots[slot].progress==0xFF)
            return mvSlots[slot].notice==0;
        if(timeoutMs && (millis()-start)>=timeoutMs) return false;
    }
}

/* 建立新的本地运动会话：先停车两次，再丢弃启动前的旧运动回传。 */
void dcarBegin(){
    Serial.begin(115200);
    dfResetMotionSession(0);
    dcarStop();
    delay(1000);
    dcarStop();
    delay(50);
    dcarUpdate();
    dfIdx=0; dfSt=0; dfNeed=0;
    dfResetMotionSession(1);
}
/* ===========================  ↑↑↑ 协议库 (不用看) ↑↑↑  ====================== */


/* ============================================================================
 *  ★★★  下面才是你要改的!  ★★★
 * ============================================================================*/
SoftwareSerial dbg(2, 3);   /* 可选调试: D3 → USB-TTL RX, 9600 */

void setup()
{
    dbg.begin(9600);
    dbg.println(F("== DcarON Arduino (single file) =="));

    dcarBegin();                // 连小车，并先停车/清理旧运动会话
    dcarSubscribeOdom(10);      // 订阅里程计 10Hz
}

void loop()
{
    /* 走一个边长 50cm 的方块 */
    for (uint8_t i = 0; i < 4; i++) {
        dcarMoveLinear(50, 0, 30, 2); // 前进 50cm, Profile 2
        dcarWaitDone(10000);          // 到位即返回，10s 为链路兜底
        dcarMoveRot(90);             // 左转 90°
        dcarWaitDone(10000);
    }

    /* 打印里程计 (单位换算成 cm / 度) */
    dcarUpdate();
    dbg.print(F("x="));   dbg.print(odomPosX * 100.0f);
    dbg.print(F("cm y=")); dbg.print(odomPosY * 100.0f);
    dbg.print(F("cm yaw=")); dbg.print(odomYaw * 57.29578f);
    dbg.println(F("deg"));

    delay(1000);
}
