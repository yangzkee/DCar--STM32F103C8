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

/* 里程计 (SI 原生: m / rad) */
float odomYaw = 0, odomPosX = 0, odomPosY = 0;   /* yaw(rad), 世界系位置(m) */
float odomVx = 0, odomVy = 0;                    /* 车体系速度(m/s) */
uint8_t odomFresh = 0;
/* 小车状态 */
uint8_t carReceived = 0, carImuOK = 0;
/* 运动完成槽 (cmd&0x07) */
volatile uint8_t mvProg[8], mvNotice[8], mvFresh[8];
uint8_t dfLastCmd = 0x64;

static void dfPutS32(uint8_t *b, int32_t v){ b[0]=v; b[1]=v>>8; b[2]=v>>16; b[3]=v>>24; }
static int16_t  dfS16(const uint8_t *p){ return (int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
static int32_t  dfS32(const uint8_t *p){ return (int32_t)((uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24)); }

/* 建帧 + 校验 + 发送 */
static void dfFrame(uint8_t A, uint8_t B, const uint8_t *pl, uint8_t len){
    uint8_t f[40], n=0;
    f[n++]=DF_HEAD; f[n++]=DF_ROBOT; f[n++]=DF_PC; f[n++]=A; f[n++]=B; f[n++]=len;
    for(uint8_t i=0;i<len;i++) f[n++]=pl[i];
    f[n++]=DF_TAIL;
    uint16_t sc=0; for(uint8_t i=0;i<(uint8_t)(len+7);i++) sc+=f[i];
    f[n++]=sc&0xFF; f[n++]=(sc>>8)&0xFF;
    Serial.write(f,n);
}
static void dfClear(uint8_t c){ uint8_t i=c&7; mvProg[i]=0; mvNotice[i]=0; mvFresh[i]=0; }

/* ---- 运动指令 (入参: cm + deg) ---- */
void dcarMoveLinear(float px_cm, float py_cm, float spd_cms, uint8_t profile=2){
    if(spd_cms<=0) return; if(profile>2) profile=2;
    uint8_t p[13];
    dfPutS32(&p[0],(int32_t)(px_cm*0.01f*SC_SI));
    dfPutS32(&p[4],(int32_t)(py_cm*0.01f*SC_SI));
    dfPutS32(&p[8],(int32_t)(spd_cms*0.01f*SC_SI));
    p[12]=profile; dfClear(0x64); dfLastCmd=0x64; dfFrame(0x02,0x64,p,13);
}
void dcarMoveRot(float dyaw_deg, float omega_dps=60){
    if(omega_dps<=0) return;
    uint8_t p[8];
    dfPutS32(&p[0],(int32_t)(dyaw_deg*DEG2RAD*SC_SI));
    dfPutS32(&p[4],(int32_t)(omega_dps*DEG2RAD*SC_SI));
    dfClear(0x63); dfLastCmd=0x63; dfFrame(0x02,0x63,p,8);
}
void dcarMoveArc(float r_cm, float dyaw_deg, float spd_cms, uint8_t profile=1){
    if(spd_cms<=0) return; if(profile>2) profile=2;
    uint8_t p[13];
    dfPutS32(&p[0],(int32_t)(r_cm*0.01f*SC_SI));
    dfPutS32(&p[4],(int32_t)(dyaw_deg*DEG2RAD*SC_SI));
    dfPutS32(&p[8],(int32_t)(spd_cms*0.01f*SC_SI));
    p[12]=profile; dfClear(0x66); dfLastCmd=0x66; dfFrame(0x02,0x66,p,13);
}
void dcarMoveVel(float vx_cms, float vy_cms, float vz_dps){
    uint8_t p[12];
    dfPutS32(&p[0],(int32_t)(vx_cms*0.01f*SC_SI));
    dfPutS32(&p[4],(int32_t)(vy_cms*0.01f*SC_SI));
    dfPutS32(&p[8],(int32_t)(vz_dps*DEG2RAD*SC_SI));
    dfFrame(0x02,0x62,p,12);
}
void dcarStop(){ dcarMoveVel(0,0,0); }
void dcarSubscribeOdom(uint8_t freqHz=10){
    uint8_t fc = (freqHz==50)?5:(freqHz==100)?10:(freqHz==200)?20:(freqHz==250)?25:(freqHz==500)?50:1;
    uint8_t p[2]={0x01,fc}; dfFrame(0x04,0x80,p,2);
}
void dcarSubscribeVelPos(uint8_t freqHz=10){
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
        uint8_t i=B&7; mvProg[i]=p[0]; mvNotice[i]=p[1]; mvFresh[i]=1;
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

/* ---- 等运动完成 (阻塞, 内部自动收) ---- */
bool dcarWaitDone(uint32_t timeoutMs=0){
    uint8_t i=dfLastCmd&7;
    mvProg[i]=0; mvNotice[i]=0; mvFresh[i]=0;
    uint32_t start=millis();
    for(;;){
        dcarUpdate();
        if(mvFresh[i] && mvProg[i]==0xFF && mvNotice[i]==0) return true;
        if(timeoutMs && (millis()-start)>timeoutMs) return false;
    }
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

    Serial.begin(115200);       // 连小车 (D0/D1 @ 115200)
    dcarSubscribeOdom(10);      // 订阅里程计 10Hz
}

void loop()
{
    /* 走一个边长 50cm 的方块 */
    for (uint8_t i = 0; i < 4; i++) {
        dcarMoveLinear(50, 0, 30, 2); // 前进 50cm, Profile 2
        dcarWaitDone();
        dcarMoveRot(90);             // 左转 90°
        dcarWaitDone();
    }

    /* 打印里程计 (单位换算成 cm / 度) */
    dcarUpdate();
    dbg.print(F("x="));   dbg.print(odomPosX * 100.0f);
    dbg.print(F("cm y=")); dbg.print(odomPosY * 100.0f);
    dbg.print(F("cm yaw=")); dbg.print(odomYaw * 57.29578f);
    dbg.println(F("deg"));

    delay(1000);
}
