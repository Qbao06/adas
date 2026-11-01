/* ADAS: 2 ESP32 + 4 HLK-LD2420 V2. Arduino IDE, nap 2 lan theo DEVICE_ROLE. */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

#define ROLE_GATEWAY 1
#define ROLE_REMOTE  2
#ifndef DEVICE_ROLE
#define DEVICE_ROLE ROLE_GATEWAY
#endif

// SUA cac gia tri nay. Hai board phai dung cung channel.
static constexpr uint8_t ESPNOW_CHANNEL=6;
// ESP32 A (Gateway): F0:24:F9:0C:66:DC
// ESP32 B (Remote) : 44:1D:64:BC:BA:C8
static constexpr uint8_t GATEWAY_MAC[6]={0xF0,0x24,0xF9,0x0C,0x66,0xDC};
static constexpr uint8_t REMOTE_MAC[6] ={0x44,0x1D,0x64,0xBC,0xBA,0xC8};
// Chan UART rieng cho tung board theo day da han thuc te.
// A/Gateway: OT1 radar 1 -> GPIO17, OT1 radar 2 -> GPIO19.
// B/Remote : OT1 radar 1 -> GPIO16, OT1 radar 2 -> GPIO18.
#if DEVICE_ROLE == ROLE_GATEWAY
static constexpr int R1_RX=17, R1_TX=16, R2_RX=19, R2_TX=18;
#else
static constexpr int R1_RX=16, R1_TX=17, R2_RX=18, R2_TX=19;
#endif

static constexpr uint32_t USB_BAUD=115200, RADAR_BAUD=115200;
static constexpr uint32_t RADAR_STALE_MS=500, REMOTE_STALE_MS=800, PERIOD_MS=50;
static constexpr uint32_t MAGIC=0x3234324CUL; // L242
static constexpr uint8_t VERSION=1;

struct Reading { uint16_t cm, energy; uint32_t updated, frames; uint8_t detected, valid, error; };

class LD2420 {
 public:
  void begin(HardwareSerial &s,int rx,int tx){ p=&s; p->setRxBufferSize(1024); p->begin(RADAR_BAUD,SERIAL_8N1,rx,tx); r={0,0,0,0,0,0,1}; }
  void poll(){ while(p->available()) consume((uint8_t)p->read()); if(r.valid && millis()-r.updated>RADAR_STALE_MS){r.valid=0;r.error=1;} }
  Reading get() const{return r;}
 private:
  HardwareSerial *p=nullptr; Reading r{}; uint8_t b[64]{},match=0; size_t n=0,ln=0; char line[48]{}; bool presence=false;
  static uint16_t le16(const uint8_t*x){return (uint16_t)x[0]|((uint16_t)x[1]<<8);}
  void publish(bool on,uint16_t cm,uint16_t e){r.cm=on?cm:0;r.energy=e;r.updated=millis();r.frames++;r.detected=on;r.valid=1;r.error=0;}
  void consume(uint8_t c){
    static const uint8_t H[4]={0xF4,0xF3,0xF2,0xF1};
    if(n){if(n<sizeof(b))b[n++]=c;else{n=0;r.error=2;} binary();return;}
    if(c==H[match]){if(++match==4){memcpy(b,H,4);n=4;match=0;}return;} match=(c==H[0]);
    if(c=='\r'||c=='\n'){if(ln){line[ln]=0;text();ln=0;}}
    else if(c>=32&&c<=126){if(ln<sizeof(line)-1)line[ln++]=(char)c;else ln=0;}
  }
  void binary(){
    if(n<6)return;
    uint16_t payload=le16(b+4); size_t total=(size_t)payload+10;
    if(payload<3||total>sizeof(b)){n=0;r.error=2;return;} if(n<total)return;
    bool footer=b[total-4]==0xF8&&b[total-3]==0xF7&&b[total-2]==0xF6&&b[total-1]==0xF5;
    if(!footer){n=0;r.error=2;return;} uint16_t peak=0;
    for(size_t i=9;i+1<total-4;i+=2){uint16_t e=le16(b+i);if(e>peak)peak=e;}
    publish(b[6]!=0,le16(b+7),peak);n=0;
  }
  void text(){
    for(size_t i=0;i<ln;i++)if(line[i]>='a'&&line[i]<='z')line[i]-=32;
    if(strstr(line,"OFF"))presence=false;else if(strstr(line,"ON"))presence=true;
    char*q=strstr(line,"RANGE");
    if(q){while(*q&&(*q<'0'||*q>'9'))q++;unsigned long v=strtoul(q,nullptr,10);if(v<=65535)publish(presence,(uint16_t)v,0);}
    else if(strstr(line,"ON")||strstr(line,"OFF"))publish(presence,presence?r.cm:0,0);
  }
};

#pragma pack(push,1)
struct WireRadar{uint16_t cm,energy;uint32_t age,frames;uint8_t detected,valid,error;};
struct Packet{uint32_t magic,boot,seq,sent;WireRadar radar[2];uint32_t crc;uint8_t version;};
#pragma pack(pop)
static_assert(sizeof(Packet)<=ESP_NOW_MAX_DATA_LEN,"packet too large");

static LD2420 radar1,radar2; static uint32_t boot_id,seq,last_tick;
static volatile bool remote_seen=false; static volatile uint32_t remote_time,remote_boot,remote_seq,loss;
static WireRadar remote_data[2]{}; static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;

static bool zeroMac(const uint8_t*m){uint8_t x=0;for(int i=0;i<6;i++)x|=m[i];return x==0;}
static uint32_t crc32(const uint8_t*d,size_t z){uint32_t c=~0UL;for(size_t i=0;i<z;i++){c^=d[i];for(int k=0;k<8;k++)c=(c>>1)^(0xEDB88320UL&(0UL-(c&1)));}return~c;}
static WireRadar wire(Reading r,uint32_t now){WireRadar w{r.cm,r.energy,r.updated?now-r.updated:0xFFFFFFFFUL,r.frames,r.detected,r.valid,r.error};w.valid=w.valid&&w.age<=RADAR_STALE_MS;if(!w.valid&&w.error==0)w.error=1;return w;}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
static void received(const esp_now_recv_info_t*i,const uint8_t*d,int len){const uint8_t*mac=i?i->src_addr:nullptr;
#else
static void received(const uint8_t*mac,const uint8_t*d,int len){
#endif
  if(!mac||!d||len!=(int)sizeof(Packet)||(!zeroMac(REMOTE_MAC)&&memcmp(mac,REMOTE_MAC,6)))return;
  Packet p;memcpy(&p,d,sizeof(p));uint32_t c=p.crc;p.crc=0;
  if(p.magic!=MAGIC||p.version!=VERSION||crc32((uint8_t*)&p,sizeof(p))!=c)return;
  portENTER_CRITICAL_ISR(&mux);
  if(remote_seen&&p.boot==remote_boot&&p.seq<=remote_seq){portEXIT_CRITICAL_ISR(&mux);return;}
  if(remote_seen&&p.boot==remote_boot&&p.seq>remote_seq+1)loss+=p.seq-remote_seq-1;
  remote_seq=p.seq;remote_boot=p.boot;memcpy(remote_data,p.radar,sizeof(remote_data));remote_time=millis();remote_seen=true;
  portEXIT_CRITICAL_ISR(&mux);
}

static bool espnow(){
  WiFi.mode(WIFI_STA);if(esp_wifi_set_channel(ESPNOW_CHANNEL,WIFI_SECOND_CHAN_NONE)!=ESP_OK||esp_now_init()!=ESP_OK)return false;
#if DEVICE_ROLE == ROLE_GATEWAY
  return esp_now_register_recv_cb(received)==ESP_OK;
#else
  if(zeroMac(GATEWAY_MAC))return false;esp_now_peer_info_t peer{};memcpy(peer.peer_addr,GATEWAY_MAC,6);peer.channel=ESPNOW_CHANNEL;return esp_now_add_peer(&peer)==ESP_OK;
#endif
}

static void jsonRadar(const char*name,WireRadar r,bool link){
  bool ok=link&&r.valid&&r.age<=RADAR_STALE_MS;
  Serial.print('"');Serial.print(name);Serial.print(F("\":{\"detected\":"));Serial.print(ok&&r.detected?F("true"):F("false"));
  Serial.print(F(",\"target_state\":\""));Serial.print(ok&&r.detected?F("moving"):F("none"));Serial.print(F("\",\"distance_m\":"));
  if(ok&&r.detected)Serial.print(r.cm/100.0f,2);else Serial.print(F("null"));
  Serial.print(F(",\"moving_distance_m\":"));if(ok&&r.detected)Serial.print(r.cm/100.0f,2);else Serial.print(F("null"));
  Serial.print(F(",\"stationary_distance_m\":null,\"moving_energy\":"));if(ok)Serial.print(r.energy);else Serial.print(F("null"));
  Serial.print(F(",\"stationary_energy\":null,\"valid\":"));Serial.print(ok?F("true"):F("false"));Serial.print(F(",\"stale\":"));Serial.print(ok?F("false"):F("true"));
  Serial.print(F(",\"age_ms\":"));Serial.print(r.age);Serial.print(F(",\"error\":"));Serial.print(ok?0:r.error);Serial.print('}');
}
static void gatewayJson(){
  uint32_t now=millis();WireRadar a=wire(radar1.get(),now),b=wire(radar2.get(),now),rr[2];uint32_t rb,pl,rt;bool seen;
  portENTER_CRITICAL(&mux);memcpy(rr,remote_data,sizeof(rr));rb=remote_boot;pl=loss;rt=remote_time;seen=remote_seen;portEXIT_CRITICAL(&mux);
  bool rok=seen&&now-rt<=REMOTE_STALE_MS;if(rok){rr[0].age+=now-rt;rr[1].age+=now-rt;}
  Serial.print(F("{\"v\":2,\"seq\":"));Serial.print(++seq);Serial.print(F(",\"gateway_ok\":1,\"remote_ok\":"));Serial.print(rok?1:0);
  Serial.print(F(",\"gateway_boot_id\":"));Serial.print(boot_id);Serial.print(F(",\"remote_boot_id\":"));Serial.print(rb);Serial.print(F(",\"packet_loss\":"));Serial.print(pl);Serial.print(F(",\"radars\":{"));
  jsonRadar("left_front",a,true);Serial.print(',');jsonRadar("left_rear",b,true);Serial.print(',');jsonRadar("right_front",rr[0],rok);Serial.print(',');jsonRadar("right_rear",rr[1],rok);Serial.println(F("}}"));
}
static void remoteSend(){uint32_t now=millis();Packet p{};p.magic=MAGIC;p.boot=boot_id;p.seq=++seq;p.sent=now;p.radar[0]=wire(radar1.get(),now);p.radar[1]=wire(radar2.get(),now);p.version=VERSION;p.crc=0;p.crc=crc32((uint8_t*)&p,sizeof(p));esp_now_send(GATEWAY_MAC,(uint8_t*)&p,sizeof(p));}
static void printMac(const uint8_t*m){for(int i=0;i<6;i++){if(i)Serial.print(':');if(m[i]<16)Serial.print('0');Serial.print(m[i],HEX);}Serial.println();}

void setup(){
  Serial.begin(USB_BAUD);delay(500);WiFi.mode(WIFI_STA);uint8_t mac[6];WiFi.macAddress(mac);Serial.print(F("# LOCAL_MAC="));printMac(mac);
  Serial.print(F("# ROLE="));Serial.println(DEVICE_ROLE==ROLE_GATEWAY?F("GATEWAY"):F("REMOTE"));boot_id=(uint32_t)(ESP.getEfuseMac()^esp_random());
  radar1.begin(Serial1,R1_RX,R1_TX);radar2.begin(Serial2,R2_RX,R2_TX);Serial.println(espnow()?F("# ESP-NOW READY"):F("# ERROR ESP-NOW: DIEN MAC/KIEM TRA CHANNEL"));
}
void loop(){radar1.poll();radar2.poll();uint32_t now=millis();if(now-last_tick>=PERIOD_MS){last_tick=now;
#if DEVICE_ROLE == ROLE_GATEWAY
gatewayJson();
#else
remoteSend();
#endif
}delay(1);}
