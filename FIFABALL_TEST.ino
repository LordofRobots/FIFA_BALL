// /********************************************************
//  * CubicChaos — Cube (ESP32-S3)
//  * Pins:
//  *   SCL=1, SDA=2, MPU_INT=3, PIEZO=4, LED=5
//  ********************************************************/

// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_now.h>
// #include <esp_wifi.h>
// #include <Wire.h>
// #include <MPU6050.h>
// #include <FastLED.h>
// #include <math.h>

// /* ===================== USER CONFIG ===================== */
// #define SCL_PIN      1
// #define SDA_PIN      2
// #define MPU_INT_PIN  3
// #define PIEZO_PIN    4
// #define LED_PIN      5

// #define FACE_LEDS 3
// #define NUM_FACES 6
// #define NUM_LEDS  (FACE_LEDS * NUM_FACES)

// #define PROTO_VER 5

// #ifndef CUBE_DEBUG
// #define CUBE_DEBUG 1
// #endif

// #if CUBE_DEBUG
//   #define CLOG(...) do{ Serial.printf(__VA_ARGS__); }while(0)
// #else
//   #define CLOG(...) do{}while(0)
// #endif

// /* ===================== Protocol ===================== */
// enum MsgType : uint8_t { MSG_PAIR_REQ=1, MSG_PAIR_ACK=2, MSG_POLL=3, MSG_STATUS=4, MSG_CMD=5 };

// // System states (match FMS)
// enum : uint8_t { SYS_STANDBY=0, SYS_GAME_START=1, SYS_IN_GAME=2, SYS_TIME_GATE=3, SYS_END_GAME=4, SYS_RESET=5 };

// // Team+level colors (match FMS)
// #define C_WHITE      0
// #define C_BLUE_1     10
// #define C_BLUE_2     11
// #define C_ORANGE_1   20
// #define C_ORANGE_2   21

// // Utility colors
// #define C_RED        3
// #define C_GREEN      4
// #define C_PURPLE     5

// static inline uint8_t levelOf(uint8_t c){
//   if(c==C_BLUE_1 || c==C_ORANGE_1) return 1;
//   if(c==C_BLUE_2 || c==C_ORANGE_2) return 2;
//   return 0;
// }

// struct __attribute__((packed)) PairReq {
//   uint8_t  proto_ver;
//   uint8_t  msg_type;
//   uint8_t  desired_fms;
//   uint32_t nonce;
// };

// struct __attribute__((packed)) PairAck {
//   uint8_t  proto_ver;
//   uint8_t  msg_type;
//   uint8_t  fms_id;
//   uint8_t  channel;
//   uint32_t nonce;
// };

// struct __attribute__((packed)) PollPkt {
//   uint8_t proto_ver;
//   uint8_t msg_type;
//   uint8_t seq;
// };

// struct __attribute__((packed)) StatusPkt {
//   uint8_t  proto_ver;
//   uint8_t  msg_type;
//   uint8_t  seq;
//   uint8_t  face;
//   uint8_t  stable20;
//   uint32_t uptime_s;
// };

// struct __attribute__((packed)) CmdPkt {
//   uint8_t  proto_ver;
//   uint8_t  msg_type;
//   uint8_t  sys_state;
//   uint8_t  color;
//   uint16_t beep_hz;
//   uint16_t beep_ms;
// };

// static_assert(sizeof(PairReq)  == 1+1+1+4,  "PairReq size mismatch");
// static_assert(sizeof(PairAck)  == 1+1+1+1+4,"PairAck size mismatch");
// static_assert(sizeof(PollPkt)  == 1+1+1,    "PollPkt size mismatch");
// static_assert(sizeof(StatusPkt)== 1+1+1+1+1+4,"StatusPkt size mismatch");
// static_assert(sizeof(CmdPkt)   == 1+1+1+1+2+2,"CmdPkt size mismatch");

// static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// static inline bool macEq(const uint8_t a[6], const uint8_t b[6]) { return memcmp(a,b,6)==0; }
// static inline bool macIsZero(const uint8_t a[6]) { return (a[0]|a[1]|a[2]|a[3]|a[4]|a[5]) == 0; }

// /* ===================== RX Queue (ISR-safe ring buffer) ===================== */
// struct RxFrame {
//   uint8_t  src[6];
//   uint8_t  len;
//   uint8_t  data[32];
// };

// class RxQueue {
// public:
//   static constexpr uint8_t  CAP   = 16;
//   static constexpr uint16_t MAXRX = 32;

//   bool push_isr(const uint8_t* src, const uint8_t* data, int len){
//     if(!src || !data) return false;
//     if(len <= 0 || len > (int)MAXRX) return false;

//     bool ok = false;
//     portENTER_CRITICAL_ISR(&mux_);

//     uint8_t next = (uint8_t)(head_ + 1);
//     if(next >= CAP) next = 0;

//     if(next == tail_){
//       drops_++;
//       ok = false;
//     } else {
//       RxFrame& f = q_[head_];
//       memcpy(f.src, src, 6);
//       f.len = (uint8_t)len;
//       memcpy(f.data, data, (size_t)len);
//       head_ = next;
//       ok = true;
//     }

//     portEXIT_CRITICAL_ISR(&mux_);
//     return ok;
//   }

//   bool pop(RxFrame& out){
//     bool ok = false;

//     portENTER_CRITICAL(&mux_);
//     if(tail_ != head_){
//       out = q_[tail_];
//       uint8_t next = (uint8_t)(tail_ + 1);
//       if(next >= CAP) next = 0;
//       tail_ = next;
//       ok = true;
//     }
//     portEXIT_CRITICAL(&mux_);

//     return ok;
//   }

//   uint32_t drops() const { return drops_; }

// private:
//   RxFrame q_[CAP]{};
//   volatile uint8_t head_ = 0;
//   volatile uint8_t tail_ = 0;
//   portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
//   volatile uint32_t drops_ = 0;
// };

// /* ===================== Sound (LEDC, packaged + independent) ===================== */
// class LedcToneOut {
// public:
//   void begin(uint8_t pin, uint8_t ledcChannel){
//     _pin = pin;
//     _ch = ledcChannel;

//     pinMode(_pin, OUTPUT);

// #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
//     (void)_ch;
//     ledcAttach(_pin, 1000 /*placeholder*/, 10 /*bits*/);
// #else
//     ledcSetup(_ch, 1000 /*placeholder*/, 10 /*bits*/);
//     ledcAttachPin(_pin, _ch);
// #endif
//     stop();
//   }

//   void tick(uint32_t ms){
//     if(_active && _durMs){
//       if((int32_t)(ms - _stopAtMs) >= 0) stop();
//     }
//   }

//   void play(uint16_t hz, uint16_t durationMs, uint16_t duty /*0..1023*/){
//     if(hz == 0){ stop(); return; }

//     _active = true;
//     _durMs  = durationMs;
//     _stopAtMs = (durationMs ? (millis() + durationMs) : 0);

// #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
//     ledcWriteTone(_pin, hz);
//     ledcWrite(_pin, clampDuty10_(duty));
// #else
//     ledcWriteTone(_ch, hz);
//     ledcWrite(_ch, clampDuty10_(duty));
// #endif
//   }

//   void stop(){
//     _active = false;
//     _durMs  = 0;
//     _stopAtMs = 0;

// #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
//     ledcWrite(_pin, 0);
//     ledcWriteTone(_pin, 0);
// #else
//     ledcWrite(_ch, 0);
//     ledcWriteTone(_ch, 0);
// #endif
//   }

//   bool active() const { return _active; }

// private:
//   uint8_t _pin = 0;
//   uint8_t _ch  = 0;

//   bool _active = false;
//   uint16_t _durMs = 0;
//   uint32_t _stopAtMs = 0;

//   static uint16_t clampDuty10_(uint16_t d){ return (d > 1023) ? 1023 : d; }
// };

// class IdentifySound {
// public:
//   void begin(uint8_t pin, uint8_t ledcChannel){
//     _tone.begin(pin, ledcChannel);
//     reset_();
//   }

//   void trigger(uint32_t now){
//     _active = true;
//     _untilMs = now + 2000;
//     _nextBeepMs = now;
//     _beepCount = 0;
//   }

//   void tick(uint32_t ms){
//     _tone.tick(ms);

//     if(!_active) return;

//     if((int32_t)(ms - _untilMs) >= 0){
//       reset_();
//       _tone.stop();
//       return;
//     }

//     if(_beepCount < 3 && (int32_t)(ms - _nextBeepMs) >= 0){
//       _tone.play(3000, 70, 512);
//       _beepCount++;
//       _nextBeepMs = ms + 333;
//     }
//   }

//   bool active() const { return _active; }

// private:
//   LedcToneOut _tone;
//   bool _active = false;
//   uint32_t _untilMs = 0;
//   uint32_t _nextBeepMs = 0;
//   uint8_t _beepCount = 0;

//   void reset_(){
//     _active = false;
//     _untilMs = 0;
//     _nextBeepMs = 0;
//     _beepCount = 0;
//   }
// };

// /* ===================== IMU IRQ (IRAM-safe, no object refs) ===================== */
// static volatile bool     g_imu_drdy = false;
// static volatile uint32_t g_imu_isrCount = 0;

// void IRAM_ATTR onImuInt_ISR(){
//   g_imu_drdy = true;
//   g_imu_isrCount++;
// }

// /* ===================== IMU Manager ===================== */
// class ImuManager {
// public:
//   void begin(){
//     Wire.begin(SDA_PIN, SCL_PIN);
//     imuInit_();

//     pinMode(MPU_INT_PIN, INPUT_PULLUP);
//     attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onImuInt_ISR, FALLING);
//   }

//   void primeFace(){
//     curFace_ = 0;
//     stable20_ = true;
//     delay(40);
//     for(int i=0;i<3;i++){ curFace_ = detectFace20_(curFace_); delay(20); }
//     lastFaceMs_ = millis();
//   }

//   void tick(uint32_t ms){
//     if(ms - lastFaceMs_ >= 40){
//       lastFaceMs_ = ms;
//       curFace_ = detectFace20_(curFace_);
//     }
//   }

//   uint8_t face() const { return curFace_; }
//   bool stable20() const { return stable20_; }
//   bool ready() const { return imu_ready_; }
//   uint32_t isrCount() const { return g_imu_isrCount; }
//   uint8_t decideDesiredFmsByStartupFace() const { return (curFace_==0) ? 1 : 2; }

// private:
//   MPU6050 mpu_;
//   bool imu_ready_ = false;

//   uint8_t curFace_ = 0;
//   bool stable20_ = true;
//   uint32_t lastFaceMs_ = 0;

//   static const int8_t Rm_[3][3];

//   void imuInit_(){
//     mpu_.initialize();
//     mpu_.setSleepEnabled(false);
//     mpu_.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//     mpu_.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
//     #ifdef MPU6050_DLPF_BW_10
//       mpu_.setDLPFMode(MPU6050_DLPF_BW_10);
//     #else
//       mpu_.setDLPFMode(5);
//     #endif

//     mpu_.setInterruptLatch(true);
//     mpu_.setInterruptLatchClear(true);
//     mpu_.setIntDataReadyEnabled(true);

//     delay(10);
//     imu_ready_ = mpu_.testConnection();
//   }

//   bool readAccelUnit_(float& x, float& y, float& z){
//     if(!imu_ready_) return false;

//     const uint8_t  N = 10;
//     const uint32_t MAX_US = 6000;
//     const uint32_t t0 = micros();

//     long sx=0, sy=0, sz=0;
//     uint8_t got=0;

//     uint32_t lastFallbackUs = t0;

//     while(got < N && (micros() - t0) < MAX_US){
//       bool take = false;

//       if(g_imu_drdy){
//         g_imu_drdy = false;
//         take = true;
//       } else {
//         uint32_t nowUs = micros();
//         if((nowUs - lastFallbackUs) >= 900){
//           lastFallbackUs = nowUs;
//           take = true;
//         }
//       }
//       if(!take) continue;

//       int16_t ax,ay,az;
//       mpu_.getAcceleration(&ax,&ay,&az);
//       (void)mpu_.getIntStatus();

//       int32_t rx = (int32_t)Rm_[0][0]*ax + (int32_t)Rm_[0][1]*ay + (int32_t)Rm_[0][2]*az;
//       int32_t ry = (int32_t)Rm_[1][0]*ax + (int32_t)Rm_[1][1]*ay + (int32_t)Rm_[1][2]*az;
//       int32_t rz = (int32_t)Rm_[2][0]*ax + (int32_t)Rm_[2][1]*ay + (int32_t)Rm_[2][2]*az;

//       sx+=rx; sy+=ry; sz+=rz; got++;
//     }

//     if(got < 4) return false;

//     float fx=sx/(float)got, fy=sy/(float)got, fz=sz/(float)got;
//     float mag=sqrtf(fx*fx + fy*fy + fz*fz);
//     if(!(mag > 1e-3f)) return false;

//     x=fx/mag; y=fy/mag; z=fz/mag;
//     return true;
//   }

//   uint8_t detectFace20_(uint8_t prevFace){
//     float x,y,z;
//     if(!readAccelUnit_(x,y,z)) return prevFace;

//     float dots[6]={ x,-x, y,-y, z,-z };
//     int best=0; float bdot=dots[0];
//     for(int i=1;i<6;i++){ if(dots[i]>bdot){ bdot=dots[i]; best=i; } }

//     const float ENTER_COS = 0.9396926f;
//     stable20_ = (bdot >= ENTER_COS);

//     if(prevFace>5) return (uint8_t)best;
//     if(best!=prevFace && bdot>=ENTER_COS) return (uint8_t)best;
//     return prevFace;
//   }
// };

// const int8_t ImuManager::Rm_[3][3] = {
//   {+1, 0, 0},
//   { 0,+1, 0},
//   { 0, 0,+1}
// };

// /* ===================== LED Management (updated behaviors) ===================== */
// class LedManager {
// public:
//   void begin(){
//     FastLED.addLeds<NEOPIXEL, LED_PIN>(leds_, NUM_LEDS);
//     FastLED.setCorrection(UncorrectedColor);
//     FastLED.setDither(0);
//     FastLED.setBrightness(brightness_);

//     clearAll_();
//     FastLED.show();

//     rp_target_ = (uint8_t)(esp_random() % NUM_LEDS);
//     rp_nextMs_ = millis() + 500;

//     lastRenderMs_ = 0;
//     lastShowMs_ = 0;
//     dirty_ = true;

//     endgameValid_ = false;
//     endgameStartMs_ = 0;
//     endgameEntryColor_ = C_WHITE;
//     endgameEntryUnstable_ = false;
//   }

//   void tick(uint32_t ms,
//             bool identifyActive,
//             bool disconnected,
//             uint8_t sysState,
//             uint8_t teamColor,
//             bool stable20,
//             uint32_t timeGateStartMs){
//     // SYS_END_GAME entry snapshot (base + entry conditions)
//     if(sysState != lastSysStateTransition_){
//       if(sysState == SYS_END_GAME){
//         snapshotEndgameEntry_(ms, teamColor, stable20);
//       } else {
//         endgameValid_ = false;
//         endgameStartMs_ = 0;
//         endgameEntryColor_ = C_WHITE;
//         endgameEntryUnstable_ = false;
//       }
//       lastSysStateTransition_ = sysState;
//       dirty_ = true;
//     }

//     if((int32_t)(ms - lastRenderMs_) < (int32_t)RENDER_PERIOD_MS_){
//       if(dirty_ && (int32_t)(ms - lastShowMs_) >= (int32_t)SHOW_PERIOD_MS_){
//         FastLED.show();
//         lastShowMs_ = ms;
//         dirty_ = false;
//       }
//       return;
//     }
//     lastRenderMs_ = ms;

//     const bool animating =
//       identifyActive ||
//       (sysState == SYS_STANDBY) ||
//       (sysState == SYS_TIME_GATE && timeGateStartMs && (ms - timeGateStartMs <= 500)) ||
//       (sysState == SYS_END_GAME) ||
//       (!stable20 && (sysState == SYS_IN_GAME)) ||
//       (disconnected);

//     if(!animating &&
//        !dirty_ &&
//        sysState == lastSysStateLatch_ &&
//        teamColor == lastTeamColor_ &&
//        identifyActive == lastIdentify_ &&
//        disconnected == lastDisconnected_ &&
//        stable20 == lastStable_){
//       return;
//     }

//     lastSysStateLatch_ = sysState;
//     lastTeamColor_ = teamColor;
//     lastIdentify_ = identifyActive;
//     lastDisconnected_ = disconnected;
//     lastStable_ = stable20;

//     clearAll_();

//     if(identifyActive){
//       const bool on = flash5Hz_(ms);
//       renderSolidAllFaces_(on ? C_GREEN : C_PURPLE);

//     } else if(sysState == SYS_STANDBY){
//       renderStandbyTwoPixel_(ms);

//     } else if(sysState == SYS_TIME_GATE){
//       if(timeGateStartMs && (ms - timeGateStartMs <= 500)){
//         for(uint8_t i=0;i<NUM_LEDS;i++){
//           leds_[i] = CHSV((uint8_t)((ms/3) + i*10), 255, 180);
//         }
//       } else {
//         renderTeamLevelPattern_(teamColor);
//       }

//     } else if(sysState == SYS_RESET){
//       renderSolidAllFaces_(C_PURPLE);

//     } else if(sysState == SYS_GAME_START){
//       renderSolidAllFaces_(C_GREEN);

//     } else if(sysState == SYS_IN_GAME){
//       if(teamColor == C_WHITE){
//         // NEW: In-game white = 2 pixels per face ON, center OFF
//         if(stable20){
//           renderInGameWhitePattern_();
//         } else {
//           if(flash5Hz_(ms)) renderInGameWhitePattern_();
//         }
//       } else {
//         if(stable20){
//           renderTeamLevelPattern_(teamColor);
//         } else {
//           if(flash5Hz_(ms)) renderTeamLevelPattern_(teamColor);
//         }
//       }

//     } else if(sysState == SYS_END_GAME){
//       if(!endgameValid_) snapshotEndgameEntry_(ms, teamColor, stable20);

//       const uint32_t dt = (endgameStartMs_ == 0) ? 0 : (ms - endgameStartMs_);

//       if(dt < 2000){
//         // Full red for 2 seconds (overrides everything)
//         renderSolidAllFaces_(C_RED);
//       } else {
//         // Underlay:
//         // - If entry color was white: underlay OFF (black)
//         // - If entry was unstable: keep flashing the entry display
//         clearAll_();

//         if(endgameEntryColor_ != C_WHITE){
//           if(endgameEntryUnstable_){
//             if(flash5Hz_(ms)){
//               renderTeamLevelPattern_(endgameEntryColor_);
//             }
//           } else {
//             renderTeamLevelPattern_(endgameEntryColor_);
//           }
//         }

//         // Overlay: two opposite red pixels cycling, DOES NOT suppress underlay except those pixels.
//         const uint32_t t = dt - 2000;

//         // Slower cycle (slower than prior):
//         //  - 6 LEDs/sec => full loop ~3.0s for 18 LEDs
//         const float pos = fmodf((float)t * 0.006f, (float)NUM_LEDS);
//         int i0 = (int)floorf(pos);
//         float frac = pos - (float)i0;
//         if(i0 < 0) i0 = 0;
//         const int i1 = (i0 + 1) % NUM_LEDS;

//         const uint8_t v0 = (uint8_t)(170.0f * (1.0f - frac));
//         const uint8_t v1 = (uint8_t)(170.0f * (frac));

//         overlayOppPairPureRed_(i0, v0);
//         overlayOppPairPureRed_(i1, v1);
//       }

//     } else {
//       renderStandbyTwoPixel_(ms);
//     }

//     // Disconnected center overlay (not during identify)
//     if(disconnected && !identifyActive){
//       const bool on = ((ms/250) & 1) == 0;
//       applyCenterAllFaces_(on ? CRGB(100,0,0) : CRGB::Black);
//     }

//     dirty_ = true;

//     if((int32_t)(ms - lastShowMs_) >= (int32_t)SHOW_PERIOD_MS_){
//       FastLED.show();
//       lastShowMs_ = ms;
//       dirty_ = false;
//     }
//   }

// private:
//   CRGB leds_[NUM_LEDS];

//   static constexpr uint8_t  TARGET_FPS_ = 120;
//   static constexpr uint32_t RENDER_PERIOD_MS_ = (1000 / TARGET_FPS_);
//   static constexpr uint32_t SHOW_PERIOD_MS_   = (1000 / TARGET_FPS_);

//   uint8_t brightness_ = 80;

//   uint32_t lastRenderMs_ = 0;
//   uint32_t lastShowMs_ = 0;
//   bool dirty_ = true;

//   uint8_t lastSysStateTransition_ = 0xFF;
//   uint8_t lastSysStateLatch_ = 0xFF;
//   uint8_t lastTeamColor_ = 0xFF;
//   bool lastIdentify_ = false;
//   bool lastDisconnected_ = false;
//   bool lastStable_ = true;

//   uint8_t rp_target_ = 0;
//   uint32_t rp_nextMs_ = 0;

//   bool endgameValid_ = false;
//   uint32_t endgameStartMs_ = 0;
//   uint8_t endgameEntryColor_ = C_WHITE;
//   bool endgameEntryUnstable_ = false;

//   static inline uint8_t faceBase_(uint8_t f){ static const uint8_t base[6]={0,6,3,12,9,15}; return base[f]; }
//   static inline uint8_t faceCenterIdx_(uint8_t f){ return faceBase_(f)+1; }
//   static inline uint8_t faceOpp_(uint8_t f){ static const uint8_t o[6]={1,0,3,2,5,4}; return o[f]; }

//   static inline void decodeIdx_(uint8_t idx, uint8_t& f, uint8_t& off){
//     for(uint8_t i=0;i<6;i++){
//       uint8_t b=faceBase_(i);
//       if(idx>=b && idx<b+FACE_LEDS){ f=i; off=idx-b; return; }
//     }
//     f=0; off=0;
//   }

//   static inline bool flash5Hz_(uint32_t ms){ return ((ms/100) & 1) == 0; }

//   static CRGB colorToCRGB_(uint8_t c){
//     switch(c){
//       case C_WHITE:    return CRGB(80,90,90);
//       case C_BLUE_1:
//       case C_BLUE_2:   return CRGB(0,0,160);
//       case C_ORANGE_1:
//       case C_ORANGE_2: return CRGB(170,40,0);
//       case C_RED:      return CRGB(150,0,0);
//       case C_GREEN:    return CRGB(0,150,0);
//       case C_PURPLE:   return CRGB(100,0,150);
//       default:         return CRGB::Black;
//     }
//   }

//   void clearAll_(){ memset(leds_, 0, sizeof(leds_)); }

//   void renderSolidAllFaces_(uint8_t color){
//     const CRGB c = colorToCRGB_(color);
//     for(uint8_t f=0; f<NUM_FACES; f++){
//       const uint8_t b = faceBase_(f);
//       leds_[b]   = c;
//       leds_[b+1] = c;
//       leds_[b+2] = c;
//     }
//   }

//   void applyCenterAllFaces_(CRGB centerColor){
//     for(uint8_t f=0; f<NUM_FACES; f++){
//       leds_[faceCenterIdx_(f)] = centerColor;
//     }
//   }

//   void renderTeamLevelPattern_(uint8_t teamLevelColor){
//     const CRGB c = colorToCRGB_(teamLevelColor);
//     const uint8_t lvl = levelOf(teamLevelColor);

//     if(lvl == 0){
//       renderSolidAllFaces_(teamLevelColor);
//       return;
//     }

//     for(uint8_t f=0; f<NUM_FACES; f++){
//       const uint8_t b = faceBase_(f);

//       if(lvl == 1){
//         leds_[b]   = CRGB::Black;
//         leds_[b+1] = c;
//         leds_[b+2] = CRGB::Black;
//       } else {
//         leds_[b]   = c;
//         leds_[b+1] = CRGB::Black;
//         leds_[b+2] = c;
//       }
//     }
//   }

//   void renderInGameWhitePattern_(){
//     // 2 pixels per face ON, center OFF
//     const CRGB w = colorToCRGB_(C_WHITE);
//     for(uint8_t f=0; f<NUM_FACES; f++){
//       const uint8_t b = faceBase_(f);
//       leds_[b]   = w;
//       leds_[b+1] = CRGB::Black; // center off
//       leds_[b+2] = w;
//     }
//   }

//   static inline uint8_t hueForIndex_(uint8_t idx, uint32_t ms){
//     return ((ms/10) & 0xFF) + (uint8_t)(idx * 7);
//   }

//   void renderStandbyTwoPixel_(uint32_t ms){
//     if((int32_t)(ms - rp_nextMs_) >= 0){
//       rp_target_ = (uint8_t)(esp_random() % NUM_LEDS);
//       rp_nextMs_ = ms + 1000;
//     }

//     const uint8_t i = rp_target_;
//     uint8_t f,o;
//     decodeIdx_(i, f, o);

//     leds_[i] = CHSV(hueForIndex_(i, ms), 255, 190);

//     const uint8_t opp = (uint8_t)(faceBase_(faceOpp_(f)) + o);
//     leds_[opp] = CHSV(hueForIndex_(opp, ms), 255, 190);
//   }

//   void snapshotEndgameEntry_(uint32_t ms, uint8_t teamColor, bool stable20){
//     endgameStartMs_ = ms;
//     endgameEntryColor_ = teamColor;
//     endgameEntryUnstable_ = !stable20;
//     endgameValid_ = true;
//   }

//   void overlayOppPairPureRed_(int idx, uint8_t redVal){
//     if(redVal == 0) return;
//     if(idx < 0) return;
//     if(idx >= (int)NUM_LEDS) return;

//     // Override pixel to pure red (prevents mixing/tinting the underlay)
//     leds_[idx].r = redVal;
//     leds_[idx].g = 0;
//     leds_[idx].b = 0;

//     uint8_t f=0, off=0;
//     decodeIdx_((uint8_t)idx, f, off);
//     const uint8_t oppIdx = (uint8_t)(faceBase_(faceOpp_(f)) + off);

//     leds_[oppIdx].r = redVal;
//     leds_[oppIdx].g = 0;
//     leds_[oppIdx].b = 0;
//   }
// };

// /* ===================== ESPNOW Link / Pairing / Cube->FMS comm ===================== */
// class EspNowLink {
// public:
//   void begin(RxQueue* rxq){
//     rxq_ = rxq;

//     WiFi.mode(WIFI_STA);
//     WiFi.setSleep(false);
//     WiFi.disconnect(false, true);
//     delay(20);

//     wifiChannel_ = 1;
//     (void)esp_wifi_set_channel(wifiChannel_, WIFI_SECOND_CHAN_NONE);

//     if(esp_now_init()!=ESP_OK){
//       delay(50);
//       if(esp_now_init()!=ESP_OK){
//         delay(250);
//         ESP.restart();
//       }
//     }

//     self_ = this;
//     esp_now_register_recv_cb(onRecvThunk_);
//     esp_now_register_send_cb(onSentThunk_);

//     (void)upsertPeer_(BCAST);

//     paired_ = false;
//     memset(fmsMac_, 0, sizeof(fmsMac_));
//     lastFmsMs_ = 0;

//     desiredFms_ = 1;
//     pairNonce_ = esp_random();

//     nextPairReqMs_ = 0;
//     pairBackoffMs_ = 250;

//     cmdSysState_ = SYS_STANDBY;
//     cmdColor_    = C_WHITE;

//     txPending_ = false;
//     txPendingSinceMs_ = 0;
//     lastStatusSeqSent_ = 0xFF;

//     pollPending_ = false;
//     pendingPollSeq_ = 0;

//     identifyTrigger_ = false;

//     rxPollCount_ = 0;
//     rxCmdCount_  = 0;
//     rxAckCount_  = 0;
//     txStatusCount_ = 0;
//     txFailCount_ = 0;
//   }

//   void setDesiredFms(uint8_t fms){ desiredFms_ = fms; }

//   void service(ImuManager& imu, uint32_t ms){
//     RxFrame fr;
//     while(rxq_ && rxq_->pop(fr)){
//       handleRxFrameFast_(fr, imu, ms);
//     }

//     tickPairing_(ms);

//     if(pollPending_){
//       if(!txPending_ || (int32_t)(ms - txPendingSinceMs_) >= (int32_t)TX_STALL_MS_){
//         txPending_ = false;
//         pollPending_ = false;
//         sendStatus_(pendingPollSeq_, imu, ms);
//       }
//     }
//   }

//   bool linkedNow(uint32_t now) const {
//     uint32_t lh = lastFmsMs_;
//     if(lh == 0) return false;
//     int32_t age = (int32_t)(now - lh);
//     if(age < 0) age = 0;
//     return (age <= (int32_t)LINK_ALIVE_MS_);
//   }

//   bool paired() const { return paired_; }
//   uint8_t wifiChannel() const { return wifiChannel_; }
//   uint32_t lastFmsMs() const { return lastFmsMs_; }

//   uint8_t cmdSysState() const { return cmdSysState_; }
//   uint8_t cmdColor() const { return cmdColor_; }

//   bool consumeIdentifyTrigger(){
//     if(identifyTrigger_){
//       identifyTrigger_ = false;
//       return true;
//     }
//     return false;
//   }

//   uint32_t rxPollCount() const { return rxPollCount_; }
//   uint32_t rxCmdCount()  const { return rxCmdCount_; }
//   uint32_t rxAckCount()  const { return rxAckCount_; }
//   uint32_t txStatusCount() const { return txStatusCount_; }
//   uint32_t txFailCount() const { return txFailCount_; }

// private:
//   static inline EspNowLink* self_ = nullptr;

//   RxQueue* rxq_ = nullptr;

//   uint8_t wifiChannel_ = 1;
//   volatile bool paired_ = false;
//   uint8_t fmsMac_[6]{};

//   uint8_t desiredFms_ = 1;
//   uint32_t pairNonce_ = 0;
//   uint32_t nextPairReqMs_ = 0;
//   uint32_t pairBackoffMs_ = 250;

//   volatile uint32_t lastFmsMs_ = 0;
//   static constexpr uint32_t LINK_ALIVE_MS_ = 2000;
//   static constexpr uint32_t LINK_LOST_MS_  = 4000;

//   uint8_t cmdSysState_ = SYS_STANDBY;
//   uint8_t cmdColor_    = C_WHITE;

//   volatile bool identifyTrigger_ = false;

//   volatile bool pollPending_ = false;
//   volatile uint8_t pendingPollSeq_ = 0;

//   bool txPending_ = false;
//   uint32_t txPendingSinceMs_ = 0;
//   uint8_t lastStatusSeqSent_ = 0xFF;

//   static constexpr uint32_t TX_STALL_MS_ = 30;

//   volatile uint32_t rxPollCount_ = 0, rxCmdCount_ = 0, rxAckCount_ = 0;
//   volatile uint32_t txStatusCount_ = 0, txFailCount_ = 0;
// /*
//   static void onRecvThunk_(const esp_now_recv_info_t* info, const uint8_t* data, int len){
//     if(!self_) return;
//     if(!info || !data || len < 2) return;
//     if(self_->rxq_) (void)self_->rxq_->push_isr(info->src_addr, data, len);
//   }
// */

// static void onRecvThunk_(const uint8_t* mac, const uint8_t* data, int len){
//   if(!self_) return;
//   if(!mac || !data || len < 2) return;
//   if(self_->rxq_) (void)self_->rxq_->push_isr(mac, data, len);
// }
//   static void onSentThunk_(const uint8_t* mac, esp_now_send_status_t status){
//     if(!self_) return;
//     self_->onSent_(mac, status);
//   }

//   void onSent_(const uint8_t* mac, esp_now_send_status_t status){
//     (void)mac;
//     txPending_ = false;
//     if(status != ESP_NOW_SEND_SUCCESS) txFailCount_++;
//   }

//   bool upsertPeer_(const uint8_t* mac){
//     if(!mac) return false;
//     if(esp_now_is_peer_exist(mac)) (void)esp_now_del_peer(mac);

//     esp_now_peer_info_t p{};
//     memcpy(p.peer_addr, mac, 6);
//     p.channel = wifiChannel_;
//     p.encrypt = false;

//     return (esp_now_add_peer(&p) == ESP_OK);
//   }

//   void setChannelAndRebuildPeers_(uint8_t ch){
//     if(ch < 1 || ch > 13) ch = 1;
//     wifiChannel_ = ch;
//     (void)esp_wifi_set_channel(wifiChannel_, WIFI_SECOND_CHAN_NONE);

//     (void)upsertPeer_(BCAST);
//     if(paired_ && !macIsZero(fmsMac_)) (void)upsertPeer_(fmsMac_);
//   }

//   void tickPairing_(uint32_t ms){
//     if(paired_){
//       uint32_t lh = lastFmsMs_;
//       uint32_t age = (lh==0) ? 0xFFFFFFFFu : (uint32_t)(ms - lh);
//       if(age >= LINK_LOST_MS_){
//         paired_ = false;
//         memset(fmsMac_, 0, sizeof(fmsMac_));
//         pairNonce_ = esp_random();
//         lastFmsMs_ = 0;

//         pairBackoffMs_ = 250;
//         nextPairReqMs_ = ms;

//         (void)upsertPeer_(BCAST);

//         CLOG("[CUBE] link lost >=%lums -> restart pairing nonce=%lu\n",
//              (unsigned long)LINK_LOST_MS_, (unsigned long)pairNonce_);
//       }
//     }

//     if(!paired_){
//       if((int32_t)(ms - nextPairReqMs_) >= 0){
//         sendPairReq_();

//         uint32_t jitter = (esp_random() % 40);
//         nextPairReqMs_ = ms + pairBackoffMs_ + jitter;
//         if(pairBackoffMs_ < 1000) pairBackoffMs_ = (pairBackoffMs_ < 800) ? (pairBackoffMs_ + 150) : 1000;
//       }
//     }
//   }

//   void sendPairReq_(){
//     PairReq pr{};
//     pr.proto_ver=PROTO_VER;
//     pr.msg_type=MSG_PAIR_REQ;
//     pr.desired_fms=desiredFms_;
//     pr.nonce=pairNonce_;
//     (void)esp_now_send(BCAST, (uint8_t*)&pr, sizeof(pr));
//   }

//   void handleRxFrameFast_(const RxFrame& f, ImuManager& imu, uint32_t ms){
//     const uint8_t* d = f.data;
//     const int len = (int)f.len;

//     if(len < 2) return;
//     if(d[0] != PROTO_VER) return;

//     const uint8_t mt = d[1];
//     const uint8_t* src = f.src;

//     if(mt == MSG_PAIR_ACK){
//       if(len != (int)sizeof(PairAck)) return;

//       PairAck ack;
//       memcpy(&ack, d, sizeof(ack));
//       rxAckCount_++;

//       if(ack.nonce != pairNonce_) return;
//       if(ack.fms_id != desiredFms_) return;

//       setChannelAndRebuildPeers_(ack.channel);

//       memcpy(fmsMac_, src, 6);
//       (void)upsertPeer_(fmsMac_);
//       paired_ = true;

//       lastFmsMs_ = ms;
//       pairBackoffMs_ = 250;
//       return;
//     }

//     if(!paired_) return;
//     if(!macEq(src, fmsMac_)) return;

//     if(mt == MSG_POLL){
//       if(len != (int)sizeof(PollPkt)) return;

//       rxPollCount_++;
//       const uint8_t seq = d[2];

//       lastFmsMs_ = ms;

//       if(seq == lastStatusSeqSent_) return;

//       pendingPollSeq_ = seq;
//       pollPending_ = true;

//       if(!txPending_){
//         pollPending_ = false;
//         sendStatus_(seq, imu, ms);
//       }
//       return;
//     }

//     if(mt == MSG_CMD){
//       if(len != (int)sizeof(CmdPkt)) return;

//       rxCmdCount_++;

//       CmdPkt c;
//       memcpy(&c, d, sizeof(c));

//       cmdSysState_ = c.sys_state;
//       cmdColor_    = c.color;

//       lastFmsMs_ = ms;

//       if(c.beep_hz == 3000 && c.beep_ms) identifyTrigger_ = true;
//       return;
//     }
//   }

//   void sendStatus_(uint8_t seq, ImuManager& imu, uint32_t ms){
//     if(!paired_ || macIsZero(fmsMac_)) return;

//     StatusPkt st{};
//     st.proto_ver = PROTO_VER;
//     st.msg_type  = MSG_STATUS;
//     st.seq       = seq;
//     st.face      = imu.face();
//     st.stable20  = imu.stable20() ? 1 : 0;
//     st.uptime_s  = (uint32_t)(ms / 1000);

//     txPending_ = true;
//     txPendingSinceMs_ = ms;

//     esp_err_t e = esp_now_send(fmsMac_, (uint8_t*)&st, sizeof(st));
//     if(e != ESP_OK){
//       txPending_ = false;
//       txFailCount_++;
//       return;
//     }

//     txStatusCount_++;
//     lastStatusSeqSent_ = seq;
//   }
// };

// /* ===================== Cube App ===================== */
// class CubeApp {
// public:
//   void begin(){
//     Serial.begin(115200);
//     delay(20);

//     soundIdentify_.begin(PIEZO_PIN, /*ledcChannel*/ 0);
//     leds_.begin();
//     imu_.begin();
//     imu_.primeFace();

//     link_.begin(&rxq_);
//     link_.setDesiredFms(imu_.decideDesiredFmsByStartupFace());

//     cmdSysState_ = SYS_STANDBY;
//     cmdColor_    = C_WHITE;
//     timeGateStartMs_ = 0;

//     CLOG("[CUBE] boot desiredFms=%u wifiCh=%u imu=%u\n",
//          (unsigned)imu_.decideDesiredFmsByStartupFace(),
//          (unsigned)link_.wifiChannel(),
//          (unsigned)imu_.ready());
//   }

//   void loop(){
//     const uint32_t ms = millis();

//     link_.service(imu_, ms);

//     if(link_.consumeIdentifyTrigger()){
//       soundIdentify_.trigger(ms);
//     }

//     imu_.tick(ms);
//     soundIdentify_.tick(ms);

//     cmdSysState_ = link_.cmdSysState();
//     cmdColor_    = link_.cmdColor();

//     if(cmdSysState_ == SYS_TIME_GATE){
//       if(timeGateStartMs_ == 0) timeGateStartMs_ = ms;
//     } else {
//       timeGateStartMs_ = 0;
//     }

//     const bool linked = link_.linkedNow(ms);
//     const bool disconnected = !linked;

//     leds_.tick(ms,
//                soundIdentify_.active(),
//                disconnected,
//                cmdSysState_,
//                cmdColor_,
//                imu_.stable20(),
//                timeGateStartMs_);

// #if CUBE_DEBUG
//     static uint32_t lastDbgMs=0;
//     if(ms - lastDbgMs >= 1000){
//       lastDbgMs = ms;

//       uint32_t lh = link_.lastFmsMs();
//       int32_t age = (lh==0) ? 2147483647 : (int32_t)(ms - lh);
//       if(age < 0) age = 0;

//       CLOG("[CUBE] t=%lu face=%u stable=%u sys=%u linked=%u paired=%u ch=%u ident=%u "
//            "isr=%lu RX(poll=%lu cmd=%lu ack=%lu) TX(status=%lu fail=%lu) rxDrops=%lu lastFmsAge=%ldms\n",
//            (unsigned long)ms,
//            (unsigned)imu_.face(),
//            (unsigned)(imu_.stable20()?1:0),
//            (unsigned)cmdSysState_,
//            (unsigned)(linked?1:0),
//            (unsigned)(link_.paired()?1:0),
//            (unsigned)link_.wifiChannel(),
//            (unsigned)(soundIdentify_.active()?1:0),
//            (unsigned long)imu_.isrCount(),
//            (unsigned long)link_.rxPollCount(),
//            (unsigned long)link_.rxCmdCount(),
//            (unsigned long)link_.rxAckCount(),
//            (unsigned long)link_.txStatusCount(),
//            (unsigned long)link_.txFailCount(),
//            (unsigned long)rxq_.drops(),
//            (long)age);
//     }
// #endif
//   }

// private:
//   RxQueue rxq_;
//   LedManager leds_;
//   ImuManager imu_;
//   IdentifySound soundIdentify_;
//   EspNowLink link_;

//   uint8_t cmdSysState_ = SYS_STANDBY;
//   uint8_t cmdColor_    = C_WHITE;

//   uint32_t timeGateStartMs_ = 0;
// };

// /* ===================== Arduino glue ===================== */
// static CubeApp g_app;

// void setup(){ g_app.begin(); }
// void loop(){ g_app.loop(); }
/********************************************************
 * CubicChaos — Ball (ESP32-S3)
 * Pins:
 *   SCL=1, SDA=2, MPU_INT=3, PIEZO=4, LED=5
 *
 * Gyro-based version:
 * - Ball always stays WHITE
 * - In SYS_IN_GAME:
 *     anim=0 -> still -> renderInGameWhitePattern_()
 *     anim=1 -> moving dir A -> animated white forward
 *     anim=2 -> moving dir B -> animated white reverse
 * - Outside SYS_IN_GAME, existing standby/start/timegate/end/reset
 *   visuals are preserved.
 ********************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <FastLED.h>
#include <math.h>

/* ===================== USER CONFIG ===================== */
#define SCL_PIN      1
#define SDA_PIN      2
#define MPU_INT_PIN  3
#define PIEZO_PIN    4
#define LED_PIN      5

#define FACE_LEDS 3
#define NUM_FACES 6
#define NUM_LEDS  (FACE_LEDS * NUM_FACES)

#define PROTO_VER 5

#ifndef CUBE_DEBUG
#define CUBE_DEBUG 1
#endif

#if CUBE_DEBUG
  #define CLOG(...) do{ Serial.printf(__VA_ARGS__); }while(0)
#else
  #define CLOG(...) do{}while(0)
#endif

/* ===================== Protocol ===================== */
enum MsgType : uint8_t { MSG_PAIR_REQ=1, MSG_PAIR_ACK=2, MSG_POLL=3, MSG_STATUS=4, MSG_CMD=5 };

// System states (match FMS)
enum : uint8_t { SYS_STANDBY=0, SYS_GAME_START=1, SYS_IN_GAME=2, SYS_TIME_GATE=3, SYS_END_GAME=4, SYS_RESET=5 };

// Team+level colors (kept for compatibility; ball will stay white)
#define C_WHITE      0
#define C_BLUE_1     10
#define C_BLUE_2     11
#define C_ORANGE_1   20
#define C_ORANGE_2   21

// Utility colors
#define C_RED        3
#define C_GREEN      4
#define C_PURPLE     5

// Animation codes from FMS
enum : uint8_t {
  ANIM_STILL = 0,
  ANIM_DIR_A = 1,
  ANIM_DIR_B = 2
};

static inline uint8_t levelOf(uint8_t c){
  if(c==C_BLUE_1 || c==C_ORANGE_1) return 1;
  if(c==C_BLUE_2 || c==C_ORANGE_2) return 2;
  return 0;
}

struct __attribute__((packed)) PairReq {
  uint8_t  proto_ver;
  uint8_t  msg_type;
  uint8_t  desired_fms;
  uint32_t nonce;
};

struct __attribute__((packed)) PairAck {
  uint8_t  proto_ver;
  uint8_t  msg_type;
  uint8_t  fms_id;
  uint8_t  channel;
  uint32_t nonce;
};

struct __attribute__((packed)) PollPkt {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t seq;
};

struct __attribute__((packed)) StatusPkt {
  uint8_t  proto_ver;
  uint8_t  msg_type;
  uint8_t  seq;
  int8_t   gyroX;
  int8_t   gyroY;
  int8_t   gyroZ;
  uint8_t  moving;
  uint32_t uptime_s;
};

struct __attribute__((packed)) CmdPkt {
  uint8_t  proto_ver;
  uint8_t  msg_type;
  uint8_t  sys_state;
  uint8_t  color;
  uint8_t  anim;
  uint16_t beep_hz;
  uint16_t beep_ms;
};

static_assert(sizeof(PairReq)  == 1+1+1+4,    "PairReq size mismatch");
static_assert(sizeof(PairAck)  == 1+1+1+1+4,  "PairAck size mismatch");
static_assert(sizeof(PollPkt)  == 1+1+1,      "PollPkt size mismatch");
static_assert(sizeof(StatusPkt)== 1+1+1+1+1+1+1+4, "StatusPkt size mismatch");
static_assert(sizeof(CmdPkt)   == 1+1+1+1+1+2+2,   "CmdPkt size mismatch");

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static inline bool macEq(const uint8_t a[6], const uint8_t b[6]) { return memcmp(a,b,6)==0; }
static inline bool macIsZero(const uint8_t a[6]) { return (a[0]|a[1]|a[2]|a[3]|a[4]|a[5]) == 0; }

/* ===================== RX Queue (ISR-safe ring buffer) ===================== */
struct RxFrame {
  uint8_t  src[6];
  uint8_t  len;
  uint8_t  data[32];
};

class RxQueue {
public:
  static constexpr uint8_t  CAP   = 16;
  static constexpr uint16_t MAXRX = 32;

  bool push_isr(const uint8_t* src, const uint8_t* data, int len){
    if(!src || !data) return false;
    if(len <= 0 || len > (int)MAXRX) return false;

    bool ok = false;
    portENTER_CRITICAL_ISR(&mux_);

    uint8_t next = (uint8_t)(head_ + 1);
    if(next >= CAP) next = 0;

    if(next == tail_){
      drops_++;
      ok = false;
    } else {
      RxFrame& f = q_[head_];
      memcpy(f.src, src, 6);
      f.len = (uint8_t)len;
      memcpy(f.data, data, (size_t)len);
      head_ = next;
      ok = true;
    }

    portEXIT_CRITICAL_ISR(&mux_);
    return ok;
  }

  bool pop(RxFrame& out){
    bool ok = false;

    portENTER_CRITICAL(&mux_);
    if(tail_ != head_){
      out = q_[tail_];
      uint8_t next = (uint8_t)(tail_ + 1);
      if(next >= CAP) next = 0;
      tail_ = next;
      ok = true;
    }
    portEXIT_CRITICAL(&mux_);

    return ok;
  }

  uint32_t drops() const { return drops_; }

private:
  RxFrame q_[CAP]{};
  volatile uint8_t head_ = 0;
  volatile uint8_t tail_ = 0;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t drops_ = 0;
};

/* ===================== Sound (LEDC, packaged + independent) ===================== */
class LedcToneOut {
public:
  void begin(uint8_t pin, uint8_t ledcChannel){
    _pin = pin;
    _ch = ledcChannel;

    pinMode(_pin, OUTPUT);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    (void)_ch;
    ledcAttach(_pin, 1000 /*placeholder*/, 10 /*bits*/);
#else
    ledcSetup(_ch, 1000 /*placeholder*/, 10 /*bits*/);
    ledcAttachPin(_pin, _ch);
#endif
    stop();
  }

  void tick(uint32_t ms){
    if(_active && _durMs){
      if((int32_t)(ms - _stopAtMs) >= 0) stop();
    }
  }

  void play(uint16_t hz, uint16_t durationMs, uint16_t duty /*0..1023*/){
    if(hz == 0){ stop(); return; }

    _active = true;
    _durMs  = durationMs;
    _stopAtMs = (durationMs ? (millis() + durationMs) : 0);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteTone(_pin, hz);
    ledcWrite(_pin, clampDuty10_(duty));
#else
    ledcWriteTone(_ch, hz);
    ledcWrite(_ch, clampDuty10_(duty));
#endif
  }

  void stop(){
    _active = false;
    _durMs  = 0;
    _stopAtMs = 0;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(_pin, 0);
    ledcWriteTone(_pin, 0);
#else
    ledcWrite(_ch, 0);
    ledcWriteTone(_ch, 0);
#endif
  }

  bool active() const { return _active; }

private:
  uint8_t _pin = 0;
  uint8_t _ch  = 0;

  bool _active = false;
  uint16_t _durMs = 0;
  uint32_t _stopAtMs = 0;

  static uint16_t clampDuty10_(uint16_t d){ return (d > 1023) ? 1023 : d; }
};

class IdentifySound {
public:
  void begin(uint8_t pin, uint8_t ledcChannel){
    _tone.begin(pin, ledcChannel);
    reset_();
  }

  void trigger(uint32_t now){
    _active = true;
    _untilMs = now + 2000;
    _nextBeepMs = now;
    _beepCount = 0;
  }

  void tick(uint32_t ms){
    _tone.tick(ms);

    if(!_active) return;

    if((int32_t)(ms - _untilMs) >= 0){
      reset_();
      _tone.stop();
      return;
    }

    if(_beepCount < 3 && (int32_t)(ms - _nextBeepMs) >= 0){
      _tone.play(3000, 70, 512);
      _beepCount++;
      _nextBeepMs = ms + 333;
    }
  }

  bool active() const { return _active; }

private:
  LedcToneOut _tone;
  bool _active = false;
  uint32_t _untilMs = 0;
  uint32_t _nextBeepMs = 0;
  uint8_t _beepCount = 0;

  void reset_(){
    _active = false;
    _untilMs = 0;
    _nextBeepMs = 0;
    _beepCount = 0;
  }
};

/* ===================== IMU IRQ (IRAM-safe, no object refs) ===================== */
static volatile bool     g_imu_drdy = false;
static volatile uint32_t g_imu_isrCount = 0;

void IRAM_ATTR onImuInt_ISR(){
  g_imu_drdy = true;
  g_imu_isrCount++;
}

/* ===================== IMU Manager ===================== */
class ImuManager {
public:
  void begin(){
    Wire.begin(SDA_PIN, SCL_PIN);
    imuInit_();

    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onImuInt_ISR, FALLING);

    calibrateGyroBias_();
  }

  void primeFace(){
    // Preserved so desired-FMS boot logic still works.
    curFace_ = 0;
    stable20_ = true;
    delay(40);
    for(int i=0;i<3;i++){ curFace_ = detectFace20_(curFace_); delay(20); }
    lastFaceMs_ = millis();
    lastGyroMs_ = millis();
  }

  void tick(uint32_t ms){
    if(ms - lastFaceMs_ >= 40){
      lastFaceMs_ = ms;
      curFace_ = detectFace20_(curFace_);
    }

    if(ms - lastGyroMs_ >= 20){
      lastGyroMs_ = ms;
      updateGyroMotion_();
    }
  }

  uint8_t face() const { return curFace_; }
  bool stable20() const { return stable20_; }
  bool ready() const { return imu_ready_; }
  uint32_t isrCount() const { return g_imu_isrCount; }
  uint8_t decideDesiredFmsByStartupFace() const { return (curFace_==0) ? 1 : 2; }

  int8_t gyroX() const { return (int8_t)constrain((int)lroundf(filtGX_), -127, 127); }
  int8_t gyroY() const { return (int8_t)constrain((int)lroundf(filtGY_), -127, 127); }
  int8_t gyroZ() const { return (int8_t)constrain((int)lroundf(filtGZ_), -127, 127); }
  bool moving() const { return moving_; }

private:
  MPU6050 mpu_;
  bool imu_ready_ = false;

  uint8_t curFace_ = 0;
  bool stable20_ = true;
  uint32_t lastFaceMs_ = 0;
  uint32_t lastGyroMs_ = 0;

  float gyroBiasX_ = 0.0f;
  float gyroBiasY_ = 0.0f;
  float gyroBiasZ_ = 0.0f;

  float filtGX_ = 0.0f;
  float filtGY_ = 0.0f;
  float filtGZ_ = 0.0f;
  bool moving_ = false;

  static const int8_t Rm_[3][3];

  void imuInit_(){
    mpu_.initialize();
    mpu_.setSleepEnabled(false);
    mpu_.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu_.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    #ifdef MPU6050_DLPF_BW_10
      mpu_.setDLPFMode(MPU6050_DLPF_BW_10);
    #else
      mpu_.setDLPFMode(5);
    #endif

    mpu_.setInterruptLatch(true);
    mpu_.setInterruptLatchClear(true);
    mpu_.setIntDataReadyEnabled(true);

    delay(10);
    imu_ready_ = mpu_.testConnection();
  }

  void calibrateGyroBias_(){
    if(!imu_ready_) return;

    const int N = 200;
    long sx = 0, sy = 0, sz = 0;

    delay(50);
    for(int i = 0; i < N; ++i){
      int16_t gx, gy, gz;
      mpu_.getRotation(&gx, &gy, &gz);
      sx += gx;
      sy += gy;
      sz += gz;
      delay(3);
    }

    gyroBiasX_ = (float)sx / (float)N / 131.0f;
    gyroBiasY_ = (float)sy / (float)N / 131.0f;
    gyroBiasZ_ = (float)sz / (float)N / 131.0f;
  }

  void updateGyroMotion_(){
    if(!imu_ready_) return;

    int16_t gxRaw, gyRaw, gzRaw;
    mpu_.getRotation(&gxRaw, &gyRaw, &gzRaw);

    // MPU6050 at FS=250dps -> 131 LSB / deg/s
    float gx = ((float)gxRaw / 131.0f) - gyroBiasX_;
    float gy = ((float)gyRaw / 131.0f) - gyroBiasY_;
    float gz = ((float)gzRaw / 131.0f) - gyroBiasZ_;

    // light low-pass
    const float alpha = 0.35f;
    filtGX_ = filtGX_ + alpha * (gx - filtGX_);
    filtGY_ = filtGY_ + alpha * (gy - filtGY_);
    filtGZ_ = filtGZ_ + alpha * (gz - filtGZ_);

    const float moveThresh = 10.0f; // deg/s ; tune if needed
    moving_ =
      (fabsf(filtGX_) > moveThresh) ||
      (fabsf(filtGY_) > moveThresh) ||
      (fabsf(filtGZ_) > moveThresh);
  }

  bool readAccelUnit_(float& x, float& y, float& z){
    if(!imu_ready_) return false;

    const uint8_t  N = 10;
    const uint32_t MAX_US = 6000;
    const uint32_t t0 = micros();

    long sx=0, sy=0, sz=0;
    uint8_t got=0;

    uint32_t lastFallbackUs = t0;

    while(got < N && (micros() - t0) < MAX_US){
      bool take = false;

      if(g_imu_drdy){
        g_imu_drdy = false;
        take = true;
      } else {
        uint32_t nowUs = micros();
        if((nowUs - lastFallbackUs) >= 900){
          lastFallbackUs = nowUs;
          take = true;
        }
      }
      if(!take) continue;

      int16_t ax,ay,az;
      mpu_.getAcceleration(&ax,&ay,&az);
      (void)mpu_.getIntStatus();

      int32_t rx = (int32_t)Rm_[0][0]*ax + (int32_t)Rm_[0][1]*ay + (int32_t)Rm_[0][2]*az;
      int32_t ry = (int32_t)Rm_[1][0]*ax + (int32_t)Rm_[1][1]*ay + (int32_t)Rm_[1][2]*az;
      int32_t rz = (int32_t)Rm_[2][0]*ax + (int32_t)Rm_[2][1]*ay + (int32_t)Rm_[2][2]*az;

      sx+=rx; sy+=ry; sz+=rz; got++;
    }

    if(got < 4) return false;

    float fx=sx/(float)got, fy=sy/(float)got, fz=sz/(float)got;
    float mag=sqrtf(fx*fx + fy*fy + fz*fz);
    if(!(mag > 1e-3f)) return false;

    x=fx/mag; y=fy/mag; z=fz/mag;
    return true;
  }

  uint8_t detectFace20_(uint8_t prevFace){
    float x,y,z;
    if(!readAccelUnit_(x,y,z)) return prevFace;

    float dots[6]={ x,-x, y,-y, z,-z };
    int best=0; float bdot=dots[0];
    for(int i=1;i<6;i++){ if(dots[i]>bdot){ bdot=dots[i]; best=i; } }

    const float ENTER_COS = 0.9396926f;
    stable20_ = (bdot >= ENTER_COS);

    if(prevFace>5) return (uint8_t)best;
    if(best!=prevFace && bdot>=ENTER_COS) return (uint8_t)best;
    return prevFace;
  }
};

const int8_t ImuManager::Rm_[3][3] = {
  {+1, 0, 0},
  { 0,+1, 0},
  { 0, 0,+1}
};

/* ===================== LED Management ===================== */
class LedManager {
public:
  void begin(){
    
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds_, NUM_LEDS);
  FastLED.setCorrection(UncorrectedColor);
  FastLED.setDither(0);
  FastLED.setBrightness(brightness_);

  // Initialize animation state
  patternOffset_ = 0;
  lastUpdate_ = 0;

  // Boot rainbow animation
  Serial.println("LED BEGIN / RAINBOW");
  for (uint8_t step = 0; step < 32; step++) {
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      leds_[i] = CHSV((uint8_t)(step * 8 + i * 10), 255, 180);
    }
    FastLED.show();
    delay(25);
  }

  clearAll_();
  FastLED.show();

  rp_target_ = (uint8_t)(esp_random() % NUM_LEDS);
  rp_nextMs_ = millis() + 500;

  lastRenderMs_ = 0;
  lastShowMs_ = 0;
  dirty_ = true;

  endgameValid_ = false;
  endgameStartMs_ = 0;
  endgameEntryColor_ = C_WHITE;
  endgameEntryUnstable_ = false;
}
  void tick(uint32_t ms,
            bool identifyActive,
            bool disconnected,
            uint8_t sysState,
            uint8_t teamColor,
            bool stable20,
            uint32_t timeGateStartMs,
            uint8_t animMode){
    if(sysState != lastSysStateTransition_){
      if(sysState == SYS_END_GAME){
        snapshotEndgameEntry_(ms, teamColor, stable20);
      } else {
        endgameValid_ = false;
        endgameStartMs_ = 0;
        endgameEntryColor_ = C_WHITE;
        endgameEntryUnstable_ = false;
      }
      lastSysStateTransition_ = sysState;
      dirty_ = true;
    }

    if((int32_t)(ms - lastRenderMs_) < (int32_t)RENDER_PERIOD_MS_){
      if(dirty_ && (int32_t)(ms - lastShowMs_) >= (int32_t)SHOW_PERIOD_MS_){
        FastLED.show();
        lastShowMs_ = ms;
        dirty_ = false;
      }
      return;
    }
    lastRenderMs_ = ms;

    const bool animating =
      identifyActive ||
      (sysState == SYS_STANDBY) ||
      (sysState == SYS_TIME_GATE && timeGateStartMs && (ms - timeGateStartMs <= 500)) ||
      (sysState == SYS_END_GAME) ||
      (sysState == SYS_IN_GAME && animMode != ANIM_STILL) ||
      (disconnected);

    if(!animating &&
       !dirty_ &&
       sysState == lastSysStateLatch_ &&
       teamColor == lastTeamColor_ &&
       identifyActive == lastIdentify_ &&
       disconnected == lastDisconnected_ &&
       stable20 == lastStable_ &&
       animMode == lastAnimMode_){
      return;
    }

    lastSysStateLatch_ = sysState;
    lastTeamColor_ = teamColor;
    lastIdentify_ = identifyActive;
    lastDisconnected_ = disconnected;
    lastStable_ = stable20;
    lastAnimMode_ = animMode;

    clearAll_();

    if(identifyActive){
      const bool on = flash5Hz_(ms);
      renderSolidAllFaces_(on ? C_GREEN : C_PURPLE);

    } else if(sysState == SYS_STANDBY){
      renderStandbyTwoPixel_(ms);

    } else if(sysState == SYS_TIME_GATE){
      if(timeGateStartMs && (ms - timeGateStartMs <= 500)){
        for(uint8_t i=0;i<NUM_LEDS;i++){
          leds_[i] = CHSV((uint8_t)((ms/3) + i*10), 255, 180);
        }
      } else {
        // Ball is effectively white forever; keep white here.
        renderInGameWhitePattern_();
      }

    } else if(sysState == SYS_RESET){
      renderSolidAllFaces_(C_PURPLE);

    } else if(sysState == SYS_GAME_START){
      renderSolidAllFaces_(C_GREEN);

    } else if(sysState == SYS_IN_GAME){
      // Always white in game.
      if(animMode == ANIM_DIR_A){
        renderInGameWhitePatternAnimated_Forward_();
      } else if(animMode == ANIM_DIR_B){
        renderInGameWhitePatternAnimated_Reverse_();
      } else {
        renderInGameWhitePattern_();
      }

    } else if(sysState == SYS_END_GAME){
      if(!endgameValid_) snapshotEndgameEntry_(ms, teamColor, stable20);

      const uint32_t dt = (endgameStartMs_ == 0) ? 0 : (ms - endgameStartMs_);

      if(dt < 2000){
        renderSolidAllFaces_(C_RED);
      } else {
        clearAll_();

        if(endgameEntryColor_ != C_WHITE){
          if(endgameEntryUnstable_){
            if(flash5Hz_(ms)){
              renderTeamLevelPattern_(endgameEntryColor_);
            }
          } else {
            renderTeamLevelPattern_(endgameEntryColor_);
          }
        }

        const uint32_t t = dt - 2000;
        const float pos = fmodf((float)t * 0.006f, (float)NUM_LEDS);
        int i0 = (int)floorf(pos);
        float frac = pos - (float)i0;
        if(i0 < 0) i0 = 0;
        const int i1 = (i0 + 1) % NUM_LEDS;

        const uint8_t v0 = (uint8_t)(170.0f * (1.0f - frac));
        const uint8_t v1 = (uint8_t)(170.0f * (frac));

        overlayOppPairPureRed_(i0, v0);
        overlayOppPairPureRed_(i1, v1);
      }

    } else {
      renderStandbyTwoPixel_(ms);
    }

    if(disconnected && !identifyActive){
      const bool on = ((ms/250) & 1) == 0;
      applyCenterAllFaces_(on ? CRGB(100,0,0) : CRGB::Black);
    }

    dirty_ = true;

    if((int32_t)(ms - lastShowMs_) >= (int32_t)SHOW_PERIOD_MS_){
      FastLED.show();
      lastShowMs_ = ms;
      dirty_ = false;
    }
  }

private:
  CRGB leds_[NUM_LEDS];

  static constexpr uint8_t  TARGET_FPS_ = 120;
  static constexpr uint32_t RENDER_PERIOD_MS_ = (1000 / TARGET_FPS_);
  static constexpr uint32_t SHOW_PERIOD_MS_   = (1000 / TARGET_FPS_);

  uint8_t brightness_ = 80;

  uint32_t lastRenderMs_ = 0;
  uint32_t lastShowMs_ = 0;
  bool dirty_ = true;

  uint8_t lastSysStateTransition_ = 0xFF;
  uint8_t lastSysStateLatch_ = 0xFF;
  uint8_t lastTeamColor_ = 0xFF;
  bool lastIdentify_ = false;
  bool lastDisconnected_ = false;
  bool lastStable_ = true;
  uint8_t lastAnimMode_ = 0xFF;

  uint8_t rp_target_ = 0;
  uint32_t rp_nextMs_ = 0;

  bool endgameValid_ = false;
  uint32_t endgameStartMs_ = 0;
  uint8_t endgameEntryColor_ = C_WHITE;
  bool endgameEntryUnstable_ = false;

  // NEW: animated white pattern state
  uint8_t patternOffset_ = 0;
  uint32_t lastUpdate_ = 0;
  static constexpr uint16_t interval_ = 100;

  static inline uint8_t faceBase_(uint8_t f){ static const uint8_t base[6]={0,6,3,12,9,15}; return base[f]; }
  static inline uint8_t faceCenterIdx_(uint8_t f){ return faceBase_(f)+1; }
  static inline uint8_t faceOpp_(uint8_t f){ static const uint8_t o[6]={1,0,3,2,5,4}; return o[f]; }

  static inline void decodeIdx_(uint8_t idx, uint8_t& f, uint8_t& off){
    for(uint8_t i=0;i<6;i++){
      uint8_t b=faceBase_(i);
      if(idx>=b && idx<b+FACE_LEDS){ f=i; off=idx-b; return; }
    }
    f=0; off=0;
  }

  static inline bool flash5Hz_(uint32_t ms){ return ((ms/100) & 1) == 0; }

  static CRGB colorToCRGB_(uint8_t c){
    switch(c){
      case C_WHITE:    return CRGB(80,90,90);
      case C_BLUE_1:
      case C_BLUE_2:   return CRGB(0,0,160);
      case C_ORANGE_1:
      case C_ORANGE_2: return CRGB(170,40,0);
      case C_RED:      return CRGB(150,0,0);
      case C_GREEN:    return CRGB(0,150,0);
      case C_PURPLE:   return CRGB(100,0,150);
      default:         return CRGB::Black;
    }
  }

  void clearAll_(){ memset(leds_, 0, sizeof(leds_)); }

  void renderSolidAllFaces_(uint8_t color){
    const CRGB c = colorToCRGB_(color);
    for(uint8_t f=0; f<NUM_FACES; f++){
      const uint8_t b = faceBase_(f);
      leds_[b]   = c;
      leds_[b+1] = c;
      leds_[b+2] = c;
    }
  }

  void applyCenterAllFaces_(CRGB centerColor){
    for(uint8_t f=0; f<NUM_FACES; f++){
      leds_[faceCenterIdx_(f)] = centerColor;
    }
  }

  void renderTeamLevelPattern_(uint8_t teamLevelColor){
    const CRGB c = colorToCRGB_(teamLevelColor);
    const uint8_t lvl = levelOf(teamLevelColor);

    for(uint8_t f=0; f<NUM_FACES; f++){
      const uint8_t b = faceBase_(f);

      if(lvl == 1){
        leds_[b]   = c;
        leds_[b+1] = CRGB::Black;
        leds_[b+2] = CRGB::Black;
      } else if(lvl == 2){
        leds_[b]   = c;
        leds_[b+1] = CRGB::Black;
        leds_[b+2] = c;
      } else {
        leds_[b]   = CRGB::Black;
        leds_[b+1] = CRGB::Black;
        leds_[b+2] = CRGB::Black;
      }
    }
  }

  void renderInGameWhitePattern_(){
    const CRGB c = colorToCRGB_(C_WHITE);
    for(uint8_t f=0; f<NUM_FACES; f++){
      const uint8_t b = faceBase_(f);
      leds_[b]   = c;
      leds_[b+1] = CRGB::Black;
      leds_[b+2] = c;
    }
  }

  void renderPatternCore_(){
    for(uint8_t f = 0; f < NUM_FACES; f++){
      const uint8_t b = faceBase_(f);

      uint8_t levels[3] = {0, 0, 0};
      levels[patternOffset_] = 255;
      levels[(patternOffset_ + 1) % 3] = 128;
      levels[(patternOffset_ + 2) % 3] = 0;

      for(uint8_t i = 0; i < 3; i++){
        leds_[b + i] = CRGB(levels[i], levels[i], levels[i]);
      }
    }
  }

  void renderInGameWhitePatternAnimated_Forward_(){
    if(millis() - lastUpdate_ >= interval_){
      lastUpdate_ = millis();
      patternOffset_ = (patternOffset_ + 1) % 3;
    }
    renderPatternCore_();
  }

  void renderInGameWhitePatternAnimated_Reverse_(){
    if(millis() - lastUpdate_ >= interval_){
      lastUpdate_ = millis();
      patternOffset_ = (patternOffset_ + 2) % 3;
    }
    renderPatternCore_();
  }

  void renderStandbyTwoPixel_(uint32_t ms){
    if((int32_t)(ms - rp_nextMs_) >= 0){
      uint8_t next = rp_target_;
      while(next == rp_target_) next = (uint8_t)(esp_random() % NUM_LEDS);
      rp_target_ = next;
      rp_nextMs_ = ms + 500;
    }

    leds_[rp_target_] = CRGB(70,70,70);

    uint8_t f, off;
    decodeIdx_(rp_target_, f, off);
    const uint8_t of = faceOpp_(f);
    leds_[faceBase_(of) + off] = CRGB(70,70,70);
  }

  void overlayOppPairPureRed_(uint8_t idx, uint8_t v){
    if(v == 0) return;

    uint8_t f, off;
    decodeIdx_(idx, f, off);

    leds_[idx] = CRGB(v, 0, 0);

    const uint8_t opp = faceBase_(faceOpp_(f)) + off;
    leds_[opp] = CRGB(v, 0, 0);
  }

  void snapshotEndgameEntry_(uint32_t ms, uint8_t teamColor, bool stable20){
    endgameValid_ = true;
    endgameStartMs_ = ms;
    endgameEntryColor_ = teamColor;
    endgameEntryUnstable_ = !stable20;
  }
};

/* ===================== Link Management ===================== */
class LinkManager {
public:
  void begin(RxQueue* q){
    q_ = q;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);

    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(wifiChannel_, WIFI_SECOND_CHAN_NONE);

    if(esp_now_init() != ESP_OK){
      delay(100);
      ESP.restart();
    }

    esp_now_register_recv_cb(onRecv_);
    esp_now_register_send_cb(onSent_);

    esp_now_peer_info_t p{};
    memcpy(p.peer_addr, BCAST, 6);
    p.channel = wifiChannel_;
    p.encrypt = false;
    esp_now_add_peer(&p);

    instance_ = this;
    lastPairAttemptMs_ = 0;
    nextPairBackoffMs_ = 300;
  }

  void setDesiredFms(uint8_t fms){ desiredFms_ = (fms == 2) ? 2 : 1; }

  void service(ImuManager& imu, uint32_t ms){
    drainRx_(imu, ms);

    if(!paired_){
      if((int32_t)(ms - lastPairAttemptMs_) >= 0){
        sendPairReq_(ms);
        lastPairAttemptMs_ = ms + nextPairBackoffMs_;
        if(nextPairBackoffMs_ < 2000) nextPairBackoffMs_ = (uint16_t)min<uint32_t>(2000, nextPairBackoffMs_ * 2);
      }
      return;
    }

    if(txPending_ && (int32_t)(ms - txPendingSinceMs_) > 50){
      txPending_ = false;
      txTimeoutCount_++;
    }

    if((int32_t)(ms - lastFmsRxMs_) > 3000){
      paired_ = false;
      pairNonce_ = 0;
      clearFmsPeer_();
      nextPairBackoffMs_ = 250;
      lastPairAttemptMs_ = 0;
    }
  }

  bool linkedNow(uint32_t ms) const {
    return paired_ && ((int32_t)(ms - lastFmsRxMs_) <= 1500);
  }

  uint8_t cmdSysState() const { return cmdSysState_; }
  uint8_t cmdColor() const { return cmdColor_; }
  uint8_t cmdAnim() const { return cmdAnim_; }
  uint8_t wifiChannel() const { return wifiChannel_; }
  uint32_t lastFmsMs() const { return lastFmsRxMs_; }

  bool consumeIdentifyTrigger(){
    bool v = identifyTrigger_;
    identifyTrigger_ = false;
    return v;
  }

private:
  static LinkManager* instance_;
  RxQueue* q_ = nullptr;

  uint8_t desiredFms_ = 1;
  uint8_t wifiChannel_ = 1;

  bool paired_ = false;
  uint32_t pairNonce_ = 0;
  uint8_t fmsMac_[6]{0};

  uint32_t lastPairAttemptMs_ = 0;
  uint16_t nextPairBackoffMs_ = 300;

  uint32_t lastFmsRxMs_ = 0;

  uint8_t cmdSysState_ = SYS_STANDBY;
  uint8_t cmdColor_    = C_WHITE;
  uint8_t cmdAnim_     = 0;

  bool identifyTrigger_ = false;

  bool txPending_ = false;
  uint32_t txPendingSinceMs_ = 0;

  uint32_t txStatusCount_ = 0;
  uint32_t txFailCount_ = 0;
  uint32_t txTimeoutCount_ = 0;
  uint8_t lastStatusSeqSent_ = 0;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  static void onRecv_(const esp_now_recv_info_t* info, const uint8_t* data, int len){
    if(!instance_) return;
    if(!info || !data || len < 2) return;
    if(instance_->q_) (void)instance_->q_->push_isr(info->src_addr, data, len);
  }

  static void onSent_(const wifi_tx_info_t* tx_info, esp_now_send_status_t status){
    if(!instance_) return;
    (void)tx_info;
    instance_->txPending_ = false;
    if(status != ESP_NOW_SEND_SUCCESS) instance_->txFailCount_++;
  }
#else
  static void onRecv_(const uint8_t* mac, const uint8_t* data, int len){
    if(!instance_) return;
    if(!mac || !data || len < 2) return;
    if(instance_->q_) (void)instance_->q_->push_isr(mac, data, len);
  }

  static void onSent_(const uint8_t* mac, esp_now_send_status_t status){
    if(!instance_) return;
    (void)mac;
    instance_->txPending_ = false;
    if(status != ESP_NOW_SEND_SUCCESS) instance_->txFailCount_++;
  }
#endif

  void clearFmsPeer_(){
    if(!macIsZero(fmsMac_) && esp_now_is_peer_exist(fmsMac_)){
      esp_now_del_peer(fmsMac_);
    }
    memset(fmsMac_, 0, sizeof(fmsMac_));
  }

  void upsertFmsPeer_(const uint8_t mac[6], uint8_t channel){
    clearFmsPeer_();

    esp_now_peer_info_t p{};
    memcpy(p.peer_addr, mac, 6);
    p.channel = channel;
    p.encrypt = false;
    esp_now_add_peer(&p);

    memcpy(fmsMac_, mac, 6);
  }

  void sendPairReq_(uint32_t ms){
    PairReq req{};
    req.proto_ver = PROTO_VER;
    req.msg_type = MSG_PAIR_REQ;
    req.desired_fms = desiredFms_;

    if(pairNonce_ == 0){
      pairNonce_ = (uint32_t)esp_random();
      if(pairNonce_ == 0) pairNonce_ = 1;
    }
    req.nonce = pairNonce_;

    esp_err_t e = esp_now_send(BCAST, (uint8_t*)&req, sizeof(req));
    if(e != ESP_OK){
      (void)ms;
    }
  }

  void drainRx_(ImuManager& imu, uint32_t ms){
    RxFrame f;
    while(q_ && q_->pop(f)){
      if(f.len < 2) continue;
      const uint8_t proto = f.data[0];
      const uint8_t type  = f.data[1];
      if(proto != PROTO_VER) continue;

      if(type == MSG_PAIR_ACK){
        if((size_t)f.len != sizeof(PairAck)) continue;
        PairAck ack{};
        memcpy(&ack, f.data, sizeof(ack));

        if(ack.fms_id != desiredFms_) continue;
        if(ack.nonce != pairNonce_) continue;

        wifiChannel_ = ack.channel;
        esp_wifi_set_channel(wifiChannel_, WIFI_SECOND_CHAN_NONE);
        upsertFmsPeer_(f.src, wifiChannel_);

        paired_ = true;
        lastFmsRxMs_ = ms;
        cmdSysState_ = SYS_STANDBY;
        cmdColor_ = C_WHITE;
        cmdAnim_ = 0;
        continue;
      }

      if(type == MSG_POLL){
        if((size_t)f.len != sizeof(PollPkt)) continue;
        if(!paired_) continue;
        if(!macEq(f.src, fmsMac_)) continue;

        PollPkt p{};
        memcpy(&p, f.data, sizeof(p));

        lastFmsRxMs_ = ms;
        sendStatus_(p.seq, imu, ms);
        continue;
      }

      if(type == MSG_CMD){
        if((size_t)f.len != sizeof(CmdPkt)) continue;
        if(!paired_) continue;
        if(!macEq(f.src, fmsMac_)) continue;

        CmdPkt c{};
        memcpy(&c, f.data, sizeof(c));

        lastFmsRxMs_ = ms;
        cmdSysState_ = c.sys_state;
        cmdColor_    = C_WHITE;   // forced white
        cmdAnim_     = c.anim;

        if(c.beep_hz && c.beep_ms) identifyTrigger_ = true;
        continue;
      }
    }
  }

  void sendStatus_(uint8_t seq, ImuManager& imu, uint32_t ms){
    if(!paired_ || macIsZero(fmsMac_)) return;

    StatusPkt st{};
    st.proto_ver = PROTO_VER;
    st.msg_type  = MSG_STATUS;
    st.seq       = seq;
    st.gyroX     = imu.gyroX();
    st.gyroY     = imu.gyroY();
    st.gyroZ     = imu.gyroZ();
    st.moving    = imu.moving() ? 1 : 0;
    st.uptime_s  = (uint32_t)(ms / 1000);

    txPending_ = true;
    txPendingSinceMs_ = ms;

    esp_err_t e = esp_now_send(fmsMac_, (uint8_t*)&st, sizeof(st));
    if(e != ESP_OK){
      txPending_ = false;
      txFailCount_++;
      return;
    }

    txStatusCount_++;
    lastStatusSeqSent_ = seq;
  }
};

LinkManager* LinkManager::instance_ = nullptr;

/* ===================== Ball App ===================== */
class CubeApp {
public:
  void begin(){
    Serial.begin(115200);
    delay(20);
    Serial.println("BALL NEW CODE START");

    soundIdentify_.begin(PIEZO_PIN, /*ledcChannel*/ 0);
    leds_.begin();
    imu_.begin();
    imu_.primeFace();

    link_.begin(&rxq_);
    link_.setDesiredFms(imu_.decideDesiredFmsByStartupFace());

    cmdSysState_ = SYS_STANDBY;
    cmdColor_    = C_WHITE;
    cmdAnim_     = 0;
    timeGateStartMs_ = 0;

    CLOG("[BALL] boot desiredFms=%u wifiCh=%u imu=%u\n",
         (unsigned)imu_.decideDesiredFmsByStartupFace(),
         (unsigned)link_.wifiChannel(),
         (unsigned)imu_.ready());
  }

  void loop(){
    const uint32_t ms = millis();

    link_.service(imu_, ms);

    if(link_.consumeIdentifyTrigger()){
      soundIdentify_.trigger(ms);
    }

    imu_.tick(ms);
    soundIdentify_.tick(ms);

    cmdSysState_ = link_.cmdSysState();
    cmdColor_    = C_WHITE;
    cmdAnim_     = link_.cmdAnim();

    if(cmdSysState_ == SYS_TIME_GATE){
      if(timeGateStartMs_ == 0) timeGateStartMs_ = ms;
    } else {
      timeGateStartMs_ = 0;
    }

    const bool linked = link_.linkedNow(ms);
    const bool disconnected = !linked;

    leds_.tick(ms,
               soundIdentify_.active(),
               disconnected,
               cmdSysState_,
               cmdColor_,
               imu_.stable20(),
               timeGateStartMs_,
               cmdAnim_);

#if CUBE_DEBUG
    static uint32_t lastDbgMs = 0;
    if(ms - lastDbgMs >= 1000){
      lastDbgMs = ms;

      uint32_t lh = link_.lastFmsMs();
      int32_t age = (lh==0) ? 2147483647 : (int32_t)(ms - lh);
      if(age < 0) age = 0;

      CLOG("[BALL] linked=%u age=%ld state=%u anim=%u gyro=(%d,%d,%d) moving=%u\n",
           (unsigned)linked,
           (long)age,
           (unsigned)cmdSysState_,
           (unsigned)cmdAnim_,
           (int)imu_.gyroX(),
           (int)imu_.gyroY(),
           (int)imu_.gyroZ(),
           (unsigned)imu_.moving());
    }
#endif
  }

private:
  RxQueue rxq_;
  ImuManager imu_;
  LedManager leds_;
  IdentifySound soundIdentify_;
  LinkManager link_;

  uint8_t cmdSysState_ = SYS_STANDBY;
  uint8_t cmdColor_ = C_WHITE;
  uint8_t cmdAnim_ = 0;
  uint32_t timeGateStartMs_ = 0;
};

static CubeApp app;

void setup(){
  app.begin();
}

void loop(){
  app.loop();
}