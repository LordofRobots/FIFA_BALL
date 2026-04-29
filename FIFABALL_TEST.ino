#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <FastLED.h>
#include <math.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
/* ===================== USER CONFIG ===================== */
#define SCL_PIN      1
#define SDA_PIN      2
#define MPU_INT_PIN  3
#define PIEZO_PIN    4
#define LED_PIN      5

#define NUM_BLOCKS 4
#define LEDS_PER_BLOCK 11
#define NUM_LEDS (NUM_BLOCKS * LEDS_PER_BLOCK)

#define PROTO_VER 5

#ifndef CUBE_DEBUG
#define CUBE_DEBUG 1
#endif

#if CUBE_DEBUG
  #define CLOG(...) do{ Serial.printf(__VA_ARGS__); }while(0)
#else
  #define CLOG(...) do{}while(0)
#endif

const char* OTA_SSID = "minibot_fms";
const char* OTA_PASS = "abcdefgh";

bool otaReady_ = false;
uint32_t last_ota_time = 0;

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
ANIM_X_POS = 1,
ANIM_X_NEG = 2,
ANIM_Y_POS = 3,
ANIM_Y_NEG = 4,
ANIM_GOAL = 5
};

enum : uint8_t {
  LOCAL_ANIM_STILL   = 20,
  LOCAL_ANIM_ROLLING = 21,
  LOCAL_ANIM_BUMP    = 22,
  LOCAL_ANIM_X_POS   = 23,
  LOCAL_ANIM_X_NEG   = 24,
  LOCAL_ANIM_Y_POS   = 25,
  LOCAL_ANIM_Y_NEG   = 26
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
  uint8_t decideDesiredFmsByStartupFace() const { return 2;} //return (curFace_==0) ? 1 : 2; TEMP FIX

  int8_t gyroX() const { return (int8_t)constrain((int)lroundf(filtGX_), -127, 127); }
  int8_t gyroY() const { return (int8_t)constrain((int)lroundf(filtGY_), -127, 127); }
  int8_t gyroZ() const { return (int8_t)constrain((int)lroundf(filtGZ_), -127, 127); }
  bool moving() const { return moving_; }
  bool bumpActive() const { return bumpActive_; }

//   uint8_t localMoveAnim() const {
//   if(!moving_) return LOCAL_ANIM_STILL;

//   const int gx = gyroX();
//   const int gy = gyroY();

//   const int ax = abs(gx);
//   const int ay = abs(gy);

//   const int axisMargin = 2;

//   if(ax >= ay + axisMargin){
//     return (gx >= 0) ? LOCAL_ANIM_X_POS : LOCAL_ANIM_X_NEG;
//   }

//   if(ay >= ax + axisMargin){
//     return (gy >= 0) ? LOCAL_ANIM_Y_POS : LOCAL_ANIM_Y_NEG;
//   }

//   return LOCAL_ANIM_ROLLING;
// }

uint8_t localMoveAnim() const {
  if(!moving_) return LOCAL_ANIM_STILL;

  const int gx = gyroX();
  const int gy = gyroY();
  const int gz = gyroZ();

  // If gyro is saturated, don't trust axis direction.
  // Use generic rolling instead of X/Y strip animation.
  if(abs(gx) >= 126 || abs(gy) >= 126 || abs(gz) >= 126){
    return LOCAL_ANIM_ROLLING;
  }

  const int ax = abs(gx);
  const int ay = abs(gy);

  const int axisMargin = 8;

  if(ax >= ay + axisMargin){
    return (gx >= 0) ? LOCAL_ANIM_X_POS : LOCAL_ANIM_X_NEG;
  }

  if(ay >= ax + axisMargin){
    return (gy >= 0) ? LOCAL_ANIM_Y_POS : LOCAL_ANIM_Y_NEG;
  }

  return LOCAL_ANIM_ROLLING;
}

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

  bool gyroCalValid_ = false;
uint32_t bootMs_ = 0;

  bool bumpActive_ = false;
uint32_t bumpStartMs_ = 0;
uint32_t lastBumpMs_ = 0;

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

  delay(200);

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

  filtGX_ = 0.0f;
  filtGY_ = 0.0f;
  filtGZ_ = 0.0f;
  moving_ = false;
  gyroCalValid_ = true;
}

  // void calibrateGyroBias_(){
  //   if(!imu_ready_) return;

  //   const int N = 200;
  //   long sx = 0, sy = 0, sz = 0;

  //   delay(200);
  //   for(int i = 0; i < N; ++i){
  //     int16_t gx, gy, gz;
  //     mpu_.getRotation(&gx, &gy, &gz);
  //     sx += gx;
  //     sy += gy;
  //     sz += gz;
  //     delay(3);
  //   }

//     bootMs_ = millis();
//     gyroBiasX_ = (float)sx / (float)N / 131.0f;
//     gyroBiasY_ = (float)sy / (float)N / 131.0f;
//     gyroBiasZ_ = (float)sz / (float)N / 131.0f;

//     filtGX_ = 0.0f;
// filtGY_ = 0.0f;
// filtGZ_ = 0.0f;
// moving_ = false;
//   }

  void updateGyroMotion_(){
    if(!imu_ready_) return;
    const uint32_t now = millis();

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

    // const float moveThresh = 10.0f; // deg/s ; tune if needed
    // moving_ =
    //   (fabsf(filtGX_) > moveThresh) ||
    //   (fabsf(filtGY_) > moveThresh) ||
    //   (fabsf(filtGZ_) > moveThresh);
    // More stable movement detection
const float START_THRESH = 18.0f;  // must exceed this to start moving
const float STOP_THRESH  = 7.0f;   // must fall below this to stop moving

const float gyroMag = sqrtf(
  filtGX_ * filtGX_ +
  filtGY_ * filtGY_ +
  filtGZ_ * filtGZ_
);

static uint8_t moveVotes = 0;
static uint8_t stillVotes = 0;

if(!moving_){
  if(gyroMag > START_THRESH){
    if(moveVotes < 5) moveVotes++;
  } else {
    moveVotes = 0;
  }

  if(moveVotes >= 2){
    moving_ = true;
    stillVotes = 0;
  }
} else {
  if(gyroMag < STOP_THRESH){
    if(stillVotes < 10) stillVotes++;
  } else {
    stillVotes = 0;
  }

  if(stillVotes >= 6){
    moving_ = false;
    moveVotes = 0;
  }
}

//BUMP/HIT DETECTOR
      int16_t axRaw, ayRaw, azRaw;
mpu_.getAcceleration(&axRaw, &ayRaw, &azRaw);

// MPU6050 at ±2g range is about 16384 LSB/g.
const float axg = (float)axRaw / 16384.0f;
const float ayg = (float)ayRaw / 16384.0f;
const float azg = (float)azRaw / 16384.0f;

const float amag = sqrtf(axg * axg + ayg * ayg + azg * azg);

// Auto-rezero gyro if the ball is physically sitting still,
// even if the gyro currently thinks it is moving/saturated.
static uint32_t rezeroStartMs = 0;
static float lastAxg = 0.0f;
static float lastAyg = 0.0f;
static float lastAzg = 0.0f;
static bool haveLastAccel = false;

float accelDelta = 999.0f;

if(haveLastAccel){
  const float dx = axg - lastAxg;
  const float dy = ayg - lastAyg;
  const float dz = azg - lastAzg;
  accelDelta = sqrtf(dx * dx + dy * dy + dz * dz);
}

lastAxg = axg;
lastAyg = ayg;
lastAzg = azg;
haveLastAccel = true;

// Sitting still means acceleration vector is stable and close to 1g.
// This works even if gyro is falsely stuck at 127.
const bool accelLooksStill =
  haveLastAccel &&
  fabsf(amag - 1.0f) < 0.18f &&
  accelDelta < 0.035f;

if(accelLooksStill){
  if(rezeroStartMs == 0) rezeroStartMs = now;

  if(now - rezeroStartMs > 2000){
    gyroBiasX_ += filtGX_;
    gyroBiasY_ += filtGY_;
    gyroBiasZ_ += filtGZ_;

    filtGX_ = 0.0f;
    filtGY_ = 0.0f;
    filtGZ_ = 0.0f;

    moving_ = false;
    moveVotes = 0;
    stillVotes = 0;

    rezeroStartMs = now;
  }
} else {
  rezeroStartMs = 0;
}


// Tune these later.
const float BUMP_G_THRESH = 1.70f;
const uint32_t BUMP_FLASH_MS = 500;
const uint32_t BUMP_COOLDOWN_MS = 800;

if(!bumpActive_ &&
   amag > BUMP_G_THRESH &&
   now - lastBumpMs_ > BUMP_COOLDOWN_MS){
  bumpActive_ = true;
  bumpStartMs_ = now;
  lastBumpMs_ = now;
}

if(bumpActive_ && now - bumpStartMs_ > BUMP_FLASH_MS){
  bumpActive_ = false;
}
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
// class LedManager {
// public:
//   void begin(){
//     FastLED.addLeds<NEOPIXEL, LED_PIN>(leds_, NUM_LEDS);
//     FastLED.setCorrection(UncorrectedColor);
//     FastLED.setDither(0);
//     FastLED.setBrightness(brightness_);

//     // Boot rainbow animation
//     Serial.println("LED BEGIN / RAINBOW");
//     for(uint8_t step = 0; step < 32; step++){
//       for(uint8_t i = 0; i < NUM_LEDS; i++){
//         leds_[i] = CHSV((uint8_t)(step * 8 + i * 10), 255, 180);
//       }
//       FastLED.show();
//       delay(25);
//     }

//     clearAll_();
//     FastLED.show();

//     // standby sparkle target
//     rp_target_ = (uint8_t)(esp_random() % NUM_LEDS);
//     rp_nextMs_ = millis() + 500;

//     lastRenderMs_ = 0;
//     lastShowMs_ = 0;
//     dirty_ = true;

//     endgameValid_ = false;
//     endgameStartMs_ = 0;
//     endgameEntryColor_ = C_WHITE;
//     endgameEntryUnstable_ = false;

//     // hold state for X/Y
//     xOffset_ = LEDS_PER_BLOCK / 2;
//     yOffset_ = LEDS_PER_BLOCK / 2;
//     lastXUpdate_ = 0;
//     lastYUpdate_ = 0;
//   }

//   void tick(uint32_t ms,
//             bool identifyActive,
//             bool disconnected,
//             uint8_t sysState,
//             uint8_t teamColor,
//             bool stable20,
//             uint32_t timeGateStartMs,
//             uint8_t animMode){

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
//       (sysState == SYS_IN_GAME && animMode != ANIM_STILL) ||
//       disconnected;

//     if(!animating &&
//        !dirty_ &&
//        sysState == lastSysStateLatch_ &&
//        teamColor == lastTeamColor_ &&
//        identifyActive == lastIdentify_ &&
//        disconnected == lastDisconnected_ &&
//        stable20 == lastStable_ &&
//        animMode == lastAnimMode_){
//       return;
//     }

//     lastSysStateLatch_ = sysState;
//     lastTeamColor_ = teamColor;
//     lastIdentify_ = identifyActive;
//     lastDisconnected_ = disconnected;
//     lastStable_ = stable20;
//     lastAnimMode_ = animMode;

//     clearAll_();

//     if(identifyActive){
//       const bool on = flash5Hz_(ms);
//       renderSolidAll_(on ? C_GREEN : C_PURPLE);

//     } else if(sysState == SYS_STANDBY){
//       renderStandbyTwoPixel_(ms);

//     } else if(sysState == SYS_TIME_GATE){
//       if(timeGateStartMs && (ms - timeGateStartMs <= 500)){
//         for(uint8_t i = 0; i < NUM_LEDS; i++){
//           leds_[i] = CHSV((uint8_t)((ms / 3) + i * 10), 255, 180);
//         }
//       } else {
//         // hold last in-game position when time gate settles
//         renderStillPatternHold_();
//       }

//     } else if(sysState == SYS_RESET){
//       renderSolidAll_(C_PURPLE);

//     } else if(sysState == SYS_GAME_START){
//       renderSolidAll_(C_GREEN);

//     } else if(sysState == SYS_IN_GAME){
//       if(animMode == ANIM_X_POS){
//         renderXMotion_Positive_(ms);
//       } else if(animMode == ANIM_X_NEG){
//         renderXMotion_Negative_(ms);
//       } else if(animMode == ANIM_Y_POS){
//         renderYMotion_Positive_(ms);
//       } else if(animMode == ANIM_Y_NEG){
//         renderYMotion_Negative_(ms);
//       } else {
//         // THIS is the preserved "still holds last lit position" behavior
//         renderStillPatternHold_();
//       }

//     } else if(sysState == SYS_END_GAME){
//       if(!endgameValid_) snapshotEndgameEntry_(ms, teamColor, stable20);

//       const uint32_t dt = (endgameStartMs_ == 0) ? 0 : (ms - endgameStartMs_);

//       if(dt < 2000){
//         renderSolidAll_(C_RED);
//       } else {
//         clearAll_();

//         // Since the ball is effectively white-only, use the held X/Y pattern as underlay.
//         renderStillPatternHold_();

//         const uint32_t t = dt - 2000;
//         const float pos = fmodf((float)t * 0.015f, (float)NUM_LEDS);
//         int i0 = (int)floorf(pos);
//         float frac = pos - (float)i0;
//         if(i0 < 0) i0 = 0;
//         const int i1 = (i0 + 1) % NUM_LEDS;

//         const uint8_t v0 = (uint8_t)(170.0f * (1.0f - frac));
//         const uint8_t v1 = (uint8_t)(170.0f * frac);

//         overlayPureRed_(i0, v0);
//         overlayPureRed_(i1, v1);
//       }

//     } else {
//       renderStandbyTwoPixel_(ms);
//     }

//     if(disconnected && !identifyActive){
//       const bool on = ((ms / 250) & 1) == 0;
//       applyDisconnectedOverlay_(on ? CRGB(100,0,0) : CRGB::Black);
//     }

//     dirty_ = true;

//     if((int32_t)(ms - lastShowMs_) >= (int32_t)SHOW_PERIOD_MS_){
//       FastLED.show();
//       lastShowMs_ = ms;
//       dirty_ = false;
//     }
//   }

// private:
//   static constexpr uint8_t  BLOCK_X_A = 0; // block 1
//   static constexpr uint8_t  BLOCK_X_B = 1; // block 2
//   static constexpr uint8_t  BLOCK_Y_A = 2; // block 3
//   static constexpr uint8_t  BLOCK_Y_B = 3; // block 4

//   CRGB leds_[NUM_LEDS];

//   static constexpr uint8_t  TARGET_FPS_ = 120;
//   static constexpr uint32_t RENDER_PERIOD_MS_ = (1000 / TARGET_FPS_);
//   static constexpr uint32_t SHOW_PERIOD_MS_   = (1000 / TARGET_FPS_);
//   static constexpr uint16_t interval_         = 100;

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
//   uint8_t lastAnimMode_ = 0xFF;

//   uint8_t rp_target_ = 0;
//   uint32_t rp_nextMs_ = 0;

//   bool endgameValid_ = false;
//   uint32_t endgameStartMs_ = 0;
//   uint8_t endgameEntryColor_ = C_WHITE;
//   bool endgameEntryUnstable_ = false;

//   // independent held positions for X and Y
//   uint8_t xOffset_ = 0;
//   uint8_t yOffset_ = 0;
//   uint32_t lastXUpdate_ = 0;
//   uint32_t lastYUpdate_ = 0;

//   static inline uint16_t ledIndex_(uint8_t block, uint8_t pos){
//     return (uint16_t)block * LEDS_PER_BLOCK + pos;
//   }

//   static inline bool flash5Hz_(uint32_t ms){
//     return ((ms / 100) & 1) == 0;
//   }

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

//   void clearAll_(){
//     memset(leds_, 0, sizeof(leds_));
//   }

//   void renderSolidAll_(uint8_t color){
//     const CRGB c = colorToCRGB_(color);
//     for(uint8_t i = 0; i < NUM_LEDS; i++){
//       leds_[i] = c;
//     }
//   }

//   void applyDisconnectedOverlay_(const CRGB& c){
//     // Put one marker LED near the center of each block
//     for(uint8_t b = 0; b < NUM_BLOCKS; b++){
//       leds_[ledIndex_(b, LEDS_PER_BLOCK / 2)] = c;
//     }
//   }

//   void snapshotEndgameEntry_(uint32_t ms, uint8_t teamColor, bool stable20){
//     endgameStartMs_ = ms;
//     endgameEntryColor_ = teamColor;
//     endgameEntryUnstable_ = !stable20;
//     endgameValid_ = true;
//   }

//   void overlayPureRed_(int idx, uint8_t redVal){
//     if(redVal == 0) return;
//     if(idx < 0) return;
//     if(idx >= (int)NUM_LEDS) return;

//     leds_[idx].r = redVal;
//     leds_[idx].g = 0;
//     leds_[idx].b = 0;
//   }

//   void setMirroredPairPixel_(uint8_t blockA, uint8_t blockB, uint8_t pos, const CRGB& color){
//     if(pos >= LEDS_PER_BLOCK) return;

//     leds_[ledIndex_(blockA, pos)] = color;
//     leds_[ledIndex_(blockB, (LEDS_PER_BLOCK - 1) - pos)] = color;
//   }

//   void renderPairAtPosWide_(uint8_t blockA, uint8_t blockB, uint8_t pos, const CRGB& color){
//     setMirroredPairPixel_(blockA, blockB, pos, color);

//     if(pos + 1 < LEDS_PER_BLOCK){
//       setMirroredPairPixel_(blockA, blockB, pos + 1, color);
//     }
//   }

//   void renderXHold_(){
//     const CRGB w = colorToCRGB_(C_WHITE);
//     renderPairAtPosWide_(BLOCK_X_A, BLOCK_X_B, xOffset_, w);
//   }

//   void renderYHold_(){
//     const CRGB w = colorToCRGB_(C_WHITE);
//     renderPairAtPosWide_(BLOCK_Y_A, BLOCK_Y_B, yOffset_, w);
//   }

//   void renderStillPatternHold_(){
//     renderXHold_();
//     renderYHold_();
//   }

//   void renderXMotion_Positive_(uint32_t ms){
//     const CRGB w = colorToCRGB_(C_WHITE);

//     if((uint32_t)(ms - lastXUpdate_) >= interval_){
//       xOffset_ = (xOffset_ + 1) % LEDS_PER_BLOCK;
//       lastXUpdate_ = ms;
//     }

//     renderPairAtPosWide_(BLOCK_X_A, BLOCK_X_B, xOffset_, w);
//     renderYHold_();
//   }

//   void renderXMotion_Negative_(uint32_t ms){
//     const CRGB w = colorToCRGB_(C_WHITE);

//     if((uint32_t)(ms - lastXUpdate_) >= interval_){
//       xOffset_ = (xOffset_ + LEDS_PER_BLOCK - 1) % LEDS_PER_BLOCK;
//       lastXUpdate_ = ms;
//     }

//     renderPairAtPosWide_(BLOCK_X_A, BLOCK_X_B, xOffset_, w);
//     renderYHold_();
//   }

//   void renderYMotion_Positive_(uint32_t ms){
//     const CRGB w = colorToCRGB_(C_WHITE);

//     if((uint32_t)(ms - lastYUpdate_) >= interval_){
//       yOffset_ = (yOffset_ + 1) % LEDS_PER_BLOCK;
//       lastYUpdate_ = ms;
//     }

//     renderXHold_();
//     renderPairAtPosWide_(BLOCK_Y_A, BLOCK_Y_B, yOffset_, w);
//   }

//   void renderYMotion_Negative_(uint32_t ms){
//     const CRGB w = colorToCRGB_(C_WHITE);

//     if((uint32_t)(ms - lastYUpdate_) >= interval_){
//       yOffset_ = (yOffset_ + LEDS_PER_BLOCK - 1) % LEDS_PER_BLOCK;
//       lastYUpdate_ = ms;
//     }

//     renderXHold_();
//     renderPairAtPosWide_(BLOCK_Y_A, BLOCK_Y_B, yOffset_, w);
//   }

//   void renderStandbyTwoPixel_(uint32_t ms){
//     if((int32_t)(ms - rp_nextMs_) >= 0){
//       rp_target_ = (uint8_t)(esp_random() % NUM_LEDS);
//       rp_nextMs_ = ms + 1000;
//     }

//     leds_[rp_target_] = CHSV((uint8_t)(((ms / 10) & 0xFF) + rp_target_ * 7), 255, 190);

//     // Mirror to the adjacent partner block when possible
//     uint8_t block = rp_target_ / LEDS_PER_BLOCK;
//     uint8_t pos   = rp_target_ % LEDS_PER_BLOCK;

//     if(block == BLOCK_X_A){
//       leds_[ledIndex_(BLOCK_X_B, (LEDS_PER_BLOCK - 1) - pos)] =
//         CHSV((uint8_t)(((ms / 10) & 0xFF) + (rp_target_ + 5) * 7), 255, 190);
//     } else if(block == BLOCK_X_B){
//       leds_[ledIndex_(BLOCK_X_A, (LEDS_PER_BLOCK - 1) - pos)] =
//         CHSV((uint8_t)(((ms / 10) & 0xFF) + (rp_target_ + 5) * 7), 255, 190);
//     } else if(block == BLOCK_Y_A){
//       leds_[ledIndex_(BLOCK_Y_B, (LEDS_PER_BLOCK - 1) - pos)] =
//         CHSV((uint8_t)(((ms / 10) & 0xFF) + (rp_target_ + 5) * 7), 255, 190);
//     } else {
//       leds_[ledIndex_(BLOCK_Y_A, (LEDS_PER_BLOCK - 1) - pos)] =
//         CHSV((uint8_t)(((ms / 10) & 0xFF) + (rp_target_ + 5) * 7), 255, 190);
//     }
//   }
// };
class LedManager {
public:
  void begin(){
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds_, NUM_LEDS);
    FastLED.setCorrection(UncorrectedColor);
    FastLED.setDither(0);
    FastLED.setBrightness(brightness_);

    Serial.println("LED BEGIN / RAINBOW");
    for(uint8_t step = 0; step < 32; step++){
      for(uint8_t i = 0; i < NUM_LEDS; i++){
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

    xOffset_ = LEDS_PER_BLOCK / 2;
    yOffset_ = LEDS_PER_BLOCK / 2;
    lastXUpdate_ = 0;
    lastYUpdate_ = 0;

    lastRollUpdate_ = 0;
    rollOffset_ = 0;

    lastMotionWasX_ = true;
    heldPrimaryPos_ = 0;
    heldSecondaryPos_ = 2;
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
  (sysState == SYS_IN_GAME) ||   
  disconnected;

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
      renderSolidAll_(on ? C_GREEN : C_PURPLE);

    } else if(sysState == SYS_STANDBY){
      renderStillPatternHold_();

    } else if(sysState == SYS_TIME_GATE){
      if(timeGateStartMs && (ms - timeGateStartMs <= 500)){
        for(uint8_t i = 0; i < NUM_LEDS; i++){
          leds_[i] = CHSV((uint8_t)((ms / 3) + i * 10), 255, 180);
        }
      } else {
        renderStillPatternHold_();
      }

    } else if(sysState == SYS_RESET){
      renderSolidAll_(C_PURPLE);

    } else if(sysState == SYS_GAME_START){
      renderSolidAll_(C_GREEN);

    } else if(sysState == SYS_IN_GAME){

  if(animMode == ANIM_GOAL){
    renderDiscoBall_(ms);
  }
  else if(animMode == LOCAL_ANIM_BUMP){
    renderBumpOrange_(ms);
  }
  else if(animMode == LOCAL_ANIM_X_POS){
    renderXMotion_Negative_(ms);
  }
  else if(animMode == LOCAL_ANIM_X_NEG){
    renderXMotion_Positive_(ms);
  }
  else if(animMode == LOCAL_ANIM_Y_POS){
    renderYMotion_Negative_(ms);
  }
  else if(animMode == LOCAL_ANIM_Y_NEG){
    renderYMotion_Positive_(ms);
  }
  else if(animMode == LOCAL_ANIM_ROLLING){
    renderRollingLocal_(ms);
  }
  else {
    renderStillPatternHold_();
  }
} else if(sysState == SYS_END_GAME){
  if(!endgameValid_) snapshotEndgameEntry_(ms, teamColor, stable20);

  const uint32_t dt = (endgameStartMs_ == 0) ? 0 : (ms - endgameStartMs_);

  if(dt < 2000){
    renderSolidAll_(C_RED);
  } 
  else if(dt < 6000){
    renderBreathingRed_(ms);
  } 
  else {
    renderStillPatternHold_();
  }

} else {
  renderStandbyTwoPixel_(ms);
}
    // } else if(sysState == SYS_END_GAME){
    //   if(!endgameValid_) snapshotEndgameEntry_(ms, teamColor, stable20);

    //   const uint32_t dt = (endgameStartMs_ == 0) ? 0 : (ms - endgameStartMs_);

    //   if(dt < 2000){ //2 Seconds SOLID RED
    //     renderSolidAll_(C_RED);
    //   } else if(dt < 6000) {
    //     renderBreathingRed_(ms); //4 seconds BREATHING RED
    //   } else { renderStillPatternHold_(); } //Return to standby
      

    // //     const uint32_t t = dt - 2000;
    // //     const float pos = fmodf((float)t * 0.015f, (float)NUM_LEDS);
    // //     int i0 = (int)floorf(pos);
    // //     float frac = pos - (float)i0;
    // //     if(i0 < 0) i0 = 0;
    // //     const int i1 = (i0 + 1) % NUM_LEDS;

    // //     const uint8_t v0 = (uint8_t)(170.0f * (1.0f - frac));
    // //     const uint8_t v1 = (uint8_t)(170.0f * frac);

    // //     overlayPureRed_(i0, v0);
    // //     overlayPureRed_(i1, v1);
    // //   }

    // // } else {
    // //   renderStandbyTwoPixel_(ms);
    // // }

    if(disconnected && !identifyActive){
      const bool on = ((ms / 250) & 1) == 0;
      applyDisconnectedOverlay_(on ? CRGB(100,0,0) : CRGB::Black);
    }

    dirty_ = true;

    if((int32_t)(ms - lastShowMs_) >= (int32_t)SHOW_PERIOD_MS_){
      FastLED.show();
      lastShowMs_ = ms;
      dirty_ = false;
    }
  }

private:
  static constexpr uint8_t  BLOCK_X_A = 0;
  static constexpr uint8_t  BLOCK_X_B = 1;
  static constexpr uint8_t  BLOCK_Y_A = 2;
  static constexpr uint8_t  BLOCK_Y_B = 3;

  CRGB leds_[NUM_LEDS];

  static constexpr uint8_t  TARGET_FPS_ = 120;
  static constexpr uint32_t RENDER_PERIOD_MS_ = (1000 / TARGET_FPS_);
  static constexpr uint32_t SHOW_PERIOD_MS_   = (1000 / TARGET_FPS_);
  static constexpr uint16_t interval_         = 100;
  static constexpr uint16_t rollInterval_     = 95;

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

  uint8_t xOffset_ = 0;
  uint8_t yOffset_ = 0;
  uint32_t lastXUpdate_ = 0;
  uint32_t lastYUpdate_ = 0;

  uint8_t rollOffset_ = 0;
  uint32_t lastRollUpdate_ = 0;

  bool lastMotionWasX_ = true;
  uint8_t heldPrimaryPos_ = 0;
  uint8_t heldSecondaryPos_ = 2;

  static inline uint16_t ledIndex_(uint8_t block, uint8_t pos){
    return (uint16_t)block * LEDS_PER_BLOCK + pos;
  }

  static inline bool flash5Hz_(uint32_t ms){
    return ((ms / 100) & 1) == 0;
  }

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

 uint8_t breathLevel_() const{
    //return 128 + scale8(cubicwave8((uint8_t)(millis() >> 4)), 125);
  return lerp8by8(20, 255, cubicwave8((uint8_t)(millis() >> 4)));
  }

  CRGB scaleColor_(const CRGB& c, uint8_t scale) const{
    CRGB out;
    out.r = scale8(c.r, scale);
    out.g = scale8(c.g, scale);
    out.b = scale8(c.b, scale);
    return out;
  }

  void clearAll_(){
    memset(leds_, 0, sizeof(leds_));
  }
//Hit animation
  void renderBumpOrange_(uint32_t ms){
  const bool on = ((ms / 80) & 1) == 0;
  fill_solid(leds_, NUM_LEDS, on ? CRGB(180, 55, 0) : CRGB::Black);
}

//End game animation 2
void renderBreathingRed_(uint32_t ms){
  const uint8_t r = beatsin8(14, 20, 150);
  fill_solid(leds_, NUM_LEDS, CRGB(r, 0, 0));
}

// void renderRollingLocal_(uint32_t ms){
//   if(ms - lastRollUpdate_ >= interval_){
//     lastRollUpdate_ = ms;
//     rollOffset_++;
//     if(rollOffset_ >= LEDS_PER_BLOCK) rollOffset_ = 0;
//   }

//   const CRGB white = CRGB(90, 90, 90);

//   const CRGB rainbow = CHSV((uint8_t)(ms / 6), 255, 160);

//   renderPairAtPosWide_(BLOCK_X_A, BLOCK_X_B, rollOffset_, white);

//   uint8_t yPos = (rollOffset_ + LEDS_PER_BLOCK / 2) % LEDS_PER_BLOCK;
//   renderPairAtPosWide_(BLOCK_Y_A, BLOCK_Y_B, yPos, rainbow);

//   xOffset_ = rollOffset_;
//   yOffset_ = yPos;
// }

void renderRollingLocal_(uint32_t ms){
  renderBaseWhiteAllMotion_();

  if(ms - lastRollUpdate_ >= rollInterval_){
    lastRollUpdate_ = ms;
    rollOffset_++;
    if(rollOffset_ >= LEDS_PER_BLOCK) rollOffset_ = 0;
  }

  const uint8_t secondary = (rollOffset_ + 2) % LEDS_PER_BLOCK;

  heldPrimaryPos_ = rollOffset_;
  heldSecondaryPos_ = secondary;

  renderSegmentedWholeBall_(rollOffset_, secondary);
}
//Goal animation
  void renderDiscoBall_(uint32_t ms){
  for(uint8_t i = 0; i < NUM_LEDS; i++){
    uint8_t hue = (uint8_t)((ms / 8) + i * 31);
    uint8_t sparkle = (uint8_t)((sin8((uint8_t)(ms / 3 + i * 47)) >> 1) + 127);

    leds_[i] = CHSV(hue, 255, sparkle);
  }

  uint8_t p0 = (ms / 70) % NUM_LEDS;
  uint8_t p1 = (p0 + 11) % NUM_LEDS;
  uint8_t p2 = (p0 + 23) % NUM_LEDS;

  leds_[p0] = CHSV((uint8_t)(ms / 4), 255, 255);
  leds_[p1] = CHSV((uint8_t)(ms / 4 + 85), 255, 255);
  leds_[p2] = CHSV((uint8_t)(ms / 4 + 170), 255, 255);
}

  void renderSolidAll_(uint8_t color){
    const CRGB c = colorToCRGB_(color);
    for(uint8_t i = 0; i < NUM_LEDS; i++){
      leds_[i] = c;
    }
  }

  void renderBaseWhiteAllFull_(){
    const CRGB w = colorToCRGB_(C_WHITE);
    for(uint8_t i = 0; i < NUM_LEDS; i++){
      leds_[i] = w;
    }
  }

  void renderBaseWhiteAllMotion_(){
    const CRGB w = scaleColor_(colorToCRGB_(C_WHITE), 165);
    for(uint8_t i = 0; i < NUM_LEDS; i++){
      leds_[i] = w;
    }
  }

  void renderBaseWhiteAllBreathing_(){
    const uint8_t breath = breathLevel_();
    const CRGB w = scaleColor_(colorToCRGB_(C_WHITE), breath);
    for(uint8_t i = 0; i < NUM_LEDS; i++){
      leds_[i] = w;
    }
  }

  void applyDisconnectedOverlay_(const CRGB& c){
    for(uint8_t b = 0; b < NUM_BLOCKS; b++){
      leds_[ledIndex_(b, LEDS_PER_BLOCK / 2)] = c;
    }
  }

  void snapshotEndgameEntry_(uint32_t ms, uint8_t teamColor, bool stable20){
    endgameStartMs_ = ms;
    endgameEntryColor_ = teamColor;
    endgameEntryUnstable_ = !stable20;
    endgameValid_ = true;
  }

  void overlayPureRed_(int idx, uint8_t redVal){
    if(redVal == 0) return;
    if(idx < 0) return;
    if(idx >= (int)NUM_LEDS) return;

    leds_[idx].r = redVal;
    leds_[idx].g = 0;
    leds_[idx].b = 0;
  }

  void setMirroredPairPixel_(uint8_t blockA, uint8_t blockB, uint8_t pos, const CRGB& color){
    if(pos >= LEDS_PER_BLOCK) return;
    leds_[ledIndex_(blockA, pos)] = color;
    leds_[ledIndex_(blockB, (LEDS_PER_BLOCK - 1) - pos)] = color;
  }

  void renderPairAtPosWide_(uint8_t blockA, uint8_t blockB, uint8_t pos, const CRGB& color){
    setMirroredPairPixel_(blockA, blockB, pos, color);
    if(pos + 1 < LEDS_PER_BLOCK){
      setMirroredPairPixel_(blockA, blockB, pos + 1, color);
    }
  }

  void renderXHold_(){
    const CRGB w = colorToCRGB_(C_WHITE);
    renderPairAtPosWide_(BLOCK_X_A, BLOCK_X_B, xOffset_, w);
  }

  void renderYHold_(){
    const CRGB w = colorToCRGB_(C_WHITE);
    renderPairAtPosWide_(BLOCK_Y_A, BLOCK_Y_B, yOffset_, w);
  }

  void paintRingSegment_(uint8_t blockA, uint8_t blockB, uint8_t startPos, const CRGB& color, uint8_t length){
    for(uint8_t i = 0; i < length; i++){
      uint8_t pos = (uint8_t)((startPos + i) % LEDS_PER_BLOCK);
      setMirroredPairPixel_(blockA, blockB, pos, color);
    }
  }

  void renderSegmentedRingOnAxis_(uint8_t blockA, uint8_t blockB, uint8_t offset, uint8_t scale){
    const CRGB red   = scaleColor_(CRGB(255,   0,   0), scale);
    const CRGB blue  = scaleColor_(CRGB(  0,   0, 255), scale);
    const CRGB green = scaleColor_(CRGB(  0, 255,   0), scale);

    // 3 colored segments, 2 LEDs each, spaced around the 11-LED lane
    paintRingSegment_(blockA, blockB, (uint8_t)((offset + 0) % LEDS_PER_BLOCK), red,   2);
    paintRingSegment_(blockA, blockB, (uint8_t)((offset + 4) % LEDS_PER_BLOCK), blue,  2);
    paintRingSegment_(blockA, blockB, (uint8_t)((offset + 8) % LEDS_PER_BLOCK), green, 2);
  }

  void renderSegmentedWholeBall_(uint8_t primaryOffset, uint8_t secondaryOffset){
    renderSegmentedRingOnAxis_(BLOCK_X_A, BLOCK_X_B, primaryOffset, 255);
    renderSegmentedRingOnAxis_(BLOCK_Y_A, BLOCK_Y_B, secondaryOffset, 180);
  }

  void renderSegmentedWholeBallScaled_(uint8_t primaryOffset, uint8_t secondaryOffset, uint8_t scale){
    renderSegmentedRingOnAxis_(BLOCK_X_A, BLOCK_X_B, primaryOffset, scale);
    renderSegmentedRingOnAxis_(BLOCK_Y_A, BLOCK_Y_B, secondaryOffset, scale8(scale, 180));
  }

  void renderHeldBreathingPattern_(){
    const uint8_t breath = breathLevel_();
    renderBaseWhiteAllBreathing_();
    renderSegmentedWholeBallScaled_(heldPrimaryPos_, heldSecondaryPos_, breath);
  }

  void renderStillPatternHold_(){
    renderHeldBreathingPattern_();
  }

  void renderSegmentedRoll_X_(uint32_t ms, bool positive){
    renderBaseWhiteAllMotion_();

    if((uint32_t)(ms - lastRollUpdate_) >= rollInterval_){
      rollOffset_ = positive
        ? (uint8_t)((rollOffset_ + 1) % LEDS_PER_BLOCK)
        : (uint8_t)((rollOffset_ + LEDS_PER_BLOCK - 1) % LEDS_PER_BLOCK);

      xOffset_ = rollOffset_;
      lastRollUpdate_ = ms;
    }

    const uint8_t secondary = positive
      ? (uint8_t)((rollOffset_ + 2) % LEDS_PER_BLOCK)
      : (uint8_t)((rollOffset_ + LEDS_PER_BLOCK - 2) % LEDS_PER_BLOCK);

    heldPrimaryPos_ = rollOffset_;
    heldSecondaryPos_ = secondary;

    renderSegmentedWholeBall_(rollOffset_, secondary);
  }

  void renderSegmentedRoll_Y_(uint32_t ms, bool positive){
    renderBaseWhiteAllMotion_();

    if((uint32_t)(ms - lastRollUpdate_) >= rollInterval_){
      rollOffset_ = positive
        ? (uint8_t)((rollOffset_ + 1) % LEDS_PER_BLOCK)
        : (uint8_t)((rollOffset_ + LEDS_PER_BLOCK - 1) % LEDS_PER_BLOCK);

      yOffset_ = rollOffset_;
      lastRollUpdate_ = ms;
    }

    const uint8_t secondary = positive
      ? (uint8_t)((rollOffset_ + 2) % LEDS_PER_BLOCK)
      : (uint8_t)((rollOffset_ + LEDS_PER_BLOCK - 2) % LEDS_PER_BLOCK);

    heldPrimaryPos_ = secondary;
    heldSecondaryPos_ = rollOffset_;

    renderSegmentedWholeBall_(secondary, rollOffset_);
  }

  void renderXMotion_Positive_(uint32_t ms){
    lastMotionWasX_ = true;
    renderSegmentedRoll_X_(ms, true);
  }

  void renderXMotion_Negative_(uint32_t ms){
    lastMotionWasX_ = true;
    renderSegmentedRoll_X_(ms, false);
  }

  void renderYMotion_Positive_(uint32_t ms){
    lastMotionWasX_ = false;
    renderSegmentedRoll_Y_(ms, true);
  }

  void renderYMotion_Negative_(uint32_t ms){
    lastMotionWasX_ = false;
    renderSegmentedRoll_Y_(ms, false);
  }

  void renderStandbyTwoPixel_(uint32_t ms){
    if((int32_t)(ms - rp_nextMs_) >= 0){
      rp_target_ = (uint8_t)(esp_random() % NUM_LEDS);
      rp_nextMs_ = ms + 1000;
    }

    leds_[rp_target_] = CHSV((uint8_t)(((ms / 10) & 0xFF) + rp_target_ * 7), 255, 190);

    uint8_t block = rp_target_ / LEDS_PER_BLOCK;
    uint8_t pos   = rp_target_ % LEDS_PER_BLOCK;

    if(block == BLOCK_X_A){
      leds_[ledIndex_(BLOCK_X_B, (LEDS_PER_BLOCK - 1) - pos)] =
        CHSV((uint8_t)(((ms / 10) & 0xFF) + (rp_target_ + 5) * 7), 255, 190);
    } else if(block == BLOCK_X_B){
      leds_[ledIndex_(BLOCK_X_A, (LEDS_PER_BLOCK - 1) - pos)] =
        CHSV((uint8_t)(((ms / 10) & 0xFF) + (rp_target_ + 5) * 7), 255, 190);
    } else if(block == BLOCK_Y_A){
      leds_[ledIndex_(BLOCK_Y_B, (LEDS_PER_BLOCK - 1) - pos)] =
        CHSV((uint8_t)(((ms / 10) & 0xFF) + (rp_target_ + 5) * 7), 255, 190);
    } else {
      leds_[ledIndex_(BLOCK_Y_A, (LEDS_PER_BLOCK - 1) - pos)] =
        CHSV((uint8_t)(((ms / 10) & 0xFF) + (rp_target_ + 5) * 7), 255, 190);
    }
  }
};
/* ===================== Link Management ===================== */
class LinkManager {
public:
  void begin(RxQueue* q){
    q_ = q;

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    esp_wifi_set_ps(WIFI_PS_NONE);
if (WiFi.status() == WL_CONNECTED) {
  wifiChannel_ = WiFi.channel();
}

    if(esp_now_init() != ESP_OK){
      delay(100);
      ESP.restart();
    }

    esp_now_register_recv_cb(onRecv_);
    esp_now_register_send_cb(onSent_);

        ensureBroadcastPeer_();

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

  void clearBroadcastPeer_(){
    if(esp_now_is_peer_exist(BCAST)){
      esp_now_del_peer(BCAST);
    }
  }

  void ensureBroadcastPeer_(){
    clearBroadcastPeer_();

    esp_now_peer_info_t p{};
    memcpy(p.peer_addr, BCAST, 6);
    p.channel = 0;          // use current Wi-Fi/ESP-NOW channel
    p.encrypt = false;
    esp_now_add_peer(&p);
  }

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
    // Keep our idea of channel fresh
    if (WiFi.status() == WL_CONNECTED) {
      wifiChannel_ = WiFi.channel();
    }

    // Make sure broadcast peer is valid for the current channel
    ensureBroadcastPeer_();

    PairReq req{};
    req.proto_ver   = PROTO_VER;
    req.msg_type    = MSG_PAIR_REQ;
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


if (WiFi.status() == WL_CONNECTED) {
  wifiChannel_ = WiFi.channel();
} else {
  wifiChannel_ = ack.channel;
  esp_wifi_set_channel(wifiChannel_, WIFI_SECOND_CHAN_NONE);
}
upsertFmsPeer_(f.src, wifiChannel_);
        // wifiChannel_ = ack.channel;
        // esp_wifi_set_channel(wifiChannel_, WIFI_SECOND_CHAN_NONE);
        // upsertFmsPeer_(f.src, wifiChannel_);

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
static void startWiFiForOTA_() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(OTA_SSID, OTA_PASS);

    Serial.print("Connecting to WiFi");
    uint32_t start = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
      delay(200);
      Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected!");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("WiFi connect timeout (continuing anyway)");
    }
  }
}

static void startOTA_() {
  ArduinoOTA
    .onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.setHostname("cubicchaos-ball");
  //ArduinoOTA.setPassword("abcdefgh");  // optional

  ArduinoOTA.begin();
  otaReady_ = true;

  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("WiFi channel: ");
Serial.println(WiFi.channel());
}

class CubeApp {
public:
  void begin(){
    Serial.begin(115200);
    delay(20);
    startWiFiForOTA_();

    soundIdentify_.begin(PIEZO_PIN, /*ledcChannel*/ 0);
    leds_.begin();
    imu_.begin();
    imu_.primeFace();

    link_.begin(&rxq_);
    link_.setDesiredFms(imu_.decideDesiredFmsByStartupFace());

    if (WiFi.status() == WL_CONNECTED && !otaReady_) {
  startOTA_();
    }

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

    static uint32_t lastWifiRetryMs = 0;

// if (WiFi.status() != WL_CONNECTED && millis() - lastWifiRetryMs > 5000) {
//   lastWifiRetryMs = millis();
//   WiFi.begin(OTA_SSID, OTA_PASS);
// }


if (!link_.linkedNow(millis()) &&
    WiFi.status() != WL_CONNECTED &&
    millis() - lastWifiRetryMs > 5000) {
  lastWifiRetryMs = millis();
  WiFi.begin(OTA_SSID, OTA_PASS);
}

if (otaReady_ && WiFi.status() != WL_CONNECTED) {
  otaReady_ = false;
}

    if (!otaReady_ && WiFi.status() == WL_CONNECTED) {
  startOTA_();
  }

  if (otaReady_) {
  ArduinoOTA.handle();
  }

    const uint32_t ms = millis();

    link_.service(imu_, ms);

    if(link_.consumeIdentifyTrigger()){
      soundIdentify_.trigger(ms);
    }

    imu_.tick(ms);
    soundIdentify_.tick(ms);

    cmdSysState_ = link_.cmdSysState();
cmdColor_    = C_WHITE;

// FMS only overrides with goal animation now.
// Normal movement animation is chosen locally by the ball.
const uint8_t fmsAnim = link_.cmdAnim();

if(fmsAnim == ANIM_GOAL){
  cmdAnim_ = ANIM_GOAL;
}
else if(imu_.bumpActive()){
  cmdAnim_ = LOCAL_ANIM_BUMP;
}
else if(imu_.moving()){
  cmdAnim_ = imu_.localMoveAnim();
}
else{
  cmdAnim_ = LOCAL_ANIM_STILL;
}

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