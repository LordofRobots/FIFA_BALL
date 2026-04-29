/********************************************************
 * CubicChaos — FMS
 *
 ********************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ESPmDNS.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FastLED.h>
#include "esp32-hal-ledc.h"

// --- Forward decl to satisfy Arduino auto-prototypes ---
struct Peer;
struct ToneStep;

/* ===================== CONFIG ===================== */
// NOTE: FMS ID is selected at boot from GPIO_36 (see setup()).
static uint8_t g_fmsId = 1;
static inline uint8_t FMS_ID_() {
  return g_fmsId;
}

static const char* STA_SSID = "minibot_fms";
static const char* STA_PASS = "abcdefgh";

static bool ipIsValid_(const IPAddress& ip) {
  return !(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);
}

static char ipCache[16] = "0.0.0.0";

static bool connectWiFiRobust_(uint32_t attemptTimeoutMs = 8000, uint8_t maxAttempts = 6) {
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);

  for (uint8_t a = 1; a <= maxAttempts; a++) {
    Serial.print("WiFi connect attempt ");
    Serial.println(a);

    WiFi.disconnect(true, true);
    delay(120);

    WiFi.begin(STA_SSID, STA_PASS);

    const uint32_t t0 = millis();
    while (millis() - t0 < attemptTimeoutMs) {
      if (WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        if (ipIsValid_(ip)) {
          snprintf(ipCache, sizeof(ipCache), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
          Serial.print("WiFi OK, IP ");
          Serial.println(ipCache);
          return true;
        }
      }
      delay(50);
    }

    Serial.println("WiFi attempt failed (no valid IP). Retrying...");
    delay(250);
  }

  Serial.println("WiFi failed after retries.");
  return false;
}

/* ===================== PIN ===================== */
#define SOUND_PIN 32

/* ===================== DIAG LEDS (FastLED) ===================== */
#define DIAG_LED_PIN 33
#define DIAG_LED_COUNT 4
static CRGB diagLeds[DIAG_LED_COUNT];

// Ring order requested: 1 2 4 3
static const uint8_t RING_ORDER[DIAG_LED_COUNT] = { 0, 1, 3, 2 };
static const uint32_t WARN_STEP_MS = 500;
static const uint32_t DIAG_LED_TICK_MS = 20;

/* ===================== FMS ID SWITCH ===================== */
#define FMS_SEL_PIN 36  // boot select (and monitored; change triggers reboot)
static bool g_fmsSelBoot = false;

/* ===================== Protocol ===================== */
#define PROTO_VER 5

enum MsgType : uint8_t { MSG_PAIR_REQ = 1,
                         MSG_PAIR_ACK = 2,
                         MSG_POLL = 3,
                         MSG_STATUS = 4,
                         MSG_CMD = 5 };

// System states
enum : uint8_t { SYS_STANDBY = 0,
                 SYS_GAME_START = 1,
                 SYS_IN_GAME = 2,
                 SYS_TIME_GATE = 3,
                 SYS_END_GAME = 4,
                 SYS_RESET = 5 };

enum : uint8_t {
  ANIM_STILL = 0,
  ANIM_X_POS = 1,
  ANIM_X_NEG = 2,
  ANIM_Y_POS = 3,
  ANIM_Y_NEG = 4,
  ANIM_GOAL = 5
};

// Team+level colors (kept for compatibility/UI)
#define C_WHITE 0
#define C_BLUE_1 10
#define C_BLUE_2 11
#define C_ORANGE_1 20
#define C_ORANGE_2 21

static inline bool isBlue_(uint8_t c) {
  return (c == C_BLUE_1 || c == C_BLUE_2);
}
static inline bool isOrange_(uint8_t c) {
  return (c == C_ORANGE_1 || c == C_ORANGE_2);
}

static inline uint8_t makeTeamLevel_(bool blue, uint8_t lvl) {
  if (lvl <= 1) return blue ? C_BLUE_1 : C_ORANGE_1;
  return blue ? C_BLUE_2 : C_ORANGE_2;
}

struct __attribute__((packed)) PairReq {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t desired_fms;
  uint32_t nonce;
};

struct __attribute__((packed)) PairAck {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t fms_id;
  uint8_t channel;
  uint32_t nonce;
};

struct __attribute__((packed)) PollPkt {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t seq;
};

struct __attribute__((packed)) StatusPkt {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t seq;
  int8_t gyroX;
  int8_t gyroY;
  int8_t gyroZ;
  uint8_t moving;
  uint32_t uptime_s;
};

struct __attribute__((packed)) CmdPkt {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t sys_state;
  uint8_t color;     // forced to C_WHITE
  uint8_t anim;      // 0=still, 1=X+, 2=X-, 3=Y+, 4=Y-
  uint16_t beep_hz;  // identify trigger: 3000
  uint16_t beep_ms;  // identify trigger: nonzero
};

static const uint8_t BCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* ===================== SCHEDULING ===================== */
// Comms is now event-driven:
// - Start next cube 10ms after previous slot completes
// - Timeout unresponsive cubes after 50ms
static const uint32_t COMM_NO_RESP_TIMEOUT_MS = 50;
static const uint32_t COMM_NEXT_GAP_MS = 10;

// Keep cmd behavior: send normal cmd shortly after poll
static const uint32_t CMD_SEND_AT_MS = 12;

// Small grace for late frames (optional; kept to preserve robustness)
static const uint32_t STATUS_GRACE_MS = 10;

static const uint32_t PRESENT_MS = 2000;
static const uint32_t SNAPSHOT_MS = 200;
static const uint32_t SSE_PUSH_MS = 200;
static const uint32_t TIMELINE_MS = 10;
static constexpr uint32_t GOAL_ANIM_MS = 1500;
static volatile bool g_goalDebugReq = false;

/* ===================== Game state ===================== */
struct GameState {
  volatile uint8_t sysState = SYS_STANDBY;
  volatile bool matchRunning = false;
  volatile uint32_t matchStartMs = 0;

  volatile uint16_t scoreBlue = 0;
  volatile uint16_t scoreOrange = 0;

  volatile bool nextPullBlue = true;

  volatile bool gate30_done = false;
  volatile bool gate60_done = false;

  volatile uint32_t lastResyncMs = 0;

  volatile uint32_t resetLedUntilMs = 0;  // purple for 1s
  volatile uint32_t endSolidUntilMs = 0;  // solid red for 2s
};

static GameState gs;

/* ===================== Peers ===================== */
#define MAX_CUBES 16

static inline uint8_t popcount20_(uint32_t v) {
  return (uint8_t)__builtin_popcount((unsigned)(v & ((1u << 20) - 1u)));
}

struct Peer {
  bool used = false;
  uint8_t mac[6]{ 0 };
  char id[18]{ 0 };

  int8_t gyroX = 0;
  int8_t gyroY = 0;
  int8_t gyroZ = 0;
  bool moving = false;
  int8_t lastGyroZ = 0;
bool goalAnimActive = false;
uint32_t goalAnimStartMs = 0;
uint32_t lastGoalTriggerMs = 0;
  uint32_t uptime_s = 0;

  uint8_t anim = 0;

  bool awaiting = false;
  uint8_t awaitingSeq = 0;
  uint32_t pollSentMs = 0;
  volatile bool hitThisSlot = false;

  bool slotClosed = false;
  bool cmdSentThisSlot = false;

  uint32_t lastSeenMs = 0;

  uint32_t hist20 = 0;
  uint8_t samples = 0;

  uint8_t color = C_WHITE;

  uint32_t identifyUiUntilMs = 0;

  bool identifyBurstPending = false;
  uint32_t identifyBurstDueMs = 0;

  // last protocol version seen from this cube (PairReq/StatusPkt)
  uint8_t proto = 0;
};

static Peer peers[MAX_CUBES];

static inline void formatMac_(char out18[18], const uint8_t mac[6]) {
  snprintf(out18, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static inline void recordResult_(Peer& p, bool good) {
  p.hist20 = ((p.hist20 << 1) | (good ? 1u : 0u)) & ((1u << 20) - 1u);
  if (p.samples < 20) p.samples++;
}

static inline uint8_t goodCount20_(const Peer& p) {
  return popcount20_(p.hist20);
}

static inline bool shouldRemovePeer_(const Peer& p) {
  if (p.samples < 20) return false;
  return (goodCount20_(p) == 0);
}

static inline const char* healthName_(const Peer& p) {
  uint8_t g = goodCount20_(p);
  if (p.samples >= 20 && g >= 20) return "green";
  if (g < 10) return "red";
  return "yellow";
}

/* ===================== ESPNOW helpers ===================== */
static int findPeerByMac(const uint8_t* mac) {
  for (int i = 0; i < MAX_CUBES; i++)
    if (peers[i].used && memcmp(peers[i].mac, mac, 6) == 0) return i;
  return -1;
}

static int allocPeerSlot() {
  for (int i = 0; i < MAX_CUBES; i++)
    if (!peers[i].used) return i;
  return -1;
}

static void upsertEspNowPeer(const uint8_t* mac, uint8_t channel) {
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = channel;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

static void delEspNowPeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
}

static void removePeer_(int idx) {
  if (idx < 0 || idx >= MAX_CUBES) return;
  if (!peers[idx].used) return;
  delEspNowPeer(peers[idx].mac);
  memset(&peers[idx], 0, sizeof(Peer));
}

/* ===================== mDNS ===================== */
static const char* mdnsName() {
  return (FMS_ID_() == 1) ? "cubic-fms-1" : "cubic-fms-2";
}

/* ===================== Poll scheduler ===================== */
static uint8_t wifiChannel = 1;
static uint8_t pollSeq = 0;
static int rr = -1;

static int nextPollIndex() {
  for (int k = 0; k < MAX_CUBES; k++) {
    int i = (rr + 1 + k) % MAX_CUBES;
    if (peers[i].used) {
      rr = i;
      return i;
    }
  }
  return -1;
}

/* ===================== ESPNOW TX helpers ===================== */
static inline void sendPairAck_(const uint8_t* cubeMac, uint32_t nonce) {
  PairAck ack{};
  ack.proto_ver = PROTO_VER;
  ack.msg_type = MSG_PAIR_ACK;
  ack.fms_id = FMS_ID_();
  ack.channel = wifiChannel;
  ack.nonce = nonce;
  esp_now_send(cubeMac, (uint8_t*)&ack, sizeof(ack));
}

static inline void sendPoll_(int idx, uint32_t now) {
  PollPkt p{};
  p.proto_ver = PROTO_VER;
  p.msg_type = MSG_POLL;
  p.seq = ++pollSeq;

  Peer& peer = peers[idx];
  peer.awaiting = true;
  peer.awaitingSeq = p.seq;
  peer.pollSentMs = now;
  peer.hitThisSlot = false;
  peer.slotClosed = false;
  peer.cmdSentThisSlot = false;

  esp_now_send(peer.mac, (uint8_t*)&p, sizeof(p));
}

static inline void sendCmdRaw_(int idx, uint16_t beep_hz = 0, uint16_t beep_ms = 0) {
  CmdPkt c{};
  c.proto_ver = PROTO_VER;
  c.msg_type = MSG_CMD;
  c.sys_state = (uint8_t)gs.sysState;
  c.color = C_WHITE;          // always white
  c.anim = peers[idx].anim;   // 0/1/2/3/4
  c.beep_hz = beep_hz;
  c.beep_ms = beep_ms;
  esp_now_send(peers[idx].mac, (uint8_t*)&c, sizeof(c));
}

static void triggerGoalDebug_() {
  const uint32_t now = millis();

  for(int i = 0; i < MAX_CUBES; i++){
    if(!peers[i].used) continue;

    peers[i].goalAnimActive = true;
    peers[i].goalAnimStartMs = now;
    peers[i].lastGoalTriggerMs = now;
    peers[i].anim = ANIM_GOAL;

    sendCmdRaw_(i);
  }
}

/* ===================== Game logic ===================== */
static inline uint32_t matchTimeMs() {
  if (!gs.matchRunning) return 0;
  uint32_t t = millis() - gs.matchStartMs;
  if (t > 90000u) t = 90000u;
  return t;
}

static void setAllWhite(bool /*resetLastFace*/) {
  for (int i = 0; i < MAX_CUBES; i++) {
    if (!peers[i].used) continue;
    peers[i].color = C_WHITE;
    peers[i].anim = 0;
  }
}

static inline uint16_t connectedCount() {
  uint16_t cnt = 0;
  uint32_t now = millis();
  for (int i = 0; i < MAX_CUBES; i++) {
    if (!peers[i].used) continue;
    if (now - peers[i].lastSeenMs < PRESENT_MS) cnt++;
  }
  return cnt;
}

static void tallyGateScores() {
  uint16_t b1 = 0, b2 = 0, o1 = 0, o2 = 0;
  const uint32_t now = millis();

  for (int i = 0; i < MAX_CUBES; i++) {
    if (!peers[i].used) continue;
    if (now - peers[i].lastSeenMs > PRESENT_MS) continue;
    switch (peers[i].color) {
      case C_BLUE_1: b1++; break;
      case C_BLUE_2: b2++; break;
      case C_ORANGE_1: o1++; break;
      case C_ORANGE_2: o2++; break;
      default: break;
    }
  }

  gs.scoreBlue += (uint16_t)(b1 * 10 + b2 * 15);
  gs.scoreOrange += (uint16_t)(o1 * 10 + o2 * 15);
}

static void processMovementState(int idx) {
  Peer& p = peers[idx];
  const uint32_t now = millis();

  p.color = C_WHITE;

  // FMS no longer chooses X/Y movement animations.
  // Ball handles local still/rolling/bump by itself.

  if(p.goalAnimActive){
    if(now - p.goalAnimStartMs < GOAL_ANIM_MS){
      p.anim = ANIM_GOAL;
      return;
    } else {
      p.goalAnimActive = false;
    }
  }

  p.anim = ANIM_STILL;
}


/* ===================== Snapshot JSON ===================== */
static constexpr size_t JSON_SZ = 4200;
static char jsonBuf[2][JSON_SZ];
static volatile uint8_t jsonFront = 0;
static portMUX_TYPE jsonMux = portMUX_INITIALIZER_UNLOCKED;

static inline const char* stateName(uint8_t s) {
  switch (s) {
    case SYS_STANDBY: return "standby";
    case SYS_GAME_START: return "game start";
    case SYS_IN_GAME: return "in game";
    case SYS_TIME_GATE: return "time gate";
    case SYS_END_GAME: return "end game";
    case SYS_RESET: return "reset";
    default: return "unknown";
  }
}

static void buildSnapshot() {
  uint32_t now = millis();
  uint8_t back = (jsonFront ^ 1);

  char* out = jsonBuf[back];
  size_t rem = JSON_SZ;

  int n = snprintf(out, rem,
                   "{\"fms_id\":%u,\"mdns\":\"%s.local\",\"ip\":\"%s\",\"state\":\"%s\",\"time_ms\":%lu,"
                   "\"score_blue\":%u,\"score_orange\":%u,\"count\":%u,\"cubes\":[",
                   (unsigned)FMS_ID_(), mdnsName(), ipCache,
                   stateName((uint8_t)gs.sysState), (unsigned long)matchTimeMs(),
                   (unsigned)gs.scoreBlue, (unsigned)gs.scoreOrange, (unsigned)connectedCount());
  if (n < 0) return;
  if ((size_t)n >= rem) n = (int)rem - 1;
  out += n;
  rem -= n;

  bool first = true;
  for (int i = 0; i < MAX_CUBES; i++) {
    if (!peers[i].used) continue;
    bool present = (now - peers[i].lastSeenMs) < PRESENT_MS;

    const char* health = healthName_(peers[i]);
    uint8_t good = goodCount20_(peers[i]);
    uint8_t loss20 = (good <= 20) ? (uint8_t)(20 - good) : 20;

    n = snprintf(out, rem,
                 "%s{\"id\":\"%s\",\"present\":%s,"
                 "\"health\":\"%s\",\"good\":%u,\"loss\":%u,"
                 "\"gyroX\":%d,\"gyroY\":%d,\"gyroZ\":%d,\"moving\":%s,"
                 "\"color\":%u,\"uptime_s\":%lu,\"ident\":%s,"
                 "\"proto\":%u}",
                 first ? "" : ",",
                 peers[i].id,
                 present ? "true" : "false",
                 health, (unsigned)good, (unsigned)loss20,
                 (int)peers[i].gyroX,
                 (int)peers[i].gyroY,
                 (int)peers[i].gyroZ,
                 peers[i].moving ? "true" : "false",
                 (unsigned)peers[i].color,
                 (unsigned long)peers[i].uptime_s,
                 (now < peers[i].identifyUiUntilMs) ? "true" : "false",
                 (unsigned)peers[i].proto);
    if (n < 0) break;
    if ((size_t)n >= rem) n = (int)rem - 1;
    out += n;
    rem -= n;
    first = false;
  }

  snprintf(out, rem, "]}");

  portENTER_CRITICAL(&jsonMux);
  jsonFront = back;
  portEXIT_CRITICAL(&jsonMux);
}

static inline const char* snapshotPtr() {
  uint8_t f;
  portENTER_CRITICAL(&jsonMux);
  f = jsonFront;
  portEXIT_CRITICAL(&jsonMux);
  return jsonBuf[f];
}

/* ===================== Web Server + SSE ===================== */
AsyncWebServer server(80);
AsyncEventSource events("/events");

/* ===================== VIEW PAGE (Scoreboard) ===================== */

static const char VIEW_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CubicChaos — Scoreboard</title>
<style>
:root{
  --bg:#06080c; --panel:#0f141b; --panel2:#0b0f15; --line:#233043;
  --text:#e9f0fa; --muted:#9fb2c7;
  --shadow:0 10px 22px rgba(0,0,0,.35);
  --r:16px;
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0;
  background:radial-gradient(1200px 700px at 20% 0%, #0b1220 0%, var(--bg) 45%, #040509 100%);
  color:var(--text);
  font-family:system-ui,Segoe UI,Roboto,Ubuntu,Arial;
}
.wrap{height:100vh;display:flex;flex-direction:column;gap:10px;padding:10px;}
.main{flex:1 1 auto;min-height:0;display:grid;grid-template-rows: 1.15fr 0.85fr;gap:10px;}
.panel{
  background:linear-gradient(180deg,var(--panel),var(--panel2));
  border:1px solid var(--line);
  border-radius:var(--r);
  box-shadow:var(--shadow);
  padding:10px;
  min-height:0;
  display:flex;
  flex-direction:column;
  align-items:center;
  justify-content:center;
  text-align:center;
}

/* ---- TIME PANEL ---- */
#timePanel{
  container-type: size;
  transition: background 120ms linear, color 120ms linear, border-color 120ms linear;
}
.timeWrap{
  width:100%;
  height:100%;
  display:flex;
  align-items:center;
  justify-content:center;
}
.bigTime{
  font-weight:980;
  line-height:0.92;
  letter-spacing:-0.04em;
  width:100%;
  text-align:center;
  font-size:min(90cqw, 90cqh);
  white-space:nowrap;
}
.bigTime .sec{ font-size:1em; }
.bigTime .dec{
  font-size:0.5em;
  vertical-align:baseline;
  letter-spacing:-0.02em;
}

/* ---- SCORE ---- */
.bigScore{
  font-weight:980;
  line-height:0.92;
  letter-spacing:-0.04em;
  font-size:min(22vw,26vh);
}
.scoreRow{
  width:100%;
  height:100%;
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:10px;
}
.scoreBox{
  border-radius:var(--r);
  border:1px solid var(--line);
  padding:10px;
  display:flex;
  flex-direction:column;
  justify-content:center;
  align-items:center;
}
.scoreBox.orange{
  background:linear-gradient(180deg, rgba(255,138,26,.58), rgba(90,44,8,.75));
  border-color:rgba(255,138,26,.7);
}
.scoreBox.blue{
  background:linear-gradient(180deg, rgba(45,125,255,.6), rgba(18,52,110,.7));
  border-color:rgba(45,125,255,.7);
}

/* ---- FOOTER ---- */
.footer{display:flex;justify-content:space-between;gap:8px;flex-wrap:wrap;}
.footerLeft,.footerRight{display:flex;gap:8px;flex-wrap:wrap;}
.pill{
  border:1px solid rgba(255,255,255,.1);
  border-radius:999px;
  padding:5px 10px;
  font-size:12px;
  color:var(--muted);
  background:rgba(255,255,255,.02);
}
.pill b{color:var(--text)}
.stateTxt{font-weight:950;letter-spacing:.12em;text-transform:uppercase;}
a{color:var(--text);text-decoration:none}
</style>
</head>

<body>
<div class="wrap">
  <div class="main">
    <div class="panel" id="timePanel">
      <div class="timeWrap">
        <div class="bigTime" id="time">
          <span class="sec">90</span><span class="dec">.0</span>
        </div>
      </div>
    </div>

    <div class="panel">
      <div class="scoreRow">
        <div class="scoreBox orange">
          <div class="bigScore" id="so">0</div>
        </div>
        <div class="scoreBox blue">
          <div class="bigScore" id="sb">0</div>
        </div>
      </div>
    </div>
  </div>

  <div class="footer">
    <div class="footerLeft">
      <div class="pill">STATE <b class="stateTxt" id="state">—</b></div>
      <div class="pill">FMS <b id="fms">—</b></div>
      <div class="pill">CUBES <b id="count">0</b></div>
    </div>
    <div class="footerRight">
      <div class="pill">IP <b id="ip">—</b></div>
      <div class="pill"><span id="mdns">—</span></div>
      <div class="pill">CTL <a id="ctl" href="#">—</a></div>
    </div>
  </div>
</div>

<script>
const MATCH_MS = 90000;
let prevState = '';
let startFlashUntil = 0;

function setTimeStyle(bg, fg, border){
  const p=document.getElementById('timePanel');
  if(!p) return;
  p.style.background=bg;
  p.style.color=fg;
  p.style.borderColor=border||'rgba(255,255,255,.1)';
}

function setTimeValue(sec){
  const s = sec.toFixed(1);
  const parts = s.split('.');
  time.innerHTML =
    '<span class="sec">'+parts[0]+'</span>' +
    '<span class="dec">.'+(parts[1]||'0')+'</span>';
}

function apply(j){
  fms.textContent=j.fms_id;
  mdns.textContent=j.mdns;
  ip.textContent=j.ip;
  count.textContent=j.count;
  state.textContent=j.state||'—';

  const url='http://' + j.ip + '/control';
  ctl.textContent=url; ctl.href=url;

  const stateStr=(j.state||'').toLowerCase();
  if(stateStr==='game start' && prevState!=='game start'){
    startFlashUntil=Date.now()+1000;
  }
  prevState=stateStr;

  let remainingMs=MATCH_MS-(j.time_ms||0);
  if(remainingMs<0) remainingMs=0;

  if((j.time_ms||0)===0 && (stateStr==='standby'||stateStr==='reset')){
    remainingMs=MATCH_MS;
  }
  if(stateStr==='end game') remainingMs=0;

  setTimeValue(remainingMs/1000);

  so.textContent=j.score_orange;
  sb.textContent=j.score_blue;

  const sec = remainingMs/1000;
  const now = Date.now();
  if (now < startFlashUntil) {
    setTimeStyle('linear-gradient(180deg, rgba(25,230,160,.95), rgba(10,120,86,.95))',
                 '#ffffff','rgba(25,230,160,.7)');
  } else if (remainingMs === 0) {
    setTimeStyle('linear-gradient(180deg, rgba(255,59,59,.95), rgba(140,20,20,.95))',
                 '#ffffff','rgba(255,59,59,.75)');
  } else if (sec < 10) {
    setTimeStyle('linear-gradient(180deg, rgba(255,255,255,.95), rgba(230,235,245,.95))',
                 '#0b0f15','rgba(255,255,255,.65)');
  } else if (sec < 30) {
    setTimeStyle('linear-gradient(180deg, rgba(45,125,255,.85), rgba(18,52,110,.85))',
                 '#ffffff','rgba(45,125,255,.75)');
  } else {
    setTimeStyle('linear-gradient(180deg,var(--panel),var(--panel2))',
                 'var(--text)','var(--line)');
  }
}

let es=null,lastMsgMs=0;
function openSSE(){
  if(es){try{es.close()}catch(e){} es=null;}
  es=new EventSource('/events');
  es.addEventListener('state',(e)=>{
    lastMsgMs=Date.now();
    try{apply(JSON.parse(e.data));}catch(err){}
  });
  lastMsgMs=Date.now();
}
function watchdog(){ if(Date.now()-lastMsgMs>3500) openSSE(); }
openSSE();
setInterval(watchdog,1000);

document.addEventListener('visibilitychange',()=>{
  if(document.hidden){ if(es){try{es.close()}catch(e){} es=null;} }
  else openSSE();
});
window.addEventListener('pagehide',()=>{
  if(es){try{es.close()}catch(e){} es=null;}
});

fetch('/api/state',{cache:'no-store'}).then(r=>r.json()).then(apply).catch(()=>{});
</script>
</body></html>
)HTML";

/* ===================== CONTROL PAGE ===================== */

static const char CTRL_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CubicChaos — Control</title>
<style>
:root{
  --bg:#06080c; --panel:#0f141b; --panel2:#0b0f15; --line:#233043;
  --text:#e9f0fa; --muted:#9fb2c7;
  --blue:#2d7dff; --orange:#ff8a1a;
  --ok:#19e6a0; --bad:#ff3b3b; --warn:#ffd34d;
  --shadow:0 10px 22px rgba(0,0,0,.35);
  --r:12px;
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0; min-height:100vh;
  background:radial-gradient(1200px 700px at 20% 0%, #0b1220 0%, var(--bg) 45%, #040509 100%);
  color:var(--text);
  font-family:system-ui,Segoe UI,Roboto,Ubuntu,Arial;
}
.wrap{max-width:1400px;margin:0 auto;padding:10px}
.top{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:8px;align-items:stretch;}
.card{
  background:linear-gradient(180deg,var(--panel),var(--panel2));
  border:1px solid var(--line);
  border-radius:var(--r);
  padding:10px;
  box-shadow:var(--shadow);
}
.title{font-weight:980;letter-spacing:.4px}
.meta{margin-top:7px;font-size:12px;color:var(--muted);display:flex;flex-wrap:wrap;gap:7px}
.pill{border:1px solid var(--line);border-radius:999px;padding:3px 9px;background:rgba(255,255,255,.02)}
.pill b{color:var(--text)}

.kpis{
  display:grid;
  grid-template-columns: repeat(auto-fit,minmax(170px,1fr));
  gap:8px;
  margin-top:9px;
}
.kpi{
  border:1px solid rgba(255,255,255,.08);
  border-radius:12px;
  padding:8px 9px 7px;
  background:rgba(0,0,0,.10);
}
.kpi .k{font-size:11px;color:var(--muted);letter-spacing:.14em}
.kpi .v{font-weight:980;font-size:clamp(40px, 6.2vw, 100px);line-height:1;margin-top:6px}
.kpi.blue{border-color:rgba(45,125,255,.35)}
.kpi.blue .v{color:var(--blue)}
.kpi.orange{border-color:rgba(255,138,26,.35)}
.kpi.orange .v{color:var(--orange)}
.kpi.state .v{text-transform:uppercase;letter-spacing:.08em;font-size:clamp(20px, 3.0vw, 42px);}

.controls{display:grid;grid-template-columns: 1fr 1fr;gap:8px;align-items:stretch;margin-top:9px;}
.btnSq{
  width:100%;
  aspect-ratio: 1 / 1;
  border-radius:16px;
  border:1px solid rgba(255,255,255,.10);
  font-weight:980;
  font-size:clamp(17px, 2.9vw, 25px);
  letter-spacing:.10em;
  text-transform:uppercase;
  color:#ffffff;
  cursor:pointer;
  box-shadow:0 10px 22px rgba(0,0,0,.35);
}
.btnSq:active{transform:translateY(1px)}
.btnStart{ background:linear-gradient(180deg, rgba(25,230,160,.95), rgba(10,120,86,.95)); }
.btnStop{  background:linear-gradient(180deg, rgba(255,59,59,.95), rgba(140,20,20,.95)); }
.btnSq:disabled{opacity:.45;cursor:not-allowed}

.small{font-size:11px;color:var(--muted);line-height:1.35;margin-top:7px}
a{color:var(--text);text-decoration:none}
.mono{font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono","Courier New", monospace;}

/* ---- ULTRA COMPACT CUBE GRID ---- */
.grid{
  display:grid;
  grid-template-columns:repeat(auto-fill,minmax(190px,1fr));
  gap:6px;
  margin-top:8px;
}

.cube{
  background:linear-gradient(180deg,var(--panel),var(--panel2));
  border:1px solid var(--line);
  border-radius:12px;
  padding:6px;
  box-shadow:var(--shadow);
}
.cube.health-green{ border-color:rgba(25,230,160,.65); }
.cube.health-yellow{ border-color:rgba(255,211,77,.75); }
.cube.health-red{ border-color:rgba(255,59,59,.85); }

.cubeHead{display:flex;align-items:center;justify-content:space-between;gap:6px;}
.cubeLeft{min-width:0}
.cubeId{font-weight:980;font-size:10px;line-height:1.1}
.badges{display:flex;gap:5px;flex-wrap:wrap;margin-top:4px;max-width:100%}
.badge{
  display:inline-flex;align-items:center;
  font-size:8px;
  letter-spacing:.14em;
  text-transform:uppercase;
  color:var(--muted);
  border:1px solid rgba(255,255,255,.12);
  border-radius:999px;
  padding:2px 6px;
  background:rgba(0,0,0,.10);
  line-height:1;
}
.badge.ident{color:var(--text);border-color:rgba(25,230,160,.55);background:rgba(25,230,160,.10)}

.identBtn{
  border:1px solid rgba(255,255,255,.22);
  border-radius:10px;
  padding:6px 7px;
  font-weight:980;
  font-size:8px;
  letter-spacing:.14em;
  text-transform:uppercase;
  background:linear-gradient(180deg, rgba(255,255,255,.10), rgba(0,0,0,.18));
  color:var(--text);
  cursor:pointer;
  box-shadow:
    0 10px 16px rgba(0,0,0,.35),
    0 0 0 1px rgba(255,255,255,.06) inset;
  user-select:none;
  -webkit-tap-highlight-color: transparent;
  touch-action: manipulation;
  transition: transform 80ms linear, background 120ms linear, border-color 120ms linear, box-shadow 120ms linear;
  white-space:nowrap;
}
.identBtn:active{ transform:translateY(1px) scale(.99); }
.identBtn:disabled{opacity:.45;cursor:not-allowed;box-shadow:none}
.identBtn.on{
  border-color:rgba(25,230,160,.70);
  background:linear-gradient(180deg, rgba(25,230,160,.28), rgba(0,0,0,.20));
  box-shadow:
    0 12px 18px rgba(0,0,0,.38),
    0 0 0 1px rgba(25,230,160,.20) inset,
    0 0 22px rgba(25,230,160,.16);
}

.kv{
  display:grid;
  grid-template-columns: repeat(3, 1fr);
  gap:5px;
  margin-top:6px;
}
.kv .cell{
  border:1px solid rgba(255,255,255,.06);
  border-radius:10px;
  padding:5px 6px;
  background:rgba(0,0,0,.12);
  min-height:32px;
}
.kv .k{
  font-size:8px;
  color:var(--muted);
  letter-spacing:.12em;
  text-transform:uppercase;
}
.kv .v{
  font-weight:950;
  margin-top:2px;
  font-size:11px;
  line-height:1.05;
}

/* ---- UPTIME COLOR INDICATION ---- */
.cell.uptime.ok{
  background:linear-gradient(
    180deg,
    rgba(25,230,160,.85),
    rgba(10,120,86,.85)
  );
  border-color:rgba(25,230,160,.95);
  box-shadow:0 0 14px rgba(25,230,160,.55);
}

.cell.uptime.warn{
  background:linear-gradient(
    180deg,
    rgba(255,211,77,.90),
    rgba(160,120,10,.90)
  );
  border-color:rgba(255,211,77,.95);
  box-shadow:0 0 14px rgba(255,211,77,.55);
}

.cell.uptime.bad{
  background:linear-gradient(
    180deg,
    rgba(255,59,59,.92),
    rgba(140,20,20,.92)
  );
  border-color:rgba(255,59,59,.98);
  box-shadow:0 0 16px rgba(255,59,59,.60);
}

.colorChip{
  display:inline-flex;align-items:center;
  padding:2px 7px;
  border-radius:999px;
  border:1px solid rgba(255,255,255,.18);
  font-size:9px;
  font-weight:950;
  letter-spacing:.06em;
}
</style></head><body>

<div class="wrap">
  <div class="top">
    <div class="card">
      <div class="title">CubicChaos — Control</div>
      <div class="meta">
        <div class="pill">FMS <b id="fms">—</b></div>
        <div class="pill"><span id="mdns">—</span></div>
        <div class="pill">IP <b id="ip">—</b></div>
        <div class="pill">Cubes <b id="count">0</b></div>
      </div>

      <div class="kpis">
        <div class="kpi orange"><div class="k">ORANGE</div><div class="v" id="so">0</div></div>
        <div class="kpi blue"><div class="k">BLUE</div><div class="v" id="sb">0</div></div>
        <div class="kpi"><div class="k">TIME</div><div class="v" id="time">90.0</div></div>
        <div class="kpi state"><div class="k">STATE</div><div class="v" id="state">—</div></div>
      </div>
    </div>

    <div class="card">
      <div class="title">Match</div>
      <div class="controls">
        <button class="btnSq btnStart" onclick="post('/api/start')">START</button>
        <button class="btnSq btnStop"  onclick="post('/api/reset')">STOP</button>
        <button class="btnSq" onclick="post('/api/goaltest')">GOAL TEST</button>
      </div>
      <div class="small" style="margin-top:9px">START begins timeline. STOP triggers reset sequence.</div>
      <div class="small" style="margin-top:6px"><a href="/diag">/diag</a></div>
    </div>
  </div>

  <main class="grid" id="grid"></main>
</div>

<script>
async function post(url){
  const ctrl=new AbortController();
  const t=setTimeout(()=>ctrl.abort(),1200);
  try{ await fetch(url,{method:'POST',cache:'no-store',signal:ctrl.signal}); }catch(e){}
  clearTimeout(t);
}

let identUiUntil = {}; // idx -> epoch ms
async function postIdentify(url){
  const ctrl=new AbortController();
  const t=setTimeout(()=>ctrl.abort(),2200);
  try{ await fetch(url,{method:'POST',cache:'no-store',signal:ctrl.signal}); }catch(e){}
  clearTimeout(t);
}
function identify(ev, i, el){
  try{ ev.preventDefault(); ev.stopPropagation(); }catch(e){}
  const until = Date.now() + 2100;
  identUiUntil[i] = until;

  if (el){
    el.classList.add('on');
    el.disabled = true;
    setTimeout(()=>{
      if ((identUiUntil[i]||0) <= Date.now()){
        el.disabled = false;
        el.classList.remove('on');
      }
    }, 900);
  }
  postIdentify('/api/identify?i='+i);
}

function colorName(c){
  switch(c){
    case 10: return 'BLUE_1';
    case 11: return 'BLUE_2';
    case 20: return 'ORANGE_1';
    case 21: return 'ORANGE_2';
    default:return 'WHITE';
  }
}
function colorStyle(c){
  switch(c){
    case 10:
    case 11: return {bg:'rgba(45,125,255,.75)', fg:'#ffffff'};
    case 20:
    case 21: return {bg:'rgba(255,138,26,.80)', fg:'#101014'};
    default:return {bg:'rgba(255,255,255,.85)', fg:'#0b0f15'};
  }
}
function n0(v){
  if (typeof v === 'number' && isFinite(v)) return v;
  const x = parseInt(String(v ?? '0'), 10);
  return isFinite(x) ? x : 0;
}
function str01(v){
  if (typeof v === 'boolean') return v ? 'true' : 'false';
  return String(v ?? '—');
}
function badgeHealthClass(h){
  h=(h||'yellow').toLowerCase();
  if(h==='green') return 'health-green';
  if(h==='red') return 'health-red';
  return 'health-yellow';
}

/* Uptime thresholds (seconds):
   > 3600 => red
   > 2200 => yellow
   > 0    => green
*/
function uptimeClass(u){
  u = n0(u);
  if (u > 3600) return 'bad';
  if (u > 2200) return 'warn';
  if (u > 0)    return 'ok';
  return '';
}

const MATCH_MS = 90000;

function apply(j){
  fms.textContent=j.fms_id; mdns.textContent=j.mdns; ip.textContent=j.ip;
  count.textContent=j.count;
  state.textContent=(j.state||'—');

  const t = n0(j.time_ms);
  let remaining = MATCH_MS - t;
  if (remaining < 0) remaining = 0;

  const st = String(j.state||'').toLowerCase();
  if (t === 0 && (st === 'standby' || st === 'reset')) remaining = MATCH_MS;
  if (st === 'end game') remaining = 0;

  time.textContent=(remaining/1000).toFixed(1);

  so.textContent=j.score_orange;
  sb.textContent=j.score_blue;

  const g=document.getElementById('grid');
  g.innerHTML='';

  (j.cubes||[]).forEach((c,idx)=>{
    const d=document.createElement('div');
    const h=(c.health||'yellow').toLowerCase();
    d.className='cube ' + badgeHealthClass(h);

    const present = !!c.present;

    const colorVal = n0(c.color);
    const colorTxt = colorName(colorVal);
    const cs = colorStyle(colorVal);

    const uiIdent = (identUiUntil[idx] || 0) > Date.now();
    const identOn = !!c.ident || uiIdent;

    const proto = (typeof c.proto !== 'undefined') ? n0(c.proto) : '—';

    const up = n0(c.uptime_s);
    const upCls = uptimeClass(up);

    d.innerHTML = `
      <div class="cubeHead">
        <div class="cubeLeft">
          <div class="cubeId mono">${c.id || ('Cube '+(idx+1))}</div>
          <div class="badges">
            <span class="badge">PROTO ${proto}</span>
            ${identOn ? `<span class="badge ident">IDENT</span>` : ``}
          </div>
        </div>
        <button class="identBtn ${identOn?'on':''}" ${present?'':'disabled'}
          onpointerdown="identify(event, ${idx}, this)"
          onclick="event.preventDefault(); return false;"
          aria-label="Identify cube ${idx+1}">IDENTIFY</button>
      </div>

      <div class="kv">
        <div class="cell"><div class="k">GYRO X</div><div class="v">${n0(c.gyroX)}</div></div>
        <div class="cell"><div class="k">GYRO Y</div><div class="v">${n0(c.gyroY)}</div></div>
        <div class="cell"><div class="k">GYRO Z</div><div class="v">${n0(c.gyroZ)}</div></div>

        <div class="cell"><div class="k">MOVING</div><div class="v">${str01(c.moving)}</div></div>
        <div class="cell uptime ${upCls}">
          <div class="k">UPTIME</div><div class="v">${up}s</div>
        </div>
        <div class="cell">
          <div class="k">COLOR</div>
          <div class="v"><span class="colorChip" style="background:${cs.bg};color:${cs.fg}">${colorTxt}</span></div>
        </div>
      </div>
    `;
    g.appendChild(d);
  });
}

let es=null, lastMsgMs=0;
function openSSE(){
  if(es){ try{ es.close(); }catch(e){} es=null; }
  es=new EventSource('/events');
  es.addEventListener('state',(e)=>{
    lastMsgMs=Date.now();
    try{ apply(JSON.parse(e.data)); }catch(err){}
  });
  lastMsgMs=Date.now();
}
function watchdog(){ if(Date.now()-lastMsgMs>3500) openSSE(); }
openSSE(); setInterval(watchdog,1000);

document.addEventListener('visibilitychange',()=>{
  if(document.hidden){ if(es){ try{ es.close(); }catch(e){} es=null; } }
  else openSSE();
});
window.addEventListener('pagehide',()=>{ if(es){ try{ es.close(); }catch(e){} es=null; } });

fetch('/api/state',{cache:'no-store'}).then(r=>r.json()).then(apply).catch(()=>{});
</script>
</body></html>

)HTML";

/* ===================== /diag HTML (auto-updating 1 Hz) ===================== */

static const char DIAG_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CubicChaos — Diagnostics</title>
<style>
:root{
  --bg:#06080c; --panel:#0f141b; --panel2:#0b0f15; --line:#233043;
  --text:#e9f0fa; --muted:#9fb2c7;
  --ok:#19e6a0; --warn:#ffd34d; --bad:#ff3b3b;
  --blue:#2d7dff; --orange:#ff8a1a;
  --shadow:0 10px 22px rgba(0,0,0,.35);
  --r:16px;
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0;
  background:radial-gradient(1200px 700px at 20% 0%, #0b1220 0%, var(--bg) 45%, #040509 100%);
  color:var(--text);
  font-family:system-ui,Segoe UI,Roboto,Ubuntu,Arial;
}
a{color:var(--text);text-decoration:none}
.wrap{max-width:1600px;margin:0 auto;padding:12px;display:flex;flex-direction:column;gap:10px}

.topbar{
  display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap;
}
.h1{display:flex;align-items:baseline;gap:10px;flex-wrap:wrap}
.h1 .t{font-weight:980;letter-spacing:.2px}
.h1 .sub{font-size:12px;color:var(--muted)}
.actions{display:flex;gap:8px;flex-wrap:wrap}
.btn{
  border:1px solid rgba(255,255,255,.14);
  background:rgba(255,255,255,.04);
  color:var(--text);
  border-radius:12px;
  padding:8px 10px;
  font-weight:900;
  font-size:12px;
  letter-spacing:.08em;
  text-transform:uppercase;
  cursor:pointer;
}
.btn:active{transform:translateY(1px)}
.btn.ok{border-color:rgba(25,230,160,.35);background:rgba(25,230,160,.08)}
.btn.warn{border-color:rgba(255,211,77,.35);background:rgba(255,211,77,.08)}
.btn.bad{border-color:rgba(255,59,59,.35);background:rgba(255,59,59,.08)}

.grid{
  display:grid;
  grid-template-columns: 1fr 1fr;
  gap:10px;
}
@media (max-width:1200px){ .grid{grid-template-columns:1fr;} }

.card{
  background:linear-gradient(180deg,var(--panel),var(--panel2));
  border:1px solid var(--line);
  border-radius:var(--r);
  box-shadow:var(--shadow);
  padding:12px;
  min-width:0;
}
.card h2{
  margin:0 0 10px 0;
  font-size:12px;
  letter-spacing:.14em;
  text-transform:uppercase;
  color:var(--muted);
  font-weight:950;
}

.pills{display:flex;gap:8px;flex-wrap:wrap}
.pill{
  border:1px solid rgba(255,255,255,.12);
  border-radius:999px;
  padding:5px 10px;
  background:rgba(255,255,255,.02);
  font-size:12px;
  color:var(--muted);
  white-space:nowrap;
}
.pill b{color:var(--text)}
.pill.ok{border-color:rgba(25,230,160,.35);background:rgba(25,230,160,.07)}
.pill.warn{border-color:rgba(255,211,77,.35);background:rgba(255,211,77,.07)}
.pill.bad{border-color:rgba(255,59,59,.35);background:rgba(255,59,59,.07)}

.kpis{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}
.kpi{
  border:1px solid rgba(255,255,255,.08);
  border-radius:14px;
  padding:10px;
  background:rgba(0,0,0,.10);
  min-width:0;
}
.kpi .k{font-size:10px;color:var(--muted);letter-spacing:.14em;text-transform:uppercase}
.kpi .v{margin-top:6px;font-weight:980;font-size:28px;line-height:1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.kpi.blue{border-color:rgba(45,125,255,.35)}
.kpi.blue .v{color:var(--blue)}
.kpi.orange{border-color:rgba(255,138,26,.35)}
.kpi.orange .v{color:var(--orange)}

.hr{height:1px;background:rgba(255,255,255,.06);margin:12px 0}

.mono{font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono","Courier New", monospace;}
.small{font-size:12px;color:var(--muted);line-height:1.35}

.tableWrap{overflow:auto;border:1px solid rgba(255,255,255,.08);border-radius:14px}
table{width:100%;border-collapse:separate;border-spacing:0;min-width:1100px}
th,td{padding:8px 10px;border-bottom:1px solid rgba(255,255,255,.06);font-size:12px;text-align:left;white-space:nowrap}
th{
  position:sticky;top:0;z-index:2;
  background:rgba(10,14,20,.96);
  color:var(--muted);
  font-size:11px;
  letter-spacing:.12em;
  text-transform:uppercase;
  font-weight:950;
}
tr:last-child td{border-bottom:none}

.badge{
  display:inline-flex;align-items:center;
  border-radius:999px;
  padding:2px 8px;
  border:1px solid rgba(255,255,255,.14);
  font-size:11px;
  letter-spacing:.08em;
}
.b-ok{border-color:rgba(25,230,160,.55);color:var(--ok);background:rgba(25,230,160,.08)}
.b-warn{border-color:rgba(255,211,77,.60);color:var(--warn);background:rgba(255,211,77,.08)}
.b-bad{border-color:rgba(255,59,59,.70);color:var(--bad);background:rgba(255,59,59,.08)}

.chip{
  display:inline-flex;align-items:center;
  padding:2px 8px;
  border-radius:999px;
  border:1px solid rgba(255,255,255,.14);
  font-size:11px;
  font-weight:950;
  letter-spacing:.06em;
}
.chip.blue{background:rgba(45,125,255,.18);border-color:rgba(45,125,255,.35);color:#fff}
.chip.orange{background:rgba(255,138,26,.18);border-color:rgba(255,138,26,.35);color:#fff}
.chip.white{background:rgba(255,255,255,.10);border-color:rgba(255,255,255,.18);color:var(--text)}

details{border:1px solid rgba(255,255,255,.08);border-radius:14px;padding:10px;background:rgba(0,0,0,.10)}
summary{cursor:pointer;color:var(--text);font-weight:950;letter-spacing:.08em;text-transform:uppercase;font-size:12px}
.code{
  margin-top:10px;
  font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono","Courier New", monospace;
  font-size:12px;
  line-height:1.35;
  padding:10px;
  border-radius:14px;
  border:1px solid rgba(255,255,255,.08);
  background:rgba(0,0,0,.18);
  white-space:pre-wrap;
  word-break:break-word;
}
</style>
</head>

<body>
<div class="wrap">

  <div class="topbar">
    <div class="h1">
      <div class="t">CubicChaos — Diagnostics</div>
      <div class="sub" id="hdrSub">—</div>
    </div>
    <div class="actions">
      <button class="btn" onclick="forceFetch()">Refresh</button>
      <button class="btn" id="autoBtn" onclick="toggleAuto()">Auto: ON</button>
      <a class="btn" href="/control">Control</a>
      <a class="btn" href="/">Scoreboard</a>
    </div>
  </div>

  <div class="grid">
    <!-- GROUP: OVERALL SYSTEM DATA -->
    <div class="card">
      <h2>Overall system data</h2>
      <div class="pills">
        <span class="pill"><span class="mono">FMS</span> <b id="fms">—</b></span>
        <span class="pill"><span class="mono">MDNS</span> <b id="mdns">—</b></span>
        <span class="pill"><span class="mono">IP</span> <b id="ip">—</b></span>
        <span class="pill"><span class="mono">STATE</span> <b id="state">—</b></span>
      </div>

      <div class="hr"></div>

      <div class="pills">
        <span class="pill" id="pSse"><span class="mono">SSE</span> <b id="sse">—</b></span>
        <span class="pill" id="pSnap"><span class="mono">SNAPSHOT_AGE</span> <b id="snapAge">—</b></span>
        <span class="pill" id="pDiag"><span class="mono">DIAG_AGE</span> <b id="diagAge">—</b></span>
        <span class="pill" id="pWifiAge"><span class="mono">WIFI_AGE</span> <b id="wifiAge">—</b></span>
      </div>

      <div class="hr"></div>

      <div class="small">
        Notes:
        <ul style="margin:6px 0 0 18px;padding:0;color:var(--muted)">
          <li><span class="mono">/api/state</span> is the authoritative structured snapshot.</li>
          <li><span class="mono">/api/diag</span> provides WiFi/Channel text and a redundant peer list.</li>
          <li><span class="mono">/api/wifi</span> provides WiFi technical/client fields.</li>
        </ul>
      </div>
    </div>

    <!-- GROUP: GAME DATA -->
    <div class="card">
      <h2>Game data</h2>
      <div class="kpis">
        <div class="kpi"><div class="k">TIME_MS</div><div class="v" id="time">0</div></div>
        <div class="kpi orange"><div class="k">ORANGE</div><div class="v" id="so">0</div></div>
        <div class="kpi blue"><div class="k">BLUE</div><div class="v" id="sb">0</div></div>
      </div>

      <div class="hr"></div>

      <div class="pills">
        <span class="pill"><span class="mono">MATCH_RUNNING</span> <b id="mr">—</b></span>
        <span class="pill"><span class="mono">CONNECTED_CUBES</span> <b id="cc">0</b></span>
      </div>

      <div class="small" style="margin-top:10px">
        Connected cubes is computed by the firmware and exposed as <span class="mono">count</span>.
      </div>
    </div>

    <!-- GROUP: WIFI TECHNICAL DATA -->
    <div class="card">
      <h2>WiFi technical data</h2>
      <div class="pills">
        <span class="pill" id="pWifi"><span class="mono">STATUS</span> <b id="wifi">—</b></span>
        <span class="pill"><span class="mono">LOCAL_IP</span> <b id="wifiIp">—</b></span>
      </div>

      <div class="hr"></div>

      <div class="pills">
        <span class="pill" id="pRssi"><span class="mono">RSSI</span> <b id="rssi">—</b></span>
        <span class="pill"><span class="mono">SSID</span> <b id="ssid">—</b></span>
        <span class="pill"><span class="mono">BSSID</span> <b id="bssid">—</b></span>
        <span class="pill"><span class="mono">GW</span> <b id="gw">—</b></span>
        <span class="pill"><span class="mono">NETMASK</span> <b id="netmask">—</b></span>
        <span class="pill"><span class="mono">DNS</span> <b id="dns">—</b></span>
      </div>
    </div>

    <!-- GROUP: WIFI CLIENT DATA -->
    <div class="card">
      <h2>WiFi client data</h2>
      <div class="pills">
        <span class="pill"><span class="mono">STA_MAC</span> <b id="staMac">—</b></span>
        <span class="pill"><span class="mono">AP_BSSID</span> <b id="apBssid">—</b></span>
        <span class="pill"><span class="mono">DHCP_LEASE_S</span> <b id="leaseS">—</b></span>
        <span class="pill"><span class="mono">RECONNECTS</span> <b id="recon">—</b></span>
        <span class="pill"><span class="mono">LAST_WIFI_LOSS_MS</span> <b id="lastLoss">—</b></span>
      </div>

      <div class="small" style="margin-top:10px">
        DHCP lease seconds may be <span class="mono">—</span> depending on core/netif.
      </div>
    </div>

    <!-- GROUP: ESP-NOW DATA -->
    <div class="card">
      <h2>ESP-NOW data</h2>
      <div class="pills">
        <span class="pill" id="pChan"><span class="mono">CHANNEL</span> <b id="ch">—</b></span>
        <span class="pill warn"><span class="mono">TX_FAIL</span> <b>N/A</b></span>
        <span class="pill warn"><span class="mono">RX_PKTS</span> <b>N/A</b></span>
        <span class="pill warn"><span class="mono">PAIR_REQS</span> <b>N/A</b></span>
        <span class="pill warn"><span class="mono">LAST_RESYNC_MS</span> <b>N/A</b></span>
      </div>

      <div class="small" style="margin-top:10px">
        Current firmware exposes: <span class="mono">Channel</span> and per-ball <span class="mono">good/loss</span>.
      </div>

      <div class="hr"></div>

      <details>
        <summary>Raw /api/diag text</summary>
        <div class="code" id="rawDiag">—</div>
      </details>
    </div>

    <!-- GROUP: BALL DATA -->
    <div class="card" style="grid-column:1 / -1;">
      <h2>Ball data</h2>

      <div class="pills">
        <span class="pill"><span class="mono">FILTER</span> <b class="mono">type in browser find (Ctrl+F)</b></span>
        <span class="pill"><span class="mono">HEALTH</span> <b class="mono">comm + uptime</b></span>
      </div>

      <div class="hr"></div>

      <div class="tableWrap">
        <table>
          <thead>
            <tr>
              <th>#</th>
              <th>ID</th>
              <th>Present</th>
              <th>Health</th>
              <th>Good</th>
              <th>Loss</th>
              <th>Proto</th>
              <th>GyroX</th>
              <th>GyroY</th>
              <th>GyroZ</th>
              <th>Moving</th>
              <th>Color</th>
              <th>Uptime_s</th>
              <th>Ident</th>
            </tr>
          </thead>
          <tbody id="rows">
            <tr><td colspan="14" class="small">loading...</td></tr>
          </tbody>
        </table>
      </div>

      <div class="hr"></div>

      <details>
        <summary>Raw /api/state JSON</summary>
        <div class="code" id="rawJson">—</div>
      </details>
    </div>
  </div>
</div>

<script>
let autoOn = true;

let lastSnapTs = 0;
let lastDiagTs = 0;
let lastWifiTs = 0;
let lastSseTs  = 0;
let sseOk = false;

let snap = null;
let diagText = '';
let wifiJ = null;

function n0(v){
  if (typeof v === 'number' && isFinite(v)) return v;
  const x = parseInt(String(v ?? '0'), 10);
  return isFinite(x) ? x : 0;
}
function pillSet(id, val, sev){
  const el = document.getElementById(id);
  if(!el) return;
  el.classList.remove('ok','warn','bad');
  if(sev==='green') el.classList.add('ok');
  else if(sev==='yellow') el.classList.add('warn');
  else if(sev==='red') el.classList.add('bad');
  el.querySelector('b').textContent = val;
}

function sevRank(s){
  s=(s||'yellow').toLowerCase();
  if(s==='red') return 2;
  if(s==='yellow') return 1;
  return 0;
}
function rankToSev(r){ return (r>=2)?'red':(r>=1)?'yellow':'green'; }
function badgeClass(sev){
  sev=(sev||'yellow').toLowerCase();
  if(sev==='green') return 'badge b-ok';
  if(sev==='red') return 'badge b-bad';
  return 'badge b-warn';
}

function commHealth(good){
  good = n0(good);
  if (good >= 20) return 'green';
  if (good < 10) return 'red';
  return 'yellow';
}
function uptimeHealth(u){
  u = n0(u);
  if (u > 3600) return 'red';
  if (u > 2200) return 'yellow';
  if (u > 0)    return 'green';
  return 'yellow';
}
function combinedHealth(good, uptime_s){
  const c = commHealth(good);
  const u = uptimeHealth(uptime_s);
  const worst = Math.max(sevRank(c), sevRank(u));
  return rankToSev(worst);
}

function colorChip(c){
  c=n0(c);
  if (c===10 || c===11) return '<span class="chip blue">BLUE</span>';
  if (c===20 || c===21) return '<span class="chip orange">ORANGE</span>';
  return '<span class="chip white">WHITE</span>';
}
function boolTxt(v){
  if (typeof v === 'boolean') return v ? '1' : '0';
  return String(v ?? '—');
}

function parseDiagText(txt){
  const out = { wifi:'—', channel:'—', matchRunning:'—' };
  const lines = String(txt||'').split('\n');
  for(const line of lines){
    if (line.startsWith('WiFi: ')) out.wifi = line.substring(6).trim();
    if (line.startsWith('Channel: ')) out.channel = line.substring(9).trim();
    if (line.startsWith('MatchRunning: ')) out.matchRunning = line.substring(13).trim();
  }
  return out;
}

function hdr(){
  const f = snap?.fms_id ?? '—';
  const m = snap?.mdns ?? '—';
  document.getElementById('hdrSub').textContent = `FMS ${f} • ${m}`;
}

function renderOverallAndGame(){
  if(!snap) return;

  fms.textContent = snap.fms_id ?? '—';
  mdns.textContent = snap.mdns ?? '—';
  ip.textContent = snap.ip ?? '—';
  state.textContent = snap.state ?? '—';

  time.textContent = n0(snap.time_ms);
  so.textContent = n0(snap.score_orange);
  sb.textContent = n0(snap.score_blue);
  cc.textContent = n0(snap.count);

  wifiIp.textContent = snap.ip ?? '—';

  rawJson.textContent = JSON.stringify(snap, null, 2);
}

function renderWifiEspNow(){
  const d = parseDiagText(diagText);
  rawDiag.textContent = diagText || '—';

  const wifiVal = d.wifi || '—';
  const wifiSev = String(wifiVal).toUpperCase().includes('CONNECTED') ? 'green' : 'red';
  wifi.textContent = wifiVal;
  pillSet('pWifi', wifiVal, wifiSev);

  const chVal = d.channel || '—';
  ch.textContent = chVal;
  pillSet('pChan', chVal, 'yellow');

  mr.textContent = d.matchRunning || '—';
}

function renderWifiFields(){
  if(!wifiJ) return;

  const st = String(wifiJ.status || '—');
  const ok = (st.toUpperCase() === 'CONNECTED');
  pillSet('pWifi', st, ok ? 'green' : 'red');

  const r = (typeof wifiJ.rssi === 'number') ? wifiJ.rssi : null;
  const rtxt = (r === null) ? '—' : `${r} dBm`;
  const rsev = (r === null) ? 'yellow' : (r < -75) ? 'red' : (r < -65) ? 'yellow' : 'green';
  pillSet('pRssi', rtxt, rsev);

  rssi.textContent = rtxt;
  ssid.textContent = wifiJ.ssid ?? '—';
  bssid.textContent = wifiJ.bssid ?? '—';
  gw.textContent = wifiJ.gw ?? '—';
  netmask.textContent = wifiJ.netmask ?? '—';
  dns.textContent = wifiJ.dns ?? '—';

  staMac.textContent = wifiJ.sta_mac ?? '—';
  apBssid.textContent = wifiJ.ap_bssid ?? '—';

  leaseS.textContent = (wifiJ.dhcp_lease_s !== null && typeof wifiJ.dhcp_lease_s !== 'undefined') ? String(wifiJ.dhcp_lease_s) : '—';
  recon.textContent = (typeof wifiJ.reconnects === 'number') ? String(wifiJ.reconnects) : '—';

  if (typeof wifiJ.last_wifi_loss_ms === 'number') {
    lastLoss.textContent = (wifiJ.last_wifi_loss_ms === 4294967295) ? '—' : String(wifiJ.last_wifi_loss_ms);
  } else {
    lastLoss.textContent = '—';
  }
}

function renderCubes(){
  if(!snap) return;
  const cubes = Array.isArray(snap.cubes) ? snap.cubes : [];
  const tb = document.getElementById('rows');

  if(!cubes.length){
    tb.innerHTML = `<tr><td colspan="14" class="small">no balls connected</td></tr>`;
    return;
  }

  tb.innerHTML = cubes.map((c, i) => {
    const h = combinedHealth(c.good, c.uptime_s);
    return `
      <tr>
        <td>${i+1}</td>
        <td class="mono">${c.id ?? '—'}</td>
        <td>${boolTxt(c.present)}</td>
        <td><span class="${badgeClass(h)}">${h.toUpperCase()}</span></td>
        <td>${n0(c.good)}</td>
        <td>${n0(c.loss)}</td>
        <td>${n0(c.proto)}</td>
        <td>${n0(c.gyroX)}</td>
        <td>${n0(c.gyroY)}</td>
        <td>${n0(c.gyroZ)}</td>
        <td>${boolTxt(c.moving)}</td>
        <td>${colorChip(c.color)}</td>
        <td>${n0(c.uptime_s)}</td>
        <td>${boolTxt(c.ident)}</td>
      </tr>
    `;
  }).join('');
}

function renderAges(){
  const now = Date.now();

  const snapAgeMs = lastSnapTs ? (now - lastSnapTs) : null;
  const diagAgeMs = lastDiagTs ? (now - lastDiagTs) : null;
  const wifiAgeMs = lastWifiTs ? (now - lastWifiTs) : null;
  const sseAgeMs  = lastSseTs  ? (now - lastSseTs)  : null;

  pillSet('pSnap', snapAgeMs===null ? '—' : `${snapAgeMs} ms`, snapAgeMs===null ? 'yellow' : (snapAgeMs < 1500 ? 'green' : snapAgeMs < 5000 ? 'yellow' : 'red'));
  pillSet('pDiag', diagAgeMs===null ? '—' : `${diagAgeMs} ms`, diagAgeMs===null ? 'yellow' : (diagAgeMs < 2500 ? 'green' : diagAgeMs < 7000 ? 'yellow' : 'red'));
  pillSet('pWifiAge', wifiAgeMs===null ? '—' : `${wifiAgeMs} ms`, wifiAgeMs===null ? 'yellow' : (wifiAgeMs < 2500 ? 'green' : wifiAgeMs < 7000 ? 'yellow' : 'red'));
  pillSet('pSse', sseAgeMs===null ? '—' : `${sseAgeMs} ms`, sseAgeMs===null ? 'yellow' : (sseAgeMs < 1500 ? 'green' : sseAgeMs < 5000 ? 'yellow' : 'red'));

  sse.textContent = sseOk ? 'LIVE' : 'RETRY';
  snapAge.textContent = snapAgeMs===null ? '—' : `${snapAgeMs} ms`;
  diagAge.textContent = diagAgeMs===null ? '—' : `${diagAgeMs} ms`;
  wifiAge.textContent = wifiAgeMs===null ? '—' : `${wifiAgeMs} ms`;
}

function renderAll(){
  hdr();
  renderOverallAndGame();
  renderWifiEspNow();
  renderWifiFields();
  renderCubes();
  renderAges();
}

async function fetchState(){
  const r = await fetch('/api/state', {cache:'no-store'});
  snap = await r.json();
  lastSnapTs = Date.now();
}
async function fetchDiag(){
  const r = await fetch('/api/diag', {cache:'no-store'});
  diagText = await r.text();
  lastDiagTs = Date.now();
}
async function fetchWifi(){
  const r = await fetch('/api/wifi', {cache:'no-store'});
  wifiJ = await r.json();
  lastWifiTs = Date.now();
}

async function forceFetch(){
  try{
    await Promise.all([fetchState(), fetchDiag(), fetchWifi()]);
    renderAll();
  }catch(e){}
}

function toggleAuto(){
  autoOn = !autoOn;
  document.getElementById('autoBtn').textContent = `Auto: ${autoOn ? 'ON' : 'OFF'}`;
}

let es = null;
function openSSE(){
  if(es){ try{ es.close(); }catch(e){} es = null; }
  es = new EventSource('/events');
  es.addEventListener('state', (e) => {
    try{
      snap = JSON.parse(e.data);
      lastSnapTs = Date.now();
      lastSseTs = Date.now();
      sseOk = true;
      renderAll();
    }catch(err){}
  });
  es.onerror = () => { sseOk = false; };
}

setInterval(async () => {
  renderAges();
  if(!autoOn) return;
  try{
    await Promise.all([fetchDiag(), fetchWifi()]);
    renderAll();
  }catch(e){}
}, 1000);

openSSE();
forceFetch();

document.addEventListener('visibilitychange',()=>{
  if(document.hidden){
    if(es){ try{ es.close(); }catch(e){} es = null; }
  } else {
    openSSE();
    if(autoOn) forceFetch();
  }
});
window.addEventListener('pagehide',()=>{ if(es){ try{ es.close(); }catch(e){} es = null; } });
</script>
</body></html>
)HTML";

/* ===================== NEW DIAG LED SYSTEM (self-contained) ===================== */

class DiagLedSystem {
public:
  struct Inputs {
    bool wifiOk = true;
    uint8_t sysState = 0;
    bool stopActive = false;
    uint32_t stopStartMs = 0;
    uint8_t connectedCubes = 0;
  };

  void begin(CRGB* leds,
             uint8_t count,
             uint8_t brightness = 160,
             const uint8_t* ringOrder /*optional*/ = nullptr,
             uint16_t bootHoldMs = 800) {
    _leds = leds;
    _count = (count > MAX_LEDS) ? MAX_LEDS : count;
    _brightness = brightness;

    for (uint8_t i = 0; i < _count; i++) _ringOrder[i] = i;
    if (ringOrder) {
      for (uint8_t i = 0; i < _count; i++) _ringOrder[i] = ringOrder[i];
    }

    FastLED.setBrightness(_brightness);

    _bootUntilMs = millis() + (uint32_t)bootHoldMs;
    fill_solid(_leds, _count, CRGB::Blue);
    FastLED.show();

    _lastTickMs = 0;
  }

  void setStateValues(uint8_t standby, uint8_t start, uint8_t inGame, uint8_t stopEnd, uint8_t timeGate) {
    _S_STANDBY = standby;
    _S_START = start;
    _S_INGAME = inGame;
    _S_STOPEND = stopEnd;
    _S_TIMEGATE = timeGate;
  }

  void setTickMs(uint16_t ms) {
    _tickMs = ms;
  }

  void tick(uint32_t now, const Inputs& in) {
    if (!_leds || _count == 0) return;
    if ((uint32_t)(now - _lastTickMs) < _tickMs) return;
    _lastTickMs = now;

    if ((int32_t)(now - _bootUntilMs) < 0) {
      fill_solid(_leds, _count, CRGB::Blue);
      applyMissingOverlay_(now, in.connectedCubes);
      FastLED.show();
      return;
    }

    if (!in.wifiOk) {
      renderChase_(CRGB::Blue, 40, 80, now);
      applyMissingOverlay_(now, in.connectedCubes);
      FastLED.show();
      return;
    }

    const bool stop = in.stopActive || (in.sysState == _S_STOPEND);
    if (stop) {
      if ((int32_t)(now - (in.stopStartMs + 2000u)) < 0) {
        fill_solid(_leds, _count, CRGB::Red);
      } else {
        renderChase_(CRGB::Red, 60, 110, now);
      }
      applyMissingOverlay_(now, in.connectedCubes);
      FastLED.show();
      return;
    }

    if (in.sysState == _S_START) {
      renderChase_(CRGB::Green, 0, 70, now);
      applyMissingOverlay_(now, in.connectedCubes);
      FastLED.show();
      return;
    }

    if (in.sysState == _S_STANDBY) {
      renderBreatheAll_(CRGB::Green, 0.0035f, now);
      applyMissingOverlay_(now, in.connectedCubes);
      FastLED.show();
      return;
    }

    if (in.sysState == _S_INGAME) {
      renderRainbow_(18, now);
      applyMissingOverlay_(now, in.connectedCubes);
      FastLED.show();
      return;
    }

    if (in.sysState == _S_TIMEGATE) {
      fill_solid(_leds, _count, CRGB::White);
      applyMissingOverlay_(now, in.connectedCubes);
      FastLED.show();
      return;
    }

    fill_solid(_leds, _count, CRGB::Purple);
    applyMissingOverlay_(now, in.connectedCubes);
    FastLED.show();
  }

private:
  static constexpr uint8_t MAX_LEDS = 16;

  CRGB* _leds = nullptr;
  uint8_t _count = 0;
  uint8_t _brightness = 160;

  uint8_t _ringOrder[MAX_LEDS]{};

  uint16_t _tickMs = 20;
  uint32_t _lastTickMs = 0;

  uint32_t _bootUntilMs = 0;

  uint8_t _S_STANDBY = 0;
  uint8_t _S_START = 1;
  uint8_t _S_INGAME = 2;
  uint8_t _S_STOPEND = 4;
  uint8_t _S_TIMEGATE = 5;

  void renderChase_(const CRGB& color,
                    uint8_t trailDim,
                    uint16_t speedMs,
                    uint32_t now) {
    fill_solid(_leds, _count, CRGB::Black);
    const uint8_t step = (uint8_t)((now / speedMs) % _count);

    for (uint8_t k = 0; k < _count; k++) {
      const uint8_t idx = _ringOrder[k];
      if (k == step) {
        _leds[idx] = color;
      } else if (trailDim) {
        CRGB dim = color;
        dim.nscale8_video(trailDim);
        _leds[idx] = dim;
      }
    }
  }

  void renderBreatheAll_(const CRGB& base, float omega, uint32_t now) {
    const uint8_t b = (uint8_t)(128 + (int)(127.0f * sinf(now * omega)));
    CRGB c = base;
    c.nscale8_video(b);
    fill_solid(_leds, _count, c);
  }

  void renderRainbow_(uint16_t speedMs, uint32_t now) {
    const uint8_t baseHue = (uint8_t)((now / speedMs) & 0xFF);
    for (uint8_t k = 0; k < _count; k++) {
      const uint8_t idx = _ringOrder[k];
      CHSV hsv((uint8_t)(baseHue + (k * (256 / _count))), 255, 255);
      hsv2rgb_rainbow(hsv, _leds[idx]);
    }
  }

  void applyMissingOverlay_(uint32_t now, uint8_t connectedCubes) {
    if (connectedCubes >= 9) return;
    const uint8_t step = (uint8_t)((now / 90) % _count);
    const uint8_t idx = _ringOrder[step];
    _leds[idx] = CRGB::Red;
  }
};

static DiagLedSystem g_diagLeds;

/* ===================== ESPNOW RX ===================== */
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data || len < 2) return;
  if (data[0] != PROTO_VER) return;

  const uint8_t* src = info->src_addr;
  const uint8_t mt = data[1];
  const uint32_t now = millis();

  if (mt == MSG_PAIR_REQ && len == (int)sizeof(PairReq)) {
    PairReq pr;
    memcpy(&pr, data, sizeof(pr));
    if (pr.desired_fms != FMS_ID_()) return;

    int idx = findPeerByMac(src);
    if (idx < 0) {
      idx = allocPeerSlot();
      if (idx < 0) return;

      peers[idx] = Peer{};
      peers[idx].used = true;
      memcpy(peers[idx].mac, src, 6);
      formatMac_(peers[idx].id, peers[idx].mac);

      peers[idx].color = C_WHITE;
      peers[idx].anim = 0;
      peers[idx].gyroX = 0;
      peers[idx].gyroY = 0;
      peers[idx].gyroZ = 0;
      peers[idx].moving = false;
    }

    peers[idx].proto = pr.proto_ver;

    upsertEspNowPeer(src, wifiChannel);
    sendPairAck_(src, pr.nonce);
    peers[idx].lastSeenMs = now;
    return;
  }

  if (mt == MSG_STATUS && len == (int)sizeof(StatusPkt)) {
    StatusPkt st;
    memcpy(&st, data, sizeof(st));

    int idx = findPeerByMac(src);
    if (idx < 0) return;

    Peer& p = peers[idx];

    if (!p.awaiting) return;
    if (st.seq != p.awaitingSeq) return;
    if ((now - p.pollSentMs) > (COMM_NO_RESP_TIMEOUT_MS + STATUS_GRACE_MS)) return;
    if (p.slotClosed) return;

    p.proto = st.proto_ver;

    p.lastSeenMs = now;
    p.gyroX = st.gyroX;
    p.gyroY = st.gyroY;
    p.gyroZ = st.gyroZ;
    p.moving = (st.moving != 0);
    p.uptime_s = st.uptime_s;

    if (gs.sysState != SYS_END_GAME) processMovementState(idx);

    p.hitThisSlot = true;
    return;
  }
}

static void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  (void)status;
}

/* ===================== ESPNOW channel + peer resync ===================== */
static void resyncEspNowChannelAndPeers_() {
  wifi_second_chan_t sc{};
  uint8_t ch = wifiChannel;
  esp_wifi_get_channel(&ch, &sc);
  wifiChannel = ch;

  upsertEspNowPeer(BCAST, wifiChannel);
  for (int i = 0; i < MAX_CUBES; i++) {
    if (peers[i].used) upsertEspNowPeer(peers[i].mac, wifiChannel);
  }

  gs.lastResyncMs = millis();
  Serial.print("[FMS] ESPNOW resync -> channel=");
  Serial.println(wifiChannel);
}

/* ===================== Web helpers ===================== */
static inline void sendNoStore(AsyncWebServerRequest* req, int code, const char* type, const char* body) {
  AsyncWebServerResponse* r = req->beginResponse(code, type, body);
  r->addHeader("Cache-Control", "no-store");
  r->addHeader("Connection", "close");
  req->send(r);
}

static inline void sendNoStoreJson(AsyncWebServerRequest* req) {
  AsyncWebServerResponse* r = req->beginResponse(200, "application/json", snapshotPtr());
  r->addHeader("Cache-Control", "no-store");
  r->addHeader("Connection", "close");
  req->send(r);
}

/* ===================== /api/diag text ===================== */
static void buildDiagText_(String& s) {
  s.reserve(2200);

  s = "";
  s += "CubicChaos FMS /diag\n";
  s += "-----------------\n";
  s += "FMS_ID: ";
  s += String((int)FMS_ID_());
  s += "\n";
  s += "MDNS: ";
  s += String(mdnsName());
  s += ".local\n";
  s += "IP: ";
  s += String(ipCache);
  s += "\n";
  s += "WiFi: ";
  s += (WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
  s += "\n";
  s += "Channel: ";
  s += String((unsigned)wifiChannel);
  s += "\n";
  s += "State: ";
  s += stateName((uint8_t)gs.sysState);
  s += "\n";
  s += "MatchRunning: ";
  s += (gs.matchRunning ? "true" : "false");
  s += "\n";
  s += "Time_ms: ";
  s += String((unsigned long)matchTimeMs());
  s += "\n";
  s += "Score Orange: ";
  s += String(gs.scoreOrange);
  s += "\n";
  s += "Score Blue: ";
  s += String(gs.scoreBlue);
  s += "\n";
  s += "Connected Count: ";
  s += String((unsigned)connectedCount());
  s += "\n";
  s += "\nPeers:\n";

  const uint32_t now = millis();
  for (int i = 0; i < MAX_CUBES; i++) {
    if (!peers[i].used) continue;
    bool present = (now - peers[i].lastSeenMs) < PRESENT_MS;

    s += "  [";
    s += String(i);
    s += "] ";
    s += peers[i].id;
    s += "  present=";
    s += (present ? "1" : "0");
    s += "  health=";
    s += healthName_(peers[i]);
    s += "  good=";
    s += String((unsigned)goodCount20_(peers[i]));
    s += "  proto=";
    s += String((unsigned)peers[i].proto);
    s += "  gyroX=";
    s += String((int)peers[i].gyroX);
    s += "  gyroY=";
    s += String((int)peers[i].gyroY);
    s += "  gyroZ=";
    s += String((int)peers[i].gyroZ);
    s += "  moving=";
    s += (peers[i].moving ? "1" : "0");
    s += "  color=";
    s += String((unsigned)peers[i].color);
    s += "  uptime_s=";
    s += String((unsigned long)peers[i].uptime_s);
    s += "\n";
  }
}

/* ===================== /api/wifi JSON ===================== */
static uint32_t g_wifiReconnects = 0;
static uint32_t g_lastWiFiLossAtMs = 0;

static void buildWifiJson_(String& s) {
  s.reserve(700);

  bool connected = (WiFi.status() == WL_CONNECTED) && ipIsValid_(WiFi.localIP());

  String ssidStr = connected ? WiFi.SSID() : String();
  String bssidStr = connected ? WiFi.BSSIDstr() : String();
  String macStr = WiFi.macAddress();

  IPAddress gw = WiFi.gatewayIP();
  IPAddress nm = WiFi.subnetMask();
  IPAddress dns = WiFi.dnsIP();

  int32_t rssi = connected ? WiFi.RSSI() : 0;

  uint32_t lastLossAge = 0xFFFFFFFFu;
  if (g_lastWiFiLossAtMs != 0) lastLossAge = (uint32_t)(millis() - g_lastWiFiLossAtMs);

  s = "";
  s += "{";
  s += "\"status\":\"";
  s += (connected ? "CONNECTED" : "DISCONNECTED");
  s += "\",";

  s += "\"rssi\":";
  if (connected) s += String(rssi);
  else s += "null";
  s += ",";

  s += "\"ssid\":";
  if (connected) { s += "\""; s += ssidStr; s += "\""; }
  else s += "null";
  s += ",";

  s += "\"bssid\":";
  if (connected) { s += "\""; s += bssidStr; s += "\""; }
  else s += "null";
  s += ",";

  s += "\"gw\":";
  if (connected) { s += "\""; s += String(gw); s += "\""; }
  else s += "null";
  s += ",";

  s += "\"netmask\":";
  if (connected) { s += "\""; s += String(nm); s += "\""; }
  else s += "null";
  s += ",";

  s += "\"dns\":";
  if (connected) { s += "\""; s += String(dns); s += "\""; }
  else s += "null";
  s += ",";

  s += "\"sta_mac\":\"";
  s += macStr;
  s += "\",";

  s += "\"ap_bssid\":";
  if (connected) { s += "\""; s += bssidStr; s += "\""; }
  else s += "null";
  s += ",";

  s += "\"dhcp_lease_s\":null,";
  s += "\"reconnects\":";
  s += String((unsigned long)g_wifiReconnects);
  s += ",";
  s += "\"last_wifi_loss_ms\":";
  s += String((unsigned long)lastLossAge);

  s += "}";
}

static void enterStandby() {
  gs.sysState = SYS_STANDBY;
  gs.matchRunning = false;
  gs.matchStartMs = 0;
  gs.scoreBlue = 0;
  gs.scoreOrange = 0;
  gs.gate30_done = false;
  gs.gate60_done = false;
  setAllWhite(true);
}

static void enterStart() {
  gs.sysState = SYS_GAME_START;
  gs.matchRunning = true;
  gs.matchStartMs = millis();
  gs.gate30_done = false;
  gs.gate60_done = false;
  setAllWhite(true);
}

static void enterInGame() {
  gs.sysState = SYS_IN_GAME;
}

static void enterTimeGate() {
  tallyGateScores();
  setAllWhite(true);
  gs.sysState = SYS_TIME_GATE;
}

static uint32_t g_endStartMs = 0;

static void enterEndGame() {
  tallyGateScores();
  gs.sysState = SYS_END_GAME;
  gs.matchRunning = false;
  gs.endSolidUntilMs = millis() + 2000;
  g_endStartMs = millis();
}

static void enterReset() {
  gs.sysState = SYS_RESET;
  gs.matchRunning = false;
  gs.resetLedUntilMs = millis() + 1000;
}

static void timelineTick() {
  static uint32_t tgStartMs = 0;
  static uint32_t resetStartMs = 0;

  uint32_t now = millis();

  if (gs.sysState == SYS_GAME_START && gs.matchRunning) {
    if (now - gs.matchStartMs >= 1000) enterInGame();
  }

  if (gs.sysState == SYS_TIME_GATE) {
    if (tgStartMs == 0) tgStartMs = now;
    if (now - tgStartMs >= 500) {
      tgStartMs = 0;
      gs.sysState = SYS_IN_GAME;
    }
  } else tgStartMs = 0;

  if (gs.sysState == SYS_RESET) {
    if (resetStartMs == 0) resetStartMs = now;
    if (now - resetStartMs >= 1200) {
      resetStartMs = 0;
      enterStandby();
    }
  } else resetStartMs = 0;

  if (gs.sysState == SYS_IN_GAME && gs.matchRunning) {
    uint32_t t = matchTimeMs();
    if (!gs.gate30_done && t >= 30000) {
      gs.gate30_done = true;
      enterTimeGate();
    }
    if (!gs.gate60_done && t >= 60000) {
      gs.gate60_done = true;
      enterTimeGate();
    }
    if (t >= 90000) enterEndGame();
  }
}
class FmsSoundSystemLEDC {
public:
  void begin(uint8_t pin, uint8_t channel = 0) {
    _pin = pin;
    _ch = channel;

    pinMode(_pin, OUTPUT);

    // ESP32 core 3.x compatible
    ledcAttach(_pin, 2000, 10);
    ledcWriteTone(_pin, 0);
  }

  void playStartupBlocking() {
    beep_(1200, 80);
    delay(100);
    beep_(1800, 80);
    delay(100);
    beep_(2400, 120);
  }

  void enqueueBeep(uint16_t hz, uint16_t ms) {
    if (_queueLen < MAX_QUEUE) {
      _queue[_queueLen++] = {hz, ms};
    }
  }

  void tickState(uint8_t state) {
    (void)state;
  }

  void tick() {
    uint32_t now = millis();

    if (_active) {
      if ((int32_t)(now - _endMs) >= 0) {
        ledcWriteTone(_pin, 0);
        _active = false;
      }
      return;
    }

    if (_queueLen > 0) {
      ToneStep t = _queue[0];
      for (uint8_t i = 1; i < _queueLen; i++) {
        _queue[i - 1] = _queue[i];
      }
      _queueLen--;

      ledcWriteTone(_pin, t.hz);
      _endMs = now + t.ms;
      _active = true;
    }
  }

private:
  struct ToneStep {
    uint16_t hz;
    uint16_t ms;
  };

  static const uint8_t MAX_QUEUE = 8;

  uint8_t _pin = 0;
  uint8_t _ch = 0;
  ToneStep _queue[MAX_QUEUE];
  uint8_t _queueLen = 0;
  bool _active = false;
  uint32_t _endMs = 0;

  void beep_(uint16_t hz, uint16_t ms) {
    ledcWriteTone(_pin, hz);
    delay(ms);
    ledcWriteTone(_pin, 0);
  }
};
static FmsSoundSystemLEDC g_fmsSound;

/* ===================== Fast comms scheduler ===================== */
static int slotIdx = -1;
static uint32_t slotStartMs = 0;
static uint32_t slotDeadlineMs = 0;
static uint32_t nextSlotDueMs = 0;

static void commsTickFast_() {
  const uint32_t now = millis();

  auto sendNormalCmdOnce_ = [&]() {
    if (slotIdx < 0) return;
    Peer& p = peers[slotIdx];
    if (p.cmdSentThisSlot) return;
    sendCmdRaw_(slotIdx);
    p.cmdSentThisSlot = true;
  };

  if (slotIdx < 0) {
    if ((int32_t)(now - nextSlotDueMs) < 0) return;

    int idx = nextPollIndex();
    if (idx < 0) return;

    slotIdx = idx;
    slotStartMs = now;
    slotDeadlineMs = now + COMM_NO_RESP_TIMEOUT_MS;
    sendPoll_(slotIdx, now);
    return;
  }

  Peer& p = peers[slotIdx];

  if (p.hitThisSlot) {
    p.hitThisSlot = false;
    p.awaiting = false;
    p.slotClosed = true;
    recordResult_(p, true);
    sendNormalCmdOnce_();

    if (shouldRemovePeer_(p)) removePeer_(slotIdx);

    slotIdx = -1;
    nextSlotDueMs = now + COMM_NEXT_GAP_MS;
    return;
  }

  if (p.awaiting && (uint32_t)(now - p.pollSentMs) >= CMD_SEND_AT_MS) {
    sendNormalCmdOnce_();
  }

  if ((int32_t)(now - slotDeadlineMs) >= 0) {
    p.awaiting = false;
    recordResult_(p, false);
    sendNormalCmdOnce_();

    if (shouldRemovePeer_(p)) removePeer_(slotIdx);

    slotIdx = -1;
    nextSlotDueMs = now + COMM_NEXT_GAP_MS;
    return;
  }

  (void)slotStartMs;
}

/* ===================== Identify burst (non-blocking) ===================== */
static void identifyBurstTick_() {
  const uint32_t now = millis();
  for (int i = 0; i < MAX_CUBES; i++) {
    if (!peers[i].used) continue;
    if (!peers[i].identifyBurstPending) continue;
    if ((int32_t)(now - peers[i].identifyBurstDueMs) < 0) continue;

    sendCmdRaw_(i, 3000, 70);
    peers[i].identifyBurstPending = false;
  }
}

/* ===================== Diagnostic LEDs ===================== */
static void diagLedsTick_() {
  const uint32_t now = millis();

  DiagLedSystem::Inputs in{};
  in.wifiOk = (WiFi.status() == WL_CONNECTED) && ipIsValid_(WiFi.localIP());
  in.sysState = (uint8_t)gs.sysState;
  in.stopActive = (gs.sysState == SYS_END_GAME);
  in.stopStartMs = g_endStartMs;
  in.connectedCubes = (uint8_t)connectedCount();

  g_diagLeds.tick(now, in);
}

/* ===================== Main control requests ===================== */
volatile bool g_startReq = false;
volatile bool g_resetReq = false;

// static void applyControlRequests() {
//   if (g_startReq) {
//     g_startReq = false;
//     if (gs.sysState == SYS_END_GAME) enterStandby();
//     enterStart();
//   }
//   if (g_resetReq) {
//     g_resetReq = false;
//     enterReset();
//   }
// }

static void applyControlRequests() {
  if (g_startReq) {
    g_startReq = false;
    if (gs.sysState == SYS_END_GAME) enterStandby();
    enterStart();
  }

  if (g_resetReq) {
    g_resetReq = false;
    enterReset();
  }

  if (g_goalDebugReq) {
    g_goalDebugReq = false;
    triggerGoalDebug_();
  }
}

/* ===================== Web setup ===================== */
static void setupWeb() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    AsyncWebServerResponse* r = req->beginResponse_P(200, "text/html", VIEW_HTML);
    r->addHeader("Cache-Control", "no-store");
    req->send(r);
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest* req) {
    AsyncWebServerResponse* r = req->beginResponse_P(200, "text/html", CTRL_HTML);
    r->addHeader("Cache-Control", "no-store");
    req->send(r);
  });

  server.on("/diag", HTTP_GET, [](AsyncWebServerRequest* req) {
    AsyncWebServerResponse* r = req->beginResponse_P(200, "text/html", DIAG_HTML);
    r->addHeader("Cache-Control", "no-store");
    req->send(r);
  });

  server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest* req) {
    sendNoStoreJson(req);
  });

  server.on("/api/diag", HTTP_GET, [](AsyncWebServerRequest* req) {
    String t;
    buildDiagText_(t);
    sendNoStore(req, 200, "text/plain", t.c_str());
  });

  server.on("/api/wifi", HTTP_GET, [](AsyncWebServerRequest* req) {
    String w;
    buildWifiJson_(w);
    sendNoStore(req, 200, "application/json", w.c_str());
  });

  server.on("/api/start", HTTP_POST, [](AsyncWebServerRequest* req) {
    g_startReq = true;
    sendNoStore(req, 200, "text/plain", "OK");
  });

  server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest* req) {
    g_resetReq = true;
    sendNoStore(req, 200, "text/plain", "OK");
  });

  server.on("/api/identify", HTTP_POST, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("i")) {
      sendNoStore(req, 400, "text/plain", "missing i");
      return;
    }
    int idx = req->getParam("i")->value().toInt();
    if (idx < 0 || idx >= MAX_CUBES || !peers[idx].used) {
      sendNoStore(req, 404, "text/plain", "bad index");
      return;
    }

    const uint32_t now = millis();
    peers[idx].identifyUiUntilMs = now + 2000;

    sendCmdRaw_(idx, 3000, 70);

    peers[idx].identifyBurstPending = true;
    peers[idx].identifyBurstDueMs = now + 120;

    sendNoStore(req, 200, "text/plain", "OK");
  });

  server.on("/api/goaltest", HTTP_POST, [](AsyncWebServerRequest* req) {
  g_goalDebugReq = true;
  sendNoStore(req, 200, "text/plain", "OK");
});

  events.onConnect([](AsyncEventSourceClient* c) {
    c->send(snapshotPtr(), "state", millis());
  });
  server.addHandler(&events);

  server.begin();
}

/* ===================== Setup / Loop ===================== */
void setup() {
  Serial.begin(115200);
  delay(200);

  g_fmsSound.begin(SOUND_PIN, 0);
  g_fmsSound.playStartupBlocking();

  pinMode(FMS_SEL_PIN, INPUT);
  delay(5);
  g_fmsSelBoot = (digitalRead(FMS_SEL_PIN) != 0);
  g_fmsId = g_fmsSelBoot ? 1 : 2;

  FastLED.addLeds<NEOPIXEL, DIAG_LED_PIN>(diagLeds, DIAG_LED_COUNT);
  FastLED.setBrightness(160);
  fill_solid(diagLeds, DIAG_LED_COUNT, CRGB::Blue);
  FastLED.show();

  g_diagLeds.begin(diagLeds, DIAG_LED_COUNT, 160, RING_ORDER, 800);
  g_diagLeds.setTickMs((uint16_t)DIAG_LED_TICK_MS);
  g_diagLeds.setStateValues(SYS_STANDBY, SYS_GAME_START, SYS_IN_GAME, SYS_END_GAME, SYS_TIME_GATE);

  if (!connectWiFiRobust_()) {
    delay(300);
    ESP.restart();
  }

  IPAddress ip = WiFi.localIP();
  snprintf(ipCache, sizeof(ipCache), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

  wifi_second_chan_t sc{};
  esp_wifi_get_channel(&wifiChannel, &sc);
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_now_init() != ESP_OK) {
    delay(200);
    ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  upsertEspNowPeer(BCAST, wifiChannel);

  enterStandby();
  buildSnapshot();

  if (MDNS.begin(mdnsName())) {
    MDNS.addService("http", "tcp", 80);
  }

  setupWeb();

  Serial.print("FMS ");
  Serial.print((int)FMS_ID_());
  Serial.print(" IP ");
  Serial.println(ipCache);
  Serial.print("View    http://");
  Serial.print(mdnsName());
  Serial.println(".local/");
  Serial.print("Control http://");
  Serial.print(mdnsName());
  Serial.println(".local/control");
  Serial.print("Diag    http://");
  Serial.print(mdnsName());
  Serial.println(".local/diag");
}

void loop() {
  static uint32_t lastTimelineMs = 0;
  static uint32_t lastSnapMs = 0;
  static uint32_t lastSseMs = 0;

  const uint32_t now = millis();

  applyControlRequests();

  {
    static uint32_t lastSwCheckMs = 0;
    if (now - lastSwCheckMs >= 50) {
      lastSwCheckMs = now;
      const bool swNow = (digitalRead(FMS_SEL_PIN) != 0);
      if (swNow != g_fmsSelBoot) ESP.restart();
    }
  }

  identifyBurstTick_();
  diagLedsTick_();

  if (now - lastTimelineMs >= TIMELINE_MS) {
    lastTimelineMs = now;
    timelineTick();
  }

  g_fmsSound.tickState((uint8_t)gs.sysState);
  g_fmsSound.tick();

  commsTickFast_();

  if (now - lastSnapMs >= SNAPSHOT_MS) {
    lastSnapMs = now;
    buildSnapshot();
  }

  if (now - lastSseMs >= SSE_PUSH_MS) {
    lastSseMs = now;
    events.send(snapshotPtr(), "state", now);
  }

  static uint32_t lastWiFiCheckMs = 0;
  static bool wasConnected = false;

  if (millis() - lastWiFiCheckMs >= 2000) {
    lastWiFiCheckMs = millis();

    IPAddress ip = WiFi.localIP();
    bool ok = (WiFi.status() == WL_CONNECTED) && ipIsValid_(ip);

    if (!ok) {
      if (wasConnected) g_lastWiFiLossAtMs = millis();

      Serial.println("WiFi lost/invalid IP. Reconnecting.");
      if (connectWiFiRobust_(4000, 4)) {
        g_wifiReconnects++;

        IPAddress nip = WiFi.localIP();
        snprintf(ipCache, sizeof(ipCache), "%u.%u.%u.%u", nip[0], nip[1], nip[2], nip[3]);

        MDNS.end();
        if (MDNS.begin(mdnsName())) MDNS.addService("http", "tcp", 80);

        resyncEspNowChannelAndPeers_();
        wasConnected = true;
      } else {
        ESP.restart();
      }
    } else {
      if (!wasConnected) resyncEspNowChannelAndPeers_();
      wasConnected = true;
    }
  }

  g_fmsSound.tickState((uint8_t)gs.sysState);
  g_fmsSound.tick();

  static int8_t lastCountdownSec = -1;

  if (gs.matchRunning && gs.sysState == SYS_IN_GAME) {
    uint32_t t = matchTimeMs();
    uint32_t remainingMs = (t >= 90000u) ? 0u : (90000u - t);

    int8_t secRem = (remainingMs > 0) ? (int8_t)((remainingMs + 999u) / 1000u) : 0;

    if (secRem >= 1 && secRem <= 10) {
      if (secRem != lastCountdownSec) {
        lastCountdownSec = secRem;
        g_fmsSound.enqueueBeep(2000, 35);
      }
    } else {
      lastCountdownSec = -1;
    }
  } else {
    lastCountdownSec = -1;
  }
}