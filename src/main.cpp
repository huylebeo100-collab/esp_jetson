/*
 * ESP32-S3 : dieu khien 2 dong co DC co encoder, PID toc do 50 Hz
 *  - USB CDC (Serial)  : debug + tuning bang tay  (giu nguyen menu cu)
 *  - UART1  (Link)     : nhan lenh van toc tu Jetson, tra ve odom
 *
 * Lenh Jetson gui xuong (moi lenh 1 dong, ket thuc '\n'):
 *   V <v_mps> <w_rad_s>   dieu khien vi sai (giong cmd_vel)
 *   M <vl_mps> <vr_mps>   dat truc tiep toc do tung banh (m/s)
 *   R <rpm_l> <rpm_r>     dat truc tiep rpm (debug)
 *   E <0|1>               tat / bat PID
 *   S                     dung khan
 *   K <1|2> <kp> <ki> <kd>  chinh he so
 *   B <1|2> <deadband>
 *   P                     ping -> tra "PONG"
 *
 * ESP32 tra ve 50 Hz:
 *   O <t_ms> <vl_mps> <vr_mps> <tick_l> <tick_r>
 */

#include <Arduino.h>
#include <string.h>
#include "config.h"
#include "wheel.h"

HardwareSerial Link(1);

Wheel WL(M1_EN_L, M1_EN_R, M1_PWM_L, M1_PWM_R, 0, 1, M1_ENC_A, M1_ENC_B,
         M1_DIR, M1_ENC_DIR, M1_KP, M1_KI, M1_KD, M1_DEADBAND);   // TRAI
Wheel WR(M2_EN_L, M2_EN_R, M2_PWM_L, M2_PWM_R, 2, 3, M2_ENC_A, M2_ENC_B,
         M2_DIR, M2_ENC_DIR, M2_KP, M2_KI, M2_KD, M2_DEADBAND);   // PHAI

bool     pidOn   = false;
bool     syncReq = false;
bool     odomOn  = ODOM_ON_BOOT;    // gui odom cho Jetson
bool     plotOn  = false;   // in do thi ra USB khi tuning
int      sel     = 3;       // 1=trai 2=phai 3=ca hai  (chi dung khi tuning tay)
uint32_t lastCmdMs = 0;
uint32_t tStart    = 0;
bool     timedOut  = false;
bool     wdEnabled = false;   // watchdog CHI bat khi lenh den tu Jetson
bool     echoOn    = false;   // in lai byte nhan duoc (chan doan terminal)
float    curV      = 0;       // van toc dai dang dat bang phim +/-
float    curW      = 0;       // van toc goc dang dat bang phim </>

// phim don, chay ngay khi nhan, KHONG can Enter va KHONG can tham so
static const char *NOARG = "123cfghxksj?qQ+-0<>e";

// ---------------------------------------------------------------- utils
static int splitFloats(const char *s, float *out, int maxn) {
  int n = 0;
  const char *p = s;
  while (n < maxn) {
    while (*p == ' ' || *p == ',' || *p == '\t') p++;
    if (!*p) break;
    char *end;
    float v = strtof(p, &end);
    if (end == p) break;
    out[n++] = v;
    p = end;
  }
  return n;
}

static bool readLine(Stream &s, String &buf, String &out) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) { out = buf; buf = ""; return true; }
    } else {
      buf += c;
      if (buf.length() > 120) buf = "";     // chong tran
    }
  }
  return false;
}

// ---------------------------------------------------------------- lenh
void stopAll() {
  pidOn = false;
  WL.target = WR.target = 0;
  WL.setPWM(0);
  WR.setPWM(0);
  WL.resetState();
  WR.resetState();
  syncReq = true;
}

void enablePID(bool on) {
  if (on) { syncReq = true; pidOn = true; tStart = millis(); }
  else    { stopAll(); }
}

void setWheelMps(float vl, float vr) {
  WL.setTargetMps(vl);
  WR.setTargetMps(vr);
  lastCmdMs = millis();
  timedOut  = false;
  if (!pidOn) enablePID(true);
}

void setCmdVel(float v, float w) {
  setWheelMps(v - w * WHEEL_BASE * 0.5f,
              v + w * WHEEL_BASE * 0.5f);
}

// UART1 cua Jetson la cong console: log khoi dong chay qua day, va
// bootloader coi moi byte nhan duoc la nguoi dung go phim. Nen im
// tuyet doi cho toi khi Jetson boot xong.
static inline bool linkReady() { return millis() >= BOOT_QUIET_MS; }
// ---------------------------------------------------------------- Jetson
void handleJetson(const String &line) {
  char cmd = toupper(line[0]);
  if (!strchr("VMRESZPKB", cmd)) return;
  wdEnabled = true;                 // Jetson dieu khien -> bat watchdog
  odomOn    = true;                 // Jetson da len -> mo dong odom
  

  float a[4];
  int n = splitFloats(line.c_str() + 1, a, 4);

  switch (cmd) {
    case 'V': if (n >= 2) setCmdVel(a[0], a[1]);              break;
    case 'M': if (n >= 2) setWheelMps(a[0], a[1]);            break;
    case 'R': if (n >= 2) { WL.setTargetRpm(a[0]);
                            WR.setTargetRpm(a[1]);
                            lastCmdMs = millis(); timedOut = false;
                            if (!pidOn) enablePID(true); }    break;
    case 'E': enablePID(n >= 1 && a[0] > 0.5f);               break;
    case 'S': stopAll();                                      break;
    case 'Z': WL.clearCount(); WR.clearCount(); syncReq = true;
              Link.println("ZOK");                            break;
    case 'P': Link.println("PONG");                           break;
    case 'K': if (n >= 4) {
                Wheel &w = (a[0] < 1.5f) ? WL : WR;
                w.Kp = a[1]; w.Ki = a[2]; w.Kd = a[3];
              }                                               break;
    case 'B': if (n >= 2) {
                Wheel &w = (a[0] < 1.5f) ? WL : WR;
                w.deadband = constrain((int)a[1], 0, 500);
              }                                               break;
    default: break;
  }
}

// ---------------------------------------------------------------- tuning
void showHelp() {
  Serial.println(F("\n=== MENU TUNING (USB) ==="));
  Serial.println(F("--- PHIM DON: bam la chay ngay, khong can Enter ---"));
  Serial.println(F(" 1/2/3 : chon TRAI / PHAI / ca hai"));
  Serial.println(F(" c     : kiem tra cap PWM-encoder + chieu quay"));
  Serial.println(F(" f     : do deadband"));
  Serial.println(F(" +     : target +0.05 m/s    | -  : target -0.05 m/s"));
  Serial.println(F(" 0     : target ve 0"));
  Serial.println(F(" <     : quay TRAI tai cho   | >  : quay PHAI tai cho"));
  Serial.println(F(" q     : PWM tho +1023       | Q  : PWM tho -1023"));
  Serial.println(F(" g / h : BAT / TAT PID       | x  : DUNG HAN"));
  Serial.println(F(" s     : bat/tat in do thi   | j  : bat/tat odom UART1"));
  Serial.println(F(" k     : xem tham so         | e  : echo byte nhan duoc"));
  Serial.println(F("--- CAN GO CA DONG + Enter (neu terminal cho phep) ---"));
  Serial.println(F(" t<so> rpm | v<so> m/s | w<v> <w> cmd_vel"));
  Serial.println(F(" p/i/d<so> Kp/Ki/Kd | b<so> deadband | m<so> PWM tho | o step"));
}

void printParams() {
  Serial.printf("[TRAI ] Kp=%.3f Ki=%.3f Kd=%.4f db=%d tgt=%.1frpm (%.3f m/s)\n",
                WL.Kp, WL.Ki, WL.Kd, WL.deadband, WL.target, WL.target * RPM_TO_MPS);
  Serial.printf("[PHAI ] Kp=%.3f Ki=%.3f Kd=%.4f db=%d tgt=%.1frpm (%.3f m/s)\n",
                WR.Kp, WR.Ki, WR.Kd, WR.deadband, WR.target, WR.target * RPM_TO_MPS);
  Serial.printf("pid=%s sel=%d  banh D=%.0fmm chuvi=%.4fm  1m/s=%.1frpm  vmax=%.2fm/s\n",
                pidOn ? "ON" : "OFF", sel, WHEEL_D * 1000, WHEEL_C, MPS_TO_RPM, MAX_MPS);
}

void findDeadband() {
  Serial.println(F(">> Do deadband, ke banh len cao..."));
  pidOn = false; WL.setPWM(0); WR.setPWM(0); delay(500);
  int f1 = 0, f2 = 0;
  for (int d = 0; d <= 400; d += 10) {
    WL.clearCount(); WR.clearCount();
    WL.setPWM(d); WR.setPWM(d);
    delay(300);
    long long c1 = llabs(WL.count()), c2 = llabs(WR.count());
    Serial.printf("   pwm=%4d  TRAI=%4lld  PHAI=%4lld\n", d, c1, c2);
    if (!f1 && c1 > 20) f1 = d;
    if (!f2 && c2 > 20) f2 = d;
    if (f1 && f2) break;
  }
  WL.setPWM(0); WR.setPWM(0);
  syncReq = true;
  Serial.printf(">> DEADBAND: TRAI=%d PHAI=%d (dang dung %d/%d)\n",
                f1, f2, WL.deadband, WR.deadband);
}

void checkPairing() {
  pidOn = false; WL.setPWM(0); WR.setPWM(0); delay(300);

  Serial.println(F(">> Chay banh TRAI (2 giay)..."));
  WL.clearCount(); WR.clearCount();
  WL.setPWM(400); delay(2000); WL.setPWM(0);
  Serial.printf("   TRAI=%6lld  PHAI=%6lld  <- TRAI phai LON va DUONG\n",
                (long long)WL.count(), (long long)WR.count());
  delay(1000);

  Serial.println(F(">> Chay banh PHAI (2 giay)..."));
  WL.clearCount(); WR.clearCount();
  WR.setPWM(400); delay(2000); WR.setPWM(0);
  Serial.printf("   TRAI=%6lld  PHAI=%6lld  <- PHAI phai LON va DUONG\n",
                (long long)WL.count(), (long long)WR.count());

  syncReq = true;
  Serial.println(F(">> Ca hai DUONG, dung cot, xe di TIEN -> OK."));
  Serial.println(F(">> Neu am hoac quay nguoc: sua M*_DIR / M*_ENC_DIR trong config.h"));
}

// dat toc do cho cac banh dang chon (dung cho phim +/-)
void applySelMps(float mps) {
  if (sel == 1 || sel == 3) WL.setTargetMps(mps);
  if (sel == 2 || sel == 3) WR.setTargetMps(mps);
  lastCmdMs = millis();
  timedOut  = false;
  if (!pidOn) enablePID(true);
}

void handleConsole(const String &line) {
  wdEnabled = false;              // dieu khien bang tay -> tat watchdog
  char c = line[0];
  float a[4];
  int n = splitFloats(line.c_str() + 1, a, 4);
  float v = (n >= 1) ? a[0] : 0;

  switch (c) {
    case '1': sel = 1; WR.setPWM(0); Serial.println(">> TRAI");    break;
    case '2': sel = 2; WL.setPWM(0); Serial.println(">> PHAI");    break;
    case '3': sel = 3;               Serial.println(">> CA HAI");  break;
    case 'c': checkPairing();  break;
    case 'f': findDeadband();  break;
    case 'g': enablePID(true);  plotOn = true;  Serial.println(">> PID ON");  break;
    case 'h': enablePID(false); plotOn = false; Serial.println(">> PID OFF"); break;
    case 'x': stopAll(); plotOn = false; Serial.println(">> DUNG");           break;
    case 'k': printParams(); break;
    case 'm': {                       // PWM tho, PID TAT -> do rpm toi da
      pidOn = false;
      int d = constrain((int)v, -PWM_MAX, PWM_MAX);
      if (sel == 1 || sel == 3) WL.setPWM(d);
      if (sel == 2 || sel == 3) WR.setPWM(d);
      plotOn = true;
      Serial.printf(">> PWM tho=%d (PID TAT). Cot 3 = rpm. Go 'x' de dung\n", d);
    } break;
    case 's': plotOn = !plotOn; Serial.printf(">> plot=%d\n", plotOn); break;
    case 'j': odomOn = !odomOn; Serial.printf(">> odom=%d\n", odomOn); break;
    case 'b':
      if (sel == 1 || sel == 3) WL.deadband = constrain((int)v, 0, 500);
      if (sel == 2 || sel == 3) WR.deadband = constrain((int)v, 0, 500);
      Serial.printf(">> deadband=%d\n", (int)v);
      break;
    case 't':
      if (sel == 1 || sel == 3) WL.setTargetRpm(v);
      if (sel == 2 || sel == 3) WR.setTargetRpm(v);
      lastCmdMs = millis(); timedOut = false; syncReq = true;
      Serial.printf(">> target=%.1f rpm (%.3f m/s)\n", v, v * RPM_TO_MPS);
      break;
    case 'v':
      if (sel == 1 || sel == 3) WL.setTargetMps(v);
      if (sel == 2 || sel == 3) WR.setTargetMps(v);
      lastCmdMs = millis(); timedOut = false; syncReq = true;
      Serial.printf(">> target=%.3f m/s (%.1f rpm)\n", v, v * MPS_TO_RPM);
      break;
    case 'w':
      if (n >= 2) { setCmdVel(a[0], a[1]);
                    Serial.printf(">> v=%.3f m/s  w=%.3f rad/s\n", a[0], a[1]); }
      break;
    case 'p': if (sel == 1 || sel == 3) WL.Kp = v;
              if (sel == 2 || sel == 3) WR.Kp = v;
              Serial.printf(">> Kp=%.3f\n", v); break;
    case 'i': if (sel == 1 || sel == 3) WL.Ki = v;
              if (sel == 2 || sel == 3) WR.Ki = v;
              Serial.printf(">> Ki=%.3f\n", v); break;
    case 'd': if (sel == 1 || sel == 3) WL.Kd = v;
              if (sel == 2 || sel == 3) WR.Kd = v;
              Serial.printf(">> Kd=%.4f\n", v); break;
    case 'o': {
      float sl = WL.target, sr = WR.target;
      Serial.println(">> STEP TEST");
      stopAll(); delay(800);
      enablePID(true); plotOn = true;
      if (sel == 1 || sel == 3) WL.target = sl;
      if (sel == 2 || sel == 3) WR.target = sr;
      lastCmdMs = millis(); timedOut = false;
    } break;
    // ---------------- phim don, khong can tham so ----------------
    case '+':
      curV += 0.05f; curW = 0;
      applySelMps(curV);
      Serial.printf(">> target=%.2f m/s (%.1f rpm)\n", curV, curV * MPS_TO_RPM);
      break;
    case '-':
      curV -= 0.05f; curW = 0;
      applySelMps(curV);
      Serial.printf(">> target=%.2f m/s (%.1f rpm)\n", curV, curV * MPS_TO_RPM);
      break;
    case '0':
      curV = curW = 0;
      applySelMps(0);
      Serial.println(">> target=0 (PID van giu, banh ham ve 0)");
      break;
    case '<':
      curV = 0; curW = 0.8f;
      setCmdVel(curV, curW);
      Serial.println(">> QUAY TRAI 0.8 rad/s: TRAI lui, PHAI tien");
      break;
    case '>':
      curV = 0; curW = -0.8f;
      setCmdVel(curV, curW);
      Serial.println(">> QUAY PHAI 0.8 rad/s: TRAI tien, PHAI lui");
      break;
    case 'q':
    case 'Q': {
      pidOn = false;
      int d = (c == 'q') ? PWM_MAX : -PWM_MAX;
      if (sel == 1 || sel == 3) WL.setPWM(d);
      if (sel == 2 || sel == 3) WR.setPWM(d);
      plotOn = true;
      Serial.printf(">> PWM tho=%d (PID TAT). Doi 3s, doc COT 3 = rpm. 'x' de dung\n", d);
    } break;
    case 'e':
      echoOn = !echoOn;
      Serial.printf(">> echo=%d (in lai byte nhan duoc de chan doan terminal)\n", echoOn);
      break;
    case '?': showHelp(); break;
    default: break;
  }
}

// Doc console USB: phim don chay ngay, lenh co tham so cho Enter
// hoac cho 300ms khong go them.
void consolePoll() {
  static String buf;
  static uint32_t lastCh = 0;

  while (Serial.available()) {
    char ch = Serial.read();
    lastCh = millis();
    if (echoOn) Serial.printf("[%02X]", (uint8_t)ch);

    if (ch == '\n' || ch == '\r') {
      if (buf.length()) { String l = buf; buf = ""; handleConsole(l); }
      continue;
    }
    if (buf.length() == 0 && ch != 0 && strchr(NOARG, ch)) {   // phim don -> chay ngay
      String l; l += ch;
      handleConsole(l);
      continue;
    }
    buf += ch;
    if (buf.length() > 60) buf = "";
  }

  // terminal khong gui Enter: tu chot dong sau 300ms im lang
  if (buf.length() && (millis() - lastCh) > 300) {
    String l = buf; buf = "";
    handleConsole(l);
  }
}

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(300);
  Serial.println(F("\n=== ESP32-S3 BASE CONTROLLER ==="));

  // phai dat TRUOC attachFullQuad()
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  WL.begin();
  WR.begin();

  Link.begin(UART1_BAUD, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  Serial.printf("UART1: TX=%d RX=%d @ %d\n", UART1_TX_PIN, UART1_RX_PIN, UART1_BAUD);
  Serial.printf("Banh D=%.0fmm -> 1 m/s = %.1f rpm | vmax=%.2f m/s\n",
                WHEEL_D * 1000, MPS_TO_RPM, MAX_MPS);
  Serial.printf("m/tick=%.8f  wheel_base=%.3fm  CPR=%.1f (x4)\n",
                M_PER_TICK, WHEEL_BASE, CPR);
  showHelp();

  lastCmdMs = millis();
  tStart    = millis();
  if (AUTO_PID_ON_BOOT) { syncReq = true; pidOn = true; }
  if (ODOM_ON_BOOT) {
  Link.printf("# BOOT pid=%d db=%d/%d maxrpm=%.0f\n",
              AUTO_PID_ON_BOOT ? 1 : 0, M1_DEADBAND, M2_DEADBAND, MAX_RPM);
  }            
}

// ---------------------------------------------------------------- loop
void loop() {
  static uint32_t tCtrl = 0, tOdom = 0;
  static String bufLink;
  String line;

  uint32_t now = millis();

  // ---------- vong dieu khien 50 Hz ----------
  int32_t dt = (int32_t)(now - tCtrl);
  if (dt >= (int32_t)DT_MS) {
    if (dt > 100 || syncReq) {              // dong bo lai sau lenh chan lau
      tCtrl = now;
      WL.syncEncoder(); WR.syncEncoder();
      WL.resetState();  WR.resetState();
      syncReq = false;
      tStart  = now;
    } else {
      tCtrl += DT_MS;

      // watchdog lenh: mat ket noi Jetson -> ha target ve 0
      if (pidOn && wdEnabled && !timedOut && (now - lastCmdMs) > CMD_TIMEOUT_MS) {
        WL.target = WR.target = 0;
        timedOut = true;
        Serial.println(">> CMD TIMEOUT -> dung banh");
      }

      WL.update(pidOn);
      WR.update(pidOn);

      // in do thi tuning: t | target | rpm | pwm
      if (plotOn) {
        float t = (now - tStart) / 1000.0f;
        Wheel &w = (sel == 2) ? WR : WL;
        Serial.printf("%.2f %.0f %.1f %d\n", t, w.target, w.meas, w.out);
      }
    }
  }

  // ---------- phan hoi cho Jetson ----------
  if (odomOn && linkReady() && (now - tOdom) >= ODOM_PERIOD_MS) {
    tOdom = now;
    Link.printf("O %lu %.3f %.3f %lld %lld\n",
                (unsigned long)now, WL.mps(), WR.mps(),
                (long long)WL.count(), (long long)WR.count());
  }

  // ---------- doc lenh ----------
  if (linkReady()) {
    if (readLine(Link, bufLink, line)) handleJetson(line);
  } else {
    while (Link.available()) Link.read();   // do bo log boot cua Jetson
    bufLink = "";
  }
  consolePoll();
}