#pragma once
#include <Arduino.h>

// =====================================================================
//  CHAN PHAN CUNG
// =====================================================================

// ---------- Dong co 1  (quy uoc: BANH TRAI) ----------
#define M1_EN_L   39
#define M1_EN_R   38
#define M1_PWM_L  48   // Lui
#define M1_PWM_R  47    // Tien
#define M1_ENC_A  6
#define M1_ENC_B  7

// ---------- Dong co 2  (quy uoc: BANH PHAI) ----------
#define M2_EN_L   42
#define M2_EN_R   41
#define M2_PWM_L  2    // Lui
#define M2_PWM_R  1    // Tien
#define M2_ENC_A  5
#define M2_ENC_B  4

// ---------- Dao dau (chinh sau khi test bang lenh 'c') ----------
// _DIR      : +1 / -1  -> doi chieu quay cua dong co
// _ENC_DIR  : +1 / -1  -> doi dau dem encoder
// Yeu cau: target duong  =>  banh quay TIEN  =>  encoder dem TANG
#define M1_DIR      (+1)
#define M1_ENC_DIR  (-1)
#define M2_DIR      (-1)   // <-- doi thanh (-1) neu banh phai lap doi xung
#define M2_ENC_DIR  (+1)   // <-- doi kem theo _DIR

// ---------- UART1 -> Jetson (lay tu build_flags) ----------
#ifndef UART1_TX_PIN
#define UART1_TX_PIN 13
#endif
#ifndef UART1_RX_PIN
#define UART1_RX_PIN 12
#endif
#ifndef UART1_BAUD
#define UART1_BAUD 115200
#endif

// =====================================================================
//  PWM
// =====================================================================
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES  = 10;
constexpr int PWM_MAX  = 1023;

// =====================================================================
//  ENCODER / VONG DIEU KHIEN
// =====================================================================
constexpr float    CPR    = 998.4f;   // xung/vong TRUC RA (da qua hop so, x4)
constexpr uint32_t DT_MS  = 20;       // 50 Hz
constexpr float    DT     = DT_MS / 1000.0f;
constexpr float    K_RPM  = 60.0f / (CPR * DT);   // xung/chu ky -> rpm
constexpr float    ALPHA  = 0.3f;                 // loc thong thap toc do

// =====================================================================
//  CO KHI  -> quy doi m/s <-> rpm
// =====================================================================
constexpr float WHEEL_D    = 0.095f;                 // duong kinh banh (m)
constexpr float WHEEL_C    = 3.14159265f * WHEEL_D;  // chu vi = 0.29845 m
constexpr float MPS_TO_RPM = 60.0f / WHEEL_C;        // 1 m/s = 201.04 rpm
constexpr float RPM_TO_MPS = 1.0f / MPS_TO_RPM;
constexpr float M_PER_TICK = WHEEL_C / CPR;          // 0.00029893 m/xung

// Khoang cach tam 2 banh chu dong (m)
constexpr float WHEEL_BASE = 0.389f;   // do duoc 39.0 - 39.1 cm

// !!! CAN XAC NHAN: toc do toi da dong co (rpm truc ra khi khong tai)
constexpr float MAX_RPM = 260.0f;
constexpr float MAX_MPS = MAX_RPM * RPM_TO_MPS;      // ~0.995 m/s

// =====================================================================
//  PID da tuning
// =====================================================================
constexpr float M1_KP = 5.6f, M1_KI = 35.0f, M1_KD = 0.0f;
constexpr float M2_KP = 7.5f, M2_KI = 25.0f, M2_KD = 0.0f;

constexpr int M1_DEADBAND = 150;   // do bang lenh 'f' (banh TRAI)
constexpr int M2_DEADBAND = 160;   // do bang lenh 'f' (banh PHAI)

// =====================================================================
//  GIAO TIEP
// =====================================================================
constexpr bool AUTO_PID_ON_BOOT = true;
constexpr bool ODOM_ON_BOOT = false;   // im lang cho toi khi Jetson gui lenh dau tien
constexpr uint32_t BOOT_QUIET_MS = 0000;  // cam tuyet doi tren UART1 trong 30s dau
constexpr uint32_t CMD_TIMEOUT_MS = 500;   // mat lenh > 500ms -> dung banh
constexpr uint32_t ODOM_PERIOD_MS = 20;    // gui phan hoi 50 Hz