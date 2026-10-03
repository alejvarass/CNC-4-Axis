/*
 * ============================================================================
 *  CNC XYZW  -  Firmware V2.0.0
 *  ESP32 (Arduino-ESP32 core 3.x)  ·  4 ejes (X, Y, Z, W)  ·  TCP 5000 + USB
 * ============================================================================
 *  Forma un sistema completo junto con:
 *    - CNC_V2.py       (GUI de operación, PySide6, por WiFi/TCP)
 *    - CNC_MANT_V2.py  (herramienta de mantenimiento/aprovisionamiento, USB)
 *
 *  Resumen de lo que corrige / optimiza respecto a V1 (ver documentos en docs/):
 *    - Protocolo con ACK/NACK explícito para TODO comando (nada se ignora en
 *      silencio), líneas largas rechazadas (nunca se ejecuta una línea truncada).
 *    - Cola de bloques de movimiento con contrapresión (NACK|full), identificador
 *      de bloque y "último bloque terminado" en telemetría (secuenciador exacto).
 *    - Planificador real: velocidades en mm/s, aceleración en mm/s^2, perfil
 *      trapezoidal o curva S, velocidad vectorial en movimientos multieje.
 *    - Finales de carrera dependientes de la dirección + ALARM latcheado; sin
 *      referenciado (HOME) no hay movimientos; "Set Zero" ya NO marca HOME.
 *    - E-STOP inmediato (corta ENABLE antes de procesar nada más), watchdog de
 *      enlace (por defecto 250 ms) y dead-man de jog; E-STOP físico opcional.
 *    - Seguridad: sin credenciales en el código, autenticación HMAC-SHA256 por
 *      desafío (token en NVS, solo escritura), contraseña WiFi nunca se imprime.
 *    - Sin String compartido entre tareas, sin mutex "decorativo": la tarea de
 *      motores es la única que escribe el estado de movimiento.
 *    - WiFi no bloqueante con reconexión; el servidor TCP siempre arranca.
 *    - Pulsos STEP por registro GPIO (mismo ancho para todos los ejes),
 *      temporización con plazo absoluto (sin deriva), NVS solo con el equipo
 *      detenido y solo lo que cambió.
 *
 *  Convenciones de coordenadas
 *    - "Máquina": pasos desde el cero de HOME (cero = sensor + soft offset).
 *    - "Trabajo": máquina - offset de trabajo (Set Zero). Es lo que ve la GUI.
 *    - El final de carrera está en el extremo NEGATIVO de cada eje. Hacia el
 *      extremo positivo no hay sensor: lo protege el límite suave (max_travel)
 *      y debe existir un tope mecánico.
 *
 *  IMPORTANTE (hardware): ver docs/ (Manual y Transferencia de ingeniería):
 *    - Pull-up externo en GPIO34 (X) y en la línea ENABLE (GPIO27).
 *    - Los finales de carrera deben ser NC a GND (activo = nivel ALTO).
 *    - Cableado de E-STOP físico: NC en serie con el enable de los drivers.
 * ============================================================================
 */

#include <WiFi.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include "lwip/sockets.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "mbedtls/md.h"

// ============================================================================
//  1. CONFIGURACIÓN DE COMPILACIÓN
// ============================================================================
#define FW_VERSION      "2.0.0"
#define PROTO_VERSION   2

static const uint16_t SERVER_PORT = 5000;
static const size_t   MAX_LINE_LEN = 256;

// --- Pines (idénticos a V1) -------------------------------------------------
enum AxisId : uint8_t { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2, AXIS_W = 3, AXIS_COUNT = 4 };
static const char AXIS_CHAR[AXIS_COUNT] = {'x', 'y', 'z', 'w'};

static const uint8_t PIN_ENABLE_ACTUATORS = 27;          // LOW = drivers habilitados
static constexpr uint8_t PIN_STEP[AXIS_COUNT]  = {23, 21, 16, 13};
static const uint8_t PIN_DIR[AXIS_COUNT]   = {22, 17, 4, 15};
static const uint8_t PIN_LIMIT[AXIS_COUNT] = {34, 25, 14, 18};
static const uint8_t LIMIT_ACTIVE_LEVEL = HIGH;          // NC a GND: abierto/roto = ALTO = activo

// E-STOP físico (opcional). -1 = no instalado. Recomendado GPIO33, contacto NC a GND
// con pull-up interno: seta pulsada o cable roto = nivel ALTO = alarma.
#define PIN_ESTOP_INPUT   -1
static const uint8_t ESTOP_ACTIVE_LEVEL = HIGH;

// Los pines STEP deben estar en el banco GPIO 0..31 (escritura por registro W1TS/W1TC).
static_assert(PIN_STEP[0] < 32 && PIN_STEP[1] < 32 && PIN_STEP[2] < 32 && PIN_STEP[3] < 32,
              "Los pines STEP deben ser GPIO < 32");

// --- Temporización de movimiento -------------------------------------------
static const uint32_t MIN_STEP_US      = 40;             // 25 kHz máx. por eje dominante
static const float    MAX_SPS          = 1000000.0f / (float)MIN_STEP_US;
static const float    MIN_START_SPS    = 50.0f;          // velocidad mínima de arranque/parada
static const uint32_t STEP_PULSE_US    = 4;              // ancho de pulso STEP (todos los ejes)
static const uint32_t DIR_SETUP_US     = 20;             // DIR estable antes del primer STEP
static const uint32_t ENABLE_SETTLE_MS = 5;              // espera tras habilitar drivers
static const float    S_RAMP_FACTOR    = 2.1f;           // distancia de rampa S vs. trapezoidal (pico de aceleración <= accel)
static const float    SOFT_LIMIT_TOL_MM = 0.05f;
static const uint32_t SETTLE_MS_HOMING = 120;            // reposo entre fases de HOMING

// --- Seguridad / enlace -----------------------------------------------------
static const uint32_t JOG_DEADMAN_MS     = 250;          // sin manual_ping => parada de jog
static const uint32_t DEFAULT_LINK_WD_MS = 250;          // sin tráfico válido con movimiento => ALARM
static const uint32_t TELEMETRY_MS       = 50;
static const uint32_t STALE_CLIENT_MS    = 3000;         // cliente sin tráfico: otro puede tomar su lugar
static const uint8_t  AUTH_MAX_FAILS     = 3;
static const uint32_t AUTH_LOCKOUT_MS    = 30000;
static const uint8_t  TOKEN_MIN_LEN      = 8;
static const uint8_t  TOKEN_MAX_LEN      = 32;

// --- Cola de bloques ---------------------------------------------------------
static const uint8_t  QCAP = 16;

// ============================================================================
//  2. TIPOS
// ============================================================================
enum MachineState : uint8_t { MS_IDLE = 0, MS_HOMING, MS_RUNNING, MS_MANUAL, MS_ALARM };
static const char* const MS_NAME[] = {"idle", "homing", "running", "manual", "alarm"};

enum AlarmCode : uint8_t {
  AL_NONE = 0,
  AL_ESTOP = 1,        // E-STOP desde la GUI/red
  AL_HARD_LIMIT = 2,   // final de carrera activado hacia el que se movía el eje
  AL_COMM_LOST = 3,    // watchdog de enlace / cliente desconectado en movimiento
  AL_HOME_FAIL = 4,    // fallo de la secuencia de HOMING
  AL_PHYS_ESTOP = 5,   // E-STOP físico
  AL_INTERNAL = 6      // fallo interno (temporizador)
};
static const char* const AL_TEXT[] = {"none", "estop", "hard_limit", "comm_lost", "home_fail", "phys_estop", "internal"};

enum AxisErr : uint8_t {
  AE_NONE = 0, AE_SOFT_LIMIT, AE_SWITCH_HIT, AE_NOT_HOMED, AE_SEEK_FAIL, AE_BACKOFF_FAIL,
  AE_REAPPROACH_FAIL, AE_DEADMAN, AE_DISABLED, AE_STOPPED, AE_ESTOP, AE_COMM
};

enum Rc : uint8_t {
  RC_OK = 0, RC_BAD_ARGS, RC_BAD_AXIS, RC_NOT_HOMED, RC_SOFT_LIMIT, RC_BUSY, RC_FULL, RC_ALARM,
  RC_DISABLED, RC_AUTH, RC_UNKNOWN, RC_DEPRECATED, RC_TOO_LONG, RC_NOT_IDLE
};
static const char* const RC_TEXT[] = {
  "ok", "bad_args", "bad_axis", "not_homed", "soft_limit", "busy", "full", "alarm",
  "disabled", "auth_required", "unknown_command", "deprecated", "line_too_long", "busy"
};

enum MotionMode : uint8_t { MM_NONE = 0, MM_MOVE, MM_HOME, MM_JOG };
enum BlockType : uint8_t { BT_MOVE = 0, BT_HOME, BT_DWELL };
enum TokenState : uint8_t { TOK_UNSET = 0, TOK_OPEN = 1, TOK_SET = 2 };

// Configuración persistente por eje (solo la modifica la tarea loop(), con el equipo en reposo)
struct AxisCfg {
  float    stepsPerMm;
  float    maxTravelMm;
  uint32_t backoffSteps;
  uint32_t softOffsetSteps;
  int32_t  seekUs, feedUs, homeBoUs, manualUs, jogUs;
  float    startMmS, cruiseMmS, endMmS, accelMmS2;
  uint8_t  useSCurve;
  uint8_t  dirFwdLevel;
  uint8_t  calibrated;
  char     lastCal[24];
};

// Estado de ejecución por eje (la tarea de motores es la única que escribe 'steps')
struct AxisRt {
  volatile int32_t steps;      // posición de máquina (pasos)
  volatile int32_t woff;       // offset de trabajo (pasos)
  volatile int32_t target;     // destino del bloque en curso (pasos de máquina)
  volatile uint8_t homed;
  volatile uint8_t moving;
  volatile uint8_t moveDir;    // 0 ninguno, 1 adelante (+), 2 atrás (-)
  volatile uint8_t err;
  volatile uint8_t limit;      // último estado muestreado del final de carrera
};

// Bloque de la cola de movimiento
struct Block {
  uint8_t  type;
  uint8_t  homeAxis;
  uint8_t  dirMask;            // bit a = 1 -> sentido positivo
  uint8_t  sCurve;
  uint32_t id;
  uint32_t steps[AXIS_COUNT];
  uint32_t nSteps;             // pasos del eje dominante
  float    vStart, vCruise, vEnd, accel;     // pasos/s del eje dominante y pasos/s^2
  uint32_t backoffSteps, softOffsetSteps, maxSearchSteps;   // HOME
  int32_t  seekUs, feedUs, homeBoUs;                        // HOME
  uint32_t dwellMs;                                         // DWELL
};

struct Profile {
  uint32_t n, nAcc, nDec;
  float    vs, vc, ve, a;
  bool     s;
};

struct PlanSnap {
  int32_t  pos[AXIS_COUNT];
  bool     homed[AXIS_COUNT];
  uint32_t epoch;
};

// ============================================================================
//  3. VARIABLES GLOBALES
// ============================================================================
static AxisCfg cfg[AXIS_COUNT];
static AxisCfg cfgSaved[AXIS_COUNT];
static AxisRt  rt[AXIS_COUNT];

static Preferences prefs;
static WiFiServer  server(SERVER_PORT);
static WiFiClient  cl;

static char     wifiSsid[33]    = "";
static char     wifiPass[65]    = "";
static char     wifiDevName[33] = "CNC-XYZW-NETLOG";
static uint8_t  ipOct4          = 167;       // 192.168.1.<ipOct4>; 0 = DHCP
static char     token[TOKEN_MAX_LEN + 1] = "";
static uint8_t  tokenState      = TOK_UNSET;
static uint32_t linkWdMs        = DEFAULT_LINK_WD_MS;   // 0 = desactivado (no recomendado)

static uint32_t stepMask[AXIS_COUNT];
static hw_timer_t* stepTimer = nullptr;
static TaskHandle_t taskMotorsHandle = nullptr;

// Estado compartido entre tareas (escritura: loop() salvo indicación; lectura: ambas)
static volatile bool     actuatorsEnabled = false;
static volatile bool     hardStopReq = false;    // detener YA (E-STOP, límite, drivers off)
static volatile bool     softStopReq = false;    // detener con rampa y vaciar la cola
static volatile uint8_t  alarmCode = AL_NONE;
static volatile int8_t   alarmAxis = -1;
static volatile bool     alarmEventPending = false;
static volatile bool     forceStatusPush = false;
static volatile uint8_t  motionMode = MM_NONE;
static volatile bool     blockActive = false;
static volatile uint32_t doneId = 0;
static volatile uint32_t lastLinkMs = 0;         // último tráfico válido de una sesión autorizada
static volatile bool     sessionAuthorized = false;
static volatile uint32_t enabledAtMs = 0;

// Cola de bloques (productor: loop(); consumidor: tarea de motores)
static Block    qbuf[QCAP];
static volatile uint8_t  qHead = 0, qTail = 0, qCount = 0;
static int32_t  planned[AXIS_COUNT];             // posición al terminar toda la cola
static bool     plannedHomed[AXIS_COUNT];
static volatile uint32_t qEpoch = 0;             // cambia con cada vaciado de cola
static portMUX_TYPE qMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE sysMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t autoId = 0x80000000UL;

// Jog (solicitud loop() -> tarea de motores)
static volatile uint8_t  jogPending = 0;
static volatile uint8_t  jogAxisReq = 0;
static volatile uint8_t  jogFwdReq = 1;
static volatile uint8_t  jogStopReq = 0;
static volatile uint32_t jogLastPing = 0;
static volatile uint8_t  jogAxisRun = 0xFF;
static volatile uint8_t  jogFwdRun = 1;

// Sesión de red
struct Session {
  bool     active;
  bool     authed;
  bool     netlog;          // ya habló el protocolo NET-LOG (recibe telemetría push)
  bool     grbl;            // ya habló dialecto GRBL
  uint32_t lastRxMs;
  uint32_t lastTelemetryMs;
  uint16_t rxLen;
  bool     rxOverflow;
  uint8_t  authFails;
  char     rx[MAX_LINE_LEN];
  char     nonce[33];
};
static Session ses;
static uint32_t authLockUntilMs = 0;
static bool     authLocked = false;
static uint8_t  authFailCount = 0;

// Estado modal del dialecto GRBL (por sesión)
static bool     grblRel = false;             // G91
static bool     grblMotionG1 = false;        // G1 (si no, G0)
static float    grblFeedMmMin = 0.0f;        // F
static bool     grblHold = false;            // línea GRBL retenida por cola llena (se reintenta)

static char     serialBuf[MAX_LINE_LEN];
static uint16_t serialLen = 0;
static bool     serialOverflow = false;

static bool     cfgDirty = false;
static uint32_t cfgDirtyAtMs = 0;

static bool     serverStarted = false;
static wl_status_t lastWifiStatus = WL_NO_SHIELD;
static uint32_t lastWifiAttemptMs = 0;

// ============================================================================
//  4. UTILIDADES
// ============================================================================
static inline int64_t nowUs() { return esp_timer_get_time(); }
static inline uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline long clampl(long v, long lo, long hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Escritor acotado: si no cabe, marca desbordamiento y NO se envía la línea.
struct OutBuf {
  char*  p;
  size_t cap;
  size_t len;
  bool   ovf;
  OutBuf(char* b, size_t c) : p(b), cap(c), len(0), ovf(false) { if (c) p[0] = 0; }
  void add(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};

void OutBuf::add(const char* fmt, ...) {
  if (ovf) return;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(p + len, cap - len, fmt, ap);
  va_end(ap);
  if (n < 0 || (size_t)n >= cap - len) {
    ovf = true;
    p[len] = 0;
  } else {
    len += (size_t)n;
  }
}

static void toHex(const uint8_t* in, size_t n, char* out) {
  static const char H[] = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) {
    out[2 * i]     = H[in[i] >> 4];
    out[2 * i + 1] = H[in[i] & 15];
  }
  out[2 * n] = 0;
}

static bool ctEqual(const char* a, const char* b, size_t n) {
  uint8_t d = 0;
  for (size_t i = 0; i < n; i++) d |= (uint8_t)(a[i] ^ b[i]);
  return d == 0;
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Decodifica %XX en el propio buffer (los campos con secretos/SSID viajan codificados).
static void urlDecodeInPlace(char* s) {
  char* o = s;
  while (*s) {
    if (*s == '%' && hexVal(s[1]) >= 0 && hexVal(s[2]) >= 0) {
      *o++ = (char)(hexVal(s[1]) * 16 + hexVal(s[2]));
      s += 3;
    } else {
      *o++ = *s++;
    }
  }
  *o = 0;
}

static void urlEncodeTo(Print& out, const char* s) {
  static const char H[] = "0123456789ABCDEF";
  for (; *s; s++) {
    uint8_t c = (uint8_t)*s;
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out.write(c);
    } else {
      out.write('%');
      out.write(H[c >> 4]);
      out.write(H[c & 15]);
    }
  }
}

static void jsonEscapeTo(Print& out, const char* s) {
  for (; *s; s++) {
    uint8_t c = (uint8_t)*s;
    if (c == '"' || c == '\\') { out.write('\\'); out.write(c); }
    else if (c < 0x20) { out.write('?'); }
    else out.write(c);
  }
}

// Copia segura: solo ASCII imprimible sin separadores del protocolo.
static void sanitizeText(char* dst, size_t cap, const char* src) {
  size_t i = 0;
  for (; src && src[i] && i + 1 < cap; i++) {
    char c = src[i];
    dst[i] = (c < 0x20 || c > 0x7E || c == '|' || c == ',' || c == '"' || c == '\\') ? '_' : c;
  }
  dst[i] = 0;
}

static bool parseFloatStrict(const char* s, float& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  double v = strtod(s, &end);
  if (end == s || *end != '\0' || !isfinite(v)) return false;
  out = (float)v;
  return true;
}

static bool parseLongStrict(const char* s, long& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;
  out = v;
  return true;
}

static bool parseAxisTok(const char* s, uint8_t& out) {
  if (!s || !s[0] || s[1]) return false;
  switch (tolower((unsigned char)s[0])) {
    case 'x': out = AXIS_X; return true;
    case 'y': out = AXIS_Y; return true;
    case 'z': out = AXIS_Z; return true;
    case 'w': out = AXIS_W; return true;
    case 'a': out = AXIS_W; return true;          // 4º eje en dialecto GRBL
    default: return false;
  }
}

// Dirección: positiva/negativa. Devuelve false si el texto no es reconocido.
static bool parseDirTok(const char* s, bool& fwd) {
  if (!s) return false;
  if (!strcmp(s, "forward") || !strcmp(s, "fwd") || !strcmp(s, "up") || !strcmp(s, "right") ||
      !strcmp(s, "+") || !strcmp(s, "1")) { fwd = true; return true; }
  if (!strcmp(s, "backward") || !strcmp(s, "back") || !strcmp(s, "down") || !strcmp(s, "left") ||
      !strcmp(s, "-") || !strcmp(s, "0") || !strcmp(s, "-1")) { fwd = false; return true; }
  return false;
}

// Divide 'line' en campos separados por '|' (in situ). Conserva campos vacíos.
static int splitFields(char* line, char* argv[], int maxArgs) {
  int argc = 0;
  char* p = line;
  if (maxArgs <= 0) return 0;
  argv[argc++] = p;
  for (; *p; p++) {
    if (*p == '|') {
      *p = 0;
      if (argc >= maxArgs) return -1;           // demasiados campos
      argv[argc++] = p + 1;
    }
  }
  return argc;
}

// ============================================================================
//  5. CONFIGURACIÓN PERSISTENTE (NVS)
//     - Solo se escribe con el equipo detenido y solo lo que cambió.
//     - Los secretos (contraseña WiFi, token) son de SOLO ESCRITURA: ningún
//       comando los devuelve ni los imprime.
// ============================================================================
static void axisDefaults(AxisCfg& c) {
  memset(&c, 0, sizeof(c));
  c.stepsPerMm = 568.0f;
  c.maxTravelMm = 110.0f;
  c.backoffSteps = 1136;
  c.softOffsetSteps = 2840;
  c.seekUs = 1200;
  c.feedUs = 2800;
  c.homeBoUs = 1500;
  c.manualUs = 1000;
  c.jogUs = 1000;
  c.startMmS = 1.5f;
  c.cruiseMmS = 15.0f;
  c.endMmS = 1.5f;
  c.accelMmS2 = 50.0f;
  c.useSCurve = 1;
  c.dirFwdLevel = HIGH;
  c.calibrated = 0;
  strcpy(c.lastCal, "Sin datos");
}

static float finiteOr(float v, float def) { return isfinite(v) ? v : def; }

// Fuerza valores dentro de rangos seguros (protege contra NVS corrupta o de otra versión).
static void axisSanitize(AxisCfg& c) {
  AxisCfg d;
  axisDefaults(d);
  c.stepsPerMm  = clampf(finiteOr(c.stepsPerMm, d.stepsPerMm), 1.0f, 100000.0f);
  c.maxTravelMm = clampf(finiteOr(c.maxTravelMm, d.maxTravelMm), 1.0f, 2000.0f);
  if (c.backoffSteps > 1000000UL) c.backoffSteps = 1000000UL;
  if (c.softOffsetSteps > 1000000UL) c.softOffsetSteps = 1000000UL;
  c.seekUs    = (int32_t)clampl(c.seekUs, MIN_STEP_US, 200000);
  c.feedUs    = (int32_t)clampl(c.feedUs, MIN_STEP_US, 200000);
  c.homeBoUs  = (int32_t)clampl(c.homeBoUs, MIN_STEP_US, 200000);
  c.manualUs  = (int32_t)clampl(c.manualUs, MIN_STEP_US, 200000);
  c.jogUs     = (int32_t)clampl(c.jogUs, MIN_STEP_US, 200000);
  c.startMmS  = clampf(finiteOr(c.startMmS, d.startMmS), 0.05f, 500.0f);
  c.cruiseMmS = clampf(finiteOr(c.cruiseMmS, d.cruiseMmS), 0.1f, 1000.0f);
  c.endMmS    = clampf(finiteOr(c.endMmS, d.endMmS), 0.05f, 500.0f);
  if (c.cruiseMmS < c.startMmS) c.cruiseMmS = c.startMmS;
  c.accelMmS2 = clampf(finiteOr(c.accelMmS2, d.accelMmS2), 1.0f, 20000.0f);
  c.useSCurve = c.useSCurve ? 1 : 0;
  c.dirFwdLevel = c.dirFwdLevel ? HIGH : LOW;
  c.calibrated = c.calibrated ? 1 : 0;
  c.lastCal[sizeof(c.lastCal) - 1] = 0;
  char tmp[sizeof(c.lastCal)];
  sanitizeText(tmp, sizeof(tmp), c.lastCal);
  memcpy(c.lastCal, tmp, sizeof(tmp));
}

static void axisKey(char* out, size_t cap, uint8_t a, const char* suffix) {
  snprintf(out, cap, "%c_%s", AXIS_CHAR[a], suffix);
}

static void loadConfig() {
  for (uint8_t a = 0; a < AXIS_COUNT; a++) axisDefaults(cfg[a]);
  if (prefs.begin("cnc_xyzw", false)) {
    char k[16];
    for (uint8_t a = 0; a < AXIS_COUNT; a++) {
      AxisCfg& c = cfg[a];
      axisKey(k, sizeof(k), a, "dfl");    c.dirFwdLevel = prefs.getUChar(k, c.dirFwdLevel);
      axisKey(k, sizeof(k), a, "spm");    c.stepsPerMm = prefs.getFloat(k, c.stepsPerMm);
      axisKey(k, sizeof(k), a, "max");    c.maxTravelMm = prefs.getFloat(k, c.maxTravelMm);
      axisKey(k, sizeof(k), a, "cal");    c.calibrated = prefs.getBool(k, false);
      axisKey(k, sizeof(k), a, "lcal");   prefs.getString(k, c.lastCal, sizeof(c.lastCal));
      axisKey(k, sizeof(k), a, "bo_st");  c.backoffSteps = prefs.getUInt(k, c.backoffSteps);
      axisKey(k, sizeof(k), a, "so_st");  c.softOffsetSteps = prefs.getUInt(k, c.softOffsetSteps);
      axisKey(k, sizeof(k), a, "uscurve"); c.useSCurve = prefs.getBool(k, true);
      axisKey(k, sizeof(k), a, "hseek");  c.seekUs = prefs.getInt(k, c.seekUs);
      axisKey(k, sizeof(k), a, "hfeed");  c.feedUs = prefs.getInt(k, c.feedUs);
      axisKey(k, sizeof(k), a, "hbo");    c.homeBoUs = prefs.getInt(k, c.homeBoUs);
      axisKey(k, sizeof(k), a, "man");    c.manualUs = prefs.getInt(k, c.manualUs);
      axisKey(k, sizeof(k), a, "jog");    c.jogUs = prefs.getInt(k, c.jogUs);
      axisKey(k, sizeof(k), a, "sc_s");   c.startMmS = prefs.getFloat(k, c.startMmS);
      axisKey(k, sizeof(k), a, "sc_c");   c.cruiseMmS = prefs.getFloat(k, c.cruiseMmS);
      axisKey(k, sizeof(k), a, "sc_e");   c.endMmS = prefs.getFloat(k, c.endMmS);
      axisKey(k, sizeof(k), a, "acc");    c.accelMmS2 = prefs.getFloat(k, c.accelMmS2);
      axisSanitize(c);
      cfgSaved[a] = c;
    }
    prefs.getString("w_ssid", wifiSsid, sizeof(wifiSsid));
    prefs.getString("w_pass", wifiPass, sizeof(wifiPass));
    prefs.getString("w_devname", wifiDevName, sizeof(wifiDevName));
    ipOct4 = prefs.getUChar("ip_o4", ipOct4);
    linkWdMs = prefs.getUInt("wd_ms", DEFAULT_LINK_WD_MS);
    if (linkWdMs != 0 && (linkWdMs < 100 || linkWdMs > 5000)) linkWdMs = DEFAULT_LINK_WD_MS;
    if (prefs.isKey("tok")) {
      prefs.getString("tok", token, sizeof(token));
      tokenState = token[0] ? TOK_SET : TOK_OPEN;
    } else {
      tokenState = TOK_UNSET;
    }
    prefs.end();
  } else {
    for (uint8_t a = 0; a < AXIS_COUNT; a++) cfgSaved[a] = cfg[a];
  }
}

// Escribe un eje completo (solo se llama si cambió).
static void saveAxisNvs(uint8_t a) {
  const AxisCfg& c = cfg[a];
  char k[16];
  axisKey(k, sizeof(k), a, "dfl");    prefs.putUChar(k, c.dirFwdLevel);
  axisKey(k, sizeof(k), a, "spm");    prefs.putFloat(k, c.stepsPerMm);
  axisKey(k, sizeof(k), a, "max");    prefs.putFloat(k, c.maxTravelMm);
  axisKey(k, sizeof(k), a, "cal");    prefs.putBool(k, c.calibrated);
  axisKey(k, sizeof(k), a, "lcal");   prefs.putString(k, c.lastCal);
  axisKey(k, sizeof(k), a, "bo_st");  prefs.putUInt(k, c.backoffSteps);
  axisKey(k, sizeof(k), a, "so_st");  prefs.putUInt(k, c.softOffsetSteps);
  axisKey(k, sizeof(k), a, "uscurve"); prefs.putBool(k, c.useSCurve);
  axisKey(k, sizeof(k), a, "hseek");  prefs.putInt(k, c.seekUs);
  axisKey(k, sizeof(k), a, "hfeed");  prefs.putInt(k, c.feedUs);
  axisKey(k, sizeof(k), a, "hbo");    prefs.putInt(k, c.homeBoUs);
  axisKey(k, sizeof(k), a, "man");    prefs.putInt(k, c.manualUs);
  axisKey(k, sizeof(k), a, "jog");    prefs.putInt(k, c.jogUs);
  axisKey(k, sizeof(k), a, "sc_s");   prefs.putFloat(k, c.startMmS);
  axisKey(k, sizeof(k), a, "sc_c");   prefs.putFloat(k, c.cruiseMmS);
  axisKey(k, sizeof(k), a, "sc_e");   prefs.putFloat(k, c.endMmS);
  axisKey(k, sizeof(k), a, "acc");    prefs.putFloat(k, c.accelMmS2);
}

// Persiste los ejes modificados. Debe llamarse solo con el equipo en reposo.
static void saveConfigIfDirty() {
  bool any = false;
  for (uint8_t a = 0; a < AXIS_COUNT; a++) if (memcmp(&cfg[a], &cfgSaved[a], sizeof(AxisCfg)) != 0) any = true;
  if (any && prefs.begin("cnc_xyzw", false)) {
    for (uint8_t a = 0; a < AXIS_COUNT; a++) {
      if (memcmp(&cfg[a], &cfgSaved[a], sizeof(AxisCfg)) != 0) {
        saveAxisNvs(a);
        cfgSaved[a] = cfg[a];
      }
    }
    prefs.end();
  }
  cfgDirty = false;
}

static void markCfgDirty() {
  cfgDirty = true;
  cfgDirtyAtMs = nowMs();
}

static void saveWifiConfig() {
  if (!prefs.begin("cnc_xyzw", false)) return;
  prefs.putString("w_ssid", wifiSsid);
  prefs.putString("w_pass", wifiPass);
  prefs.putString("w_devname", wifiDevName);
  prefs.putUChar("ip_o4", ipOct4);
  prefs.end();
}

static void saveLinkWd() {
  if (!prefs.begin("cnc_xyzw", false)) return;
  prefs.putUInt("wd_ms", linkWdMs);
  prefs.end();
}

static void saveToken() {
  if (!prefs.begin("cnc_xyzw", false)) return;
  if (tokenState == TOK_UNSET) prefs.remove("tok");
  else prefs.putString("tok", token);
  prefs.end();
}

// ============================================================================
//  6. HARDWARE: ENABLE, DIRECCIÓN, PULSOS, FINALES DE CARRERA, TEMPORIZADOR
// ============================================================================
void IRAM_ATTR onStepTimer() {
  BaseType_t woken = pdFALSE;
  if (taskMotorsHandle != nullptr) {
    vTaskNotifyGiveFromISR(taskMotorsHandle, &woken);
    if (woken) portYIELD_FROM_ISR();
  }
}

static inline void pulseMask(uint32_t m) {
  REG_WRITE(GPIO_OUT_W1TS_REG, m);
  delayMicroseconds(STEP_PULSE_US);
  REG_WRITE(GPIO_OUT_W1TC_REG, m);
}

static inline void setDirPin(uint8_t a, bool fwd) {
  uint8_t lvl = fwd ? cfg[a].dirFwdLevel : (cfg[a].dirFwdLevel == HIGH ? LOW : HIGH);
  digitalWrite(PIN_DIR[a], lvl);
}

static inline bool limitRaw(uint8_t a) {
  return digitalRead(PIN_LIMIT[a]) == LIMIT_ACTIVE_LEVEL;
}

// Confirma el final de carrera con 3 lecturas (anti-ruido EMI): solo cuesta tiempo si está activo.
static bool limitConfirmed(uint8_t a) {
  if (!limitRaw(a)) return false;
  delayMicroseconds(30);
  if (!limitRaw(a)) return false;
  delayMicroseconds(30);
  return limitRaw(a);
}

static inline bool physEstopActive() {
#if PIN_ESTOP_INPUT >= 0
  return digitalRead(PIN_ESTOP_INPUT) == ESTOP_ACTIVE_LEVEL;
#else
  return false;
#endif
}

// ============================================================================
//  7. ESTADO, ALARMAS Y DETENCIÓN
// ============================================================================
static inline bool machineBusy() {
  return qCount > 0 || blockActive || motionMode != MM_NONE || jogPending;
}

static uint8_t currentState() {
  if (alarmCode != AL_NONE) return MS_ALARM;
  switch (motionMode) {
    case MM_HOME: return MS_HOMING;
    case MM_JOG:  return MS_MANUAL;
    case MM_MOVE: return MS_RUNNING;
    default: break;
  }
  if (jogPending) return MS_MANUAL;
  return (qCount > 0) ? MS_RUNNING : MS_IDLE;
}

// Puede llamarse desde cualquier tarea. 'hard' = detener ya; si no, con rampa.
static void raiseAlarm(uint8_t code, int8_t axis, bool hard) {
  const bool estop = (code == AL_ESTOP || code == AL_PHYS_ESTOP);
  if (estop) {
    digitalWrite(PIN_ENABLE_ACTUATORS, HIGH);       // lo primero: cortar los drivers
    actuatorsEnabled = false;
  }
  portENTER_CRITICAL(&sysMux);
  bool curIsEstop = (alarmCode == AL_ESTOP || alarmCode == AL_PHYS_ESTOP);
  if (alarmCode == AL_NONE || (estop && !curIsEstop)) {
    alarmCode = code;
    alarmAxis = axis;
    alarmEventPending = true;
  }
  portEXIT_CRITICAL(&sysMux);
  if (hard) hardStopReq = true; else softStopReq = true;

  if (estop) {
    for (uint8_t a = 0; a < AXIS_COUNT; a++) { rt[a].homed = 0; rt[a].err = AE_ESTOP; }
  } else if (code == AL_HARD_LIMIT) {
    for (uint8_t a = 0; a < AXIS_COUNT; a++) {
      if (rt[a].moving || (int8_t)a == axis) rt[a].homed = 0;
    }
    if (axis >= 0 && axis < AXIS_COUNT) rt[axis].err = AE_SWITCH_HIT;
  } else if (code == AL_COMM_LOST) {
    for (uint8_t a = 0; a < AXIS_COUNT; a++) if (rt[a].moving) rt[a].err = AE_COMM;
  }
  forceStatusPush = true;
}

// Llamar solo desde la tarea de motores, fuera de cualquier ejecución de bloque.
static void housekeepStop() {
  portENTER_CRITICAL(&qMux);
  qHead = 0; qTail = 0; qCount = 0;
  qEpoch = qEpoch + 1;
  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    planned[a] = rt[a].steps;
    plannedHomed[a] = rt[a].homed != 0;
  }
  portEXIT_CRITICAL(&qMux);
  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    rt[a].moving = 0;
    rt[a].moveDir = 0;
    rt[a].target = rt[a].steps;
  }
  jogPending = 0;
  jogStopReq = 0;
  jogAxisRun = 0xFF;
  motionMode = MM_NONE;
  blockActive = false;
  hardStopReq = false;
  softStopReq = false;
  forceStatusPush = true;
}

// Condiciones de aborto comunes a todos los ejecutores.
//   0 = continuar, 1 = detener YA, 2 = detener con rampa
static inline uint8_t abortCheck() {
  if (hardStopReq || !actuatorsEnabled) return 1;
  if (physEstopActive()) { raiseAlarm(AL_PHYS_ESTOP, -1, true); return 1; }
  if (linkWdMs != 0 && alarmCode == AL_NONE) {
    if (!sessionAuthorized || (uint32_t)(nowMs() - lastLinkMs) > linkWdMs) {
      raiseAlarm(AL_COMM_LOST, -1, false);
    }
  }
  if (softStopReq) return 2;
  return 0;
}

// ============================================================================
//  8. PLANIFICADOR: PERFILES DE VELOCIDAD Y COLA DE BLOQUES
// ============================================================================
static void buildProfile(Profile& p, uint32_t n, float vs, float vc, float ve, float a, bool s) {
  p.n = n;
  p.s = s;
  p.a = a < 1.0f ? 1.0f : a;
  vs = clampf(vs, MIN_START_SPS, MAX_SPS);
  vc = clampf(vc, MIN_START_SPS, MAX_SPS);
  ve = clampf(ve, MIN_START_SPS, MAX_SPS);
  if (vc < vs) vs = vc;
  if (vc < ve) ve = vc;
  const float k = s ? S_RAMP_FACTOR : 1.0f;
  float dAcc = k * (vc * vc - vs * vs) / (2.0f * p.a);
  float dDec = k * (vc * vc - ve * ve) / (2.0f * p.a);
  if (dAcc + dDec > (float)n) {
    // Movimiento corto: pico reducido (perfil triangular)
    const float vmax = vs > ve ? vs : ve;
    float vp2 = (2.0f * p.a * (float)n / k + vs * vs + ve * ve) * 0.5f;
    vc = (vp2 > vmax * vmax) ? sqrtf(vp2) : vmax;
    dAcc = k * (vc * vc - vs * vs) / (2.0f * p.a);
    dDec = k * (vc * vc - ve * ve) / (2.0f * p.a);
    if (dAcc + dDec > (float)n) {
      float sc = (float)n / (dAcc + dDec);
      dAcc *= sc;
      dDec *= sc;
    }
  }
  p.vs = vs; p.vc = vc; p.ve = ve;
  p.nAcc = (uint32_t)(dAcc + 0.5f);
  p.nDec = (uint32_t)(dDec + 0.5f);
  if (p.nAcc > n) p.nAcc = n;
  if (p.nAcc + p.nDec > n) p.nDec = n - p.nAcc;
}

// Velocidad (pasos/s del eje dominante) para el intervalo que sigue al paso 'i'.
static inline float profileSpeed(const Profile& p, uint32_t i) {
  float v = p.vc;
  if (i < p.nAcc) {
    if (p.s) {
      float x = ((float)i + 0.5f) / (float)p.nAcc;
      v = p.vs + (p.vc - p.vs) * 0.5f * (1.0f - cosf((float)M_PI * x));
    } else {
      v = sqrtf(p.vs * p.vs + 2.0f * p.a * ((float)i + 0.5f));
      if (v > p.vc) v = p.vc;
    }
  } else if (i + p.nDec >= p.n) {
    uint32_t j = p.n - 1 - i;
    if (p.s) {
      float x = ((float)j + 0.5f) / (float)p.nDec;
      v = p.ve + (p.vc - p.ve) * 0.5f * (1.0f - cosf((float)M_PI * x));
    } else {
      v = sqrtf(p.ve * p.ve + 2.0f * p.a * ((float)j + 0.5f));
      if (v > p.vc) v = p.vc;
    }
  }
  return v < MIN_START_SPS ? MIN_START_SPS : v;
}

static void snapPlan(PlanSnap& s) {
  portENTER_CRITICAL(&qMux);
  for (uint8_t a = 0; a < AXIS_COUNT; a++) { s.pos[a] = planned[a]; s.homed[a] = plannedHomed[a]; }
  s.epoch = qEpoch;
  portEXIT_CRITICAL(&qMux);
}

// Inserta el bloque si la planificación sigue vigente (no hubo vaciado de cola) y hay espacio.
static Rc commitBlock(const Block& b, const PlanSnap& s, const int32_t* newPos, const bool* newHomed) {
  Rc rc = RC_OK;
  portENTER_CRITICAL(&qMux);
  if (qEpoch != s.epoch) {
    rc = RC_BUSY;
  } else if (qCount >= QCAP) {
    rc = RC_FULL;
  } else {
    qbuf[qTail] = b;
    qTail = (uint8_t)((qTail + 1) % QCAP);
    qCount = (uint8_t)(qCount + 1);
    if (newPos)   for (uint8_t a = 0; a < AXIS_COUNT; a++) planned[a] = newPos[a];
    if (newHomed) for (uint8_t a = 0; a < AXIS_COUNT; a++) plannedHomed[a] = newHomed[a];
  }
  portEXIT_CRITICAL(&qMux);
  return rc;
}

// Extrae el siguiente bloque (tarea de motores). Marca blockActive de forma atómica.
static bool dequeueBlock(Block& out) {
  bool ok = false;
  portENTER_CRITICAL(&qMux);
  if (qCount > 0) {
    out = qbuf[qHead];
    qHead = (uint8_t)((qHead + 1) % QCAP);
    qCount = (uint8_t)(qCount - 1);
    blockActive = true;
    ok = true;
  }
  portEXIT_CRITICAL(&qMux);
  return ok;
}

static uint32_t assignId(uint32_t requested) {
  return requested ? requested : ++autoId;
}

// Velocidades del bloque a partir de la configuración de los ejes involucrados.
static void computeBlockSpeeds(Block& b, float feedMmS) {
  uint8_t dom = 0;
  for (uint8_t a = 0; a < AXIS_COUNT; a++) if (b.steps[a] > b.steps[dom]) dom = a;
  const float n = (float)b.nSteps;
  float vDom = MAX_SPS, aDom = 1.0e9f, vsDom = 1.0e9f, veDom = 1.0e9f, len2 = 0.0f;
  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    if (!b.steps[a]) continue;
    const AxisCfg& c = cfg[a];
    const float r = n / (float)b.steps[a];                       // eje dominante / este eje
    const float axMax = c.useSCurve ? c.cruiseMmS * c.stepsPerMm : 1000000.0f / (float)c.jogUs;
    vDom  = fminf(vDom,  axMax * r);
    aDom  = fminf(aDom,  c.accelMmS2 * c.stepsPerMm * r);
    vsDom = fminf(vsDom, c.startMmS * c.stepsPerMm * r);
    veDom = fminf(veDom, c.endMmS * c.stepsPerMm * r);
    const float d = (float)b.steps[a] / c.stepsPerMm;
    len2 += d * d;
  }
  if (feedMmS > 0.0f && len2 > 0.0f) vDom = fminf(vDom, n * feedMmS / sqrtf(len2));
  vDom = clampf(vDom, MIN_START_SPS, MAX_SPS);
  b.vCruise = vDom;
  b.vStart = clampf(fminf(vsDom, vDom), MIN_START_SPS, MAX_SPS);
  b.vEnd = clampf(fminf(veDom, vDom), MIN_START_SPS, MAX_SPS);
  b.accel = aDom;
  b.sCurve = cfg[dom].useSCurve;
}

// tgt[a] = NAN -> el eje no participa. Coordenadas de TRABAJO (mm). 'relative' = incrementos.
static Rc planMove(const float tgt[AXIS_COUNT], bool relative, float feedMmS, uint32_t id, int8_t* badAxis) {
  if (badAxis) *badAxis = -1;
  if (!actuatorsEnabled) return RC_DISABLED;
  if (alarmCode != AL_NONE) return RC_ALARM;
  if (motionMode == MM_JOG || jogPending) return RC_BUSY;

  for (int attempt = 0; attempt < 3; attempt++) {
    PlanSnap s;
    snapPlan(s);
    Block b;
    memset(&b, 0, sizeof(b));
    b.type = BT_MOVE;
    b.id = id;
    int32_t newPos[AXIS_COUNT];
    for (uint8_t a = 0; a < AXIS_COUNT; a++) newPos[a] = s.pos[a];

    for (uint8_t a = 0; a < AXIS_COUNT; a++) {
      if (isnan(tgt[a])) continue;
      if (!s.homed[a]) { if (badAxis) *badAxis = (int8_t)a; return RC_NOT_HOMED; }
      const float spm = cfg[a].stepsPerMm;
      const float tmm = tgt[a];
      if (!isfinite(tmm) || fabsf(tmm) > 100000.0f) return RC_BAD_ARGS;
      int32_t ts = relative ? s.pos[a] + (int32_t)lroundf(tmm * spm)
                            : (int32_t)lroundf(tmm * spm) + rt[a].woff;
      const int32_t tol = (int32_t)lroundf(SOFT_LIMIT_TOL_MM * spm);
      const int32_t maxS = (int32_t)lroundf(cfg[a].maxTravelMm * spm);
      if (ts < -tol || ts > maxS + tol) { if (badAxis) *badAxis = (int8_t)a; return RC_SOFT_LIMIT; }
      int32_t delta = ts - s.pos[a];
      if (delta >= 0) { b.steps[a] = (uint32_t)delta; b.dirMask |= (uint8_t)(1u << a); }
      else            { b.steps[a] = (uint32_t)(-delta); }
      newPos[a] = ts;
    }
    for (uint8_t a = 0; a < AXIS_COUNT; a++) if (b.steps[a] > b.nSteps) b.nSteps = b.steps[a];
    if (b.nSteps > 0) computeBlockSpeeds(b, feedMmS);
    Rc rc = commitBlock(b, s, newPos, nullptr);
    if (rc != RC_BUSY) return rc;
  }
  return RC_BUSY;
}

static Rc planHome(uint8_t a, uint32_t id) {
  if (!actuatorsEnabled) return RC_DISABLED;
  if (alarmCode != AL_NONE) return RC_ALARM;
  if (motionMode == MM_JOG || jogPending) return RC_BUSY;
  for (int attempt = 0; attempt < 3; attempt++) {
    PlanSnap s;
    snapPlan(s);
    Block b;
    memset(&b, 0, sizeof(b));
    b.type = BT_HOME;
    b.id = id;
    b.homeAxis = a;
    const AxisCfg& c = cfg[a];
    b.backoffSteps = c.backoffSteps;
    b.softOffsetSteps = c.softOffsetSteps;
    b.seekUs = c.seekUs;
    b.feedUs = c.feedUs;
    b.homeBoUs = c.homeBoUs;
    // Tope de búsqueda: 1.25 x recorrido + offset + retroceso (V1 usaba 2 x recorrido)
    b.maxSearchSteps = (uint32_t)(c.maxTravelMm * c.stepsPerMm * 1.25f) + c.softOffsetSteps + c.backoffSteps;
    int32_t newPos[AXIS_COUNT];
    bool newHomed[AXIS_COUNT];
    for (uint8_t i = 0; i < AXIS_COUNT; i++) { newPos[i] = s.pos[i]; newHomed[i] = s.homed[i]; }
    newPos[a] = 0;
    newHomed[a] = true;
    Rc rc = commitBlock(b, s, newPos, newHomed);
    if (rc != RC_BUSY) return rc;
  }
  return RC_BUSY;
}

static Rc planDwell(uint32_t ms, uint32_t id) {
  if (!actuatorsEnabled) return RC_DISABLED;
  if (alarmCode != AL_NONE) return RC_ALARM;
  if (motionMode == MM_JOG || jogPending) return RC_BUSY;
  if (ms > 600000UL) return RC_BAD_ARGS;
  for (int attempt = 0; attempt < 3; attempt++) {
    PlanSnap s;
    snapPlan(s);
    Block b;
    memset(&b, 0, sizeof(b));
    b.type = BT_DWELL;
    b.id = id;
    b.dwellMs = ms;
    Rc rc = commitBlock(b, s, nullptr, nullptr);
    if (rc != RC_BUSY) return rc;
  }
  return RC_BUSY;
}

// ============================================================================
//  9. EJECUTORES DE MOVIMIENTO (se ejecutan SOLO en la tarea de motores)
// ============================================================================
enum RunEnd : uint8_t { RE_DONE = 0, RE_SWITCH, RE_STOP_REQ, RE_HARD, RE_BOUND, RE_DEADMAN };

struct RunSpec {
  uint8_t  ax;
  bool     fwd;
  uint32_t maxSteps;
  float    vTarget, vMin, accel;     // pasos/s, pasos/s, pasos/s^2
  uint8_t  stopOn;                   // 0 nada, 1 al activarse el final de carrera, 2 al liberarse
  bool     hardLimit;                // moverse hacia el sensor ya activo => ALARM
  bool     jog;                      // respeta dead-man, stop manual y cambio de eje
  bool     useBounds;
  int32_t  low, high;                // límites en pasos (si useBounds)
};

static int64_t gDeadline = 0;

static inline void deadlineReset() { gDeadline = nowUs(); }

static inline float spsFromUs(int32_t us) {
  return clampf(1000000.0f / (float)(us < 1 ? 1 : us), MIN_START_SPS, MAX_SPS);
}

// Avanza el plazo absoluto 'dtUs' y espera hasta él. false = el temporizador no disparó.
// El plazo absoluto elimina la deriva por tiempo de cómputo (V1 reiniciaba el timer en cada paso).
static bool deadlineWait(float dtUs) {
  if (dtUs < (float)MIN_STEP_US) dtUs = (float)MIN_STEP_US;
  gDeadline += (int64_t)(dtUs + 0.5f);
  const int64_t now = nowUs();
  const int64_t rem = gDeadline - now;
  if (rem < -1000) { gDeadline = now; return true; }          // atrasados > 1 ms: resincronizar
  if (rem >= 50) {
    ulTaskNotifyTake(pdTRUE, 0);                              // descarta notificaciones viejas
    timerWrite(stepTimer, 0);
    timerAlarm(stepTimer, (uint64_t)(rem - 10), false, 0);
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS((uint32_t)(rem / 1000) + 30)) == 0) return false;
  }
  while (nowUs() < gDeadline) { }                             // últimos µs: espera activa
  return true;
}

// Espera 'ms' vigilando abortos. false = abortado.
static bool settleAbortable(uint32_t ms) {
  const uint32_t t0 = nowMs();
  while ((uint32_t)(nowMs() - t0) < ms) {
    if (abortCheck() != 0) return false;
    vTaskDelay(1);
  }
  return true;
}

static void resyncPlannedAxis(uint8_t a) {
  portENTER_CRITICAL(&qMux);
  planned[a] = rt[a].steps;
  portEXIT_CRITICAL(&qMux);
}

// Movimiento de un solo eje con rampa lineal (HOMING y JOG).
static RunEnd runAxis(const RunSpec& s, uint32_t& done) {
  const uint8_t a = s.ax;
  const float a2 = 2.0f * s.accel;
  const float vMin = s.vMin;
  float v = vMin < s.vTarget ? vMin : s.vTarget;
  bool decel = false;
  RunEnd end = RE_DONE;
  done = 0;

  setDirPin(a, s.fwd);
  rt[a].moveDir = s.fwd ? 1 : 2;
  rt[a].moving = 1;
  delayMicroseconds(DIR_SETUP_US);
  deadlineReset();

  for (;;) {
    const uint8_t ab = abortCheck();
    if (ab == 1) { end = RE_HARD; break; }
    if (ab == 2 && !decel) { decel = true; end = RE_STOP_REQ; }

    if (s.jog && !decel) {
      if (jogStopReq || jogPending) { decel = true; end = RE_STOP_REQ; }
      else if ((uint32_t)(nowMs() - jogLastPing) > JOG_DEADMAN_MS) { decel = true; end = RE_DEADMAN; }
    }

    if (s.stopOn == 1) {
      if (limitRaw(a) && limitConfirmed(a)) { end = RE_SWITCH; break; }
    } else if (s.stopOn == 2) {
      if (!limitRaw(a)) { end = RE_SWITCH; break; }
    }

    if (s.hardLimit && !s.fwd && limitRaw(a) && limitConfirmed(a)) {
      raiseAlarm(AL_HARD_LIMIT, (int8_t)a, true);
      end = RE_HARD;
      break;
    }

    if (s.useBounds) {
      const int32_t rem = s.fwd ? (s.high - rt[a].steps) : (rt[a].steps - s.low);
      if (rem <= 0) { if (end == RE_DONE) end = RE_BOUND; break; }
      if (!decel) {
        const float stopDist = (v * v - vMin * vMin) / a2;
        if ((float)rem <= stopDist + 1.0f) { decel = true; end = RE_BOUND; }
      }
    }

    if (done >= s.maxSteps) break;

    pulseMask(stepMask[a]);
    rt[a].steps += s.fwd ? 1 : -1;
    done++;

    if (decel) {
      const float v2 = v * v - a2;
      if (v2 <= vMin * vMin) {
        if (!deadlineWait(1000000.0f / vMin)) raiseAlarm(AL_INTERNAL, (int8_t)a, true);
        break;
      }
      v = sqrtf(v2);
    } else {
      v = sqrtf(v * v + a2);
      if (v > s.vTarget) v = s.vTarget;
    }
    if (!deadlineWait(1000000.0f / v)) { raiseAlarm(AL_INTERNAL, (int8_t)a, true); end = RE_HARD; break; }
  }

  rt[a].moving = 0;
  rt[a].moveDir = 0;
  return end;
}

// Bloque de movimiento lineal multieje (Bresenham sobre el eje dominante).
static void runMoveBlock(const Block& b) {
  motionMode = MM_MOVE;
  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    int32_t tgt = rt[a].steps;
    if (b.steps[a]) {
      tgt += (b.dirMask & (1u << a)) ? (int32_t)b.steps[a] : -(int32_t)b.steps[a];
      rt[a].err = AE_NONE;
    }
    rt[a].target = tgt;
  }
  if (b.nSteps == 0) { doneId = b.id; return; }

  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    if (!b.steps[a]) continue;
    const bool fwd = (b.dirMask >> a) & 1u;
    setDirPin(a, fwd);
    rt[a].moveDir = fwd ? 1 : 2;
    rt[a].moving = 1;
  }
  delayMicroseconds(DIR_SETUP_US);

  Profile p;
  buildProfile(p, b.nSteps, b.vStart, b.vCruise, b.vEnd, b.accel, b.sCurve != 0);
  const int32_t n = (int32_t)b.nSteps;
  int32_t errAcc[AXIS_COUNT] = {0, 0, 0, 0};
  uint32_t i = 0;
  uint32_t nTotal = p.n;
  float vNow = p.vs;
  bool stopping = false;
  uint8_t endKind = 0;                 // 0 normal, 1 aborto duro, 2 parada con rampa
  deadlineReset();

  while (i < nTotal) {
    const uint8_t ab = abortCheck();
    if (ab == 1) { endKind = 1; break; }
    if (ab == 2 && !stopping) {
      // Parada controlada: se recorta el bloque y se desacelera con la aceleración configurada
      const float ve = p.ve;
      uint32_t nStop = 0;
      if (vNow > ve) nStop = (uint32_t)ceilf((vNow * vNow - ve * ve) / (2.0f * p.a));
      if (i + nStop < nTotal) {
        nTotal = i + nStop;
        p.n = nTotal;
        p.nDec = nStop;
        p.nAcc = 0;
        p.vc = vNow;
        p.s = false;
      }
      stopping = true;
      endKind = 2;
      if (i >= nTotal) break;
    }

    uint32_t mask = 0;
    uint8_t axMask = 0;
    for (uint8_t a = 0; a < AXIS_COUNT; a++) {
      if (!b.steps[a]) continue;
      errAcc[a] += (int32_t)b.steps[a];
      if (errAcc[a] >= n) {
        errAcc[a] -= n;
        mask |= stepMask[a];
        axMask |= (uint8_t)(1u << a);
      }
    }

    bool tripped = false;
    for (uint8_t a = 0; a < AXIS_COUNT; a++) {
      if (!(axMask & (1u << a))) continue;
      if (!((b.dirMask >> a) & 1u) && limitRaw(a) && limitConfirmed(a)) {
        raiseAlarm(AL_HARD_LIMIT, (int8_t)a, true);
        tripped = true;
        break;
      }
    }
    if (tripped) { endKind = 1; break; }

    if (mask) {
      pulseMask(mask);
      for (uint8_t a = 0; a < AXIS_COUNT; a++) {
        if (axMask & (1u << a)) rt[a].steps += ((b.dirMask >> a) & 1u) ? 1 : -1;
      }
    }

    vNow = profileSpeed(p, i);
    i++;
    if (!deadlineWait(1000000.0f / vNow)) { raiseAlarm(AL_INTERNAL, -1, true); endKind = 1; break; }
  }

  for (uint8_t a = 0; a < AXIS_COUNT; a++) { rt[a].moving = 0; rt[a].moveDir = 0; }
  if (endKind == 0) {
    for (uint8_t a = 0; a < AXIS_COUNT; a++) rt[a].target = rt[a].steps;
    doneId = b.id;
  }
}

// Secuencia de HOMING: búsqueda -> retroceso -> reaproximación lenta -> offset -> cero.
static void homeFail(uint8_t a, uint8_t err) {
  rt[a].err = err;
  raiseAlarm(AL_HOME_FAIL, (int8_t)a, true);
}

static void runHomeBlock(const Block& b) {
  const uint8_t a = b.homeAxis;
  const AxisCfg& c = cfg[a];
  motionMode = MM_HOME;
  rt[a].homed = 0;
  rt[a].err = AE_NONE;
  rt[a].target = rt[a].steps;
  const float spm = c.stepsPerMm;
  const float accel = c.accelMmS2 * spm;
  const float vMin = fmaxf(MIN_START_SPS, c.startMmS * spm);
  uint32_t done = 0;
  RunEnd e;
  RunSpec s;
  memset(&s, 0, sizeof(s));
  s.ax = a;
  s.vMin = vMin;
  s.accel = accel;

  // 1) BÚSQUEDA hacia el sensor (sentido negativo). Si ya está sobre el sensor se omite.
  if (!limitConfirmed(a)) {
    s.fwd = false; s.maxSteps = b.maxSearchSteps; s.vTarget = spsFromUs(b.seekUs); s.stopOn = 1;
    e = runAxis(s, done);
    if (e == RE_HARD || e == RE_STOP_REQ) { if (e == RE_STOP_REQ) rt[a].err = AE_STOPPED; return; }
    if (e != RE_SWITCH) { homeFail(a, AE_SEEK_FAIL); return; }
    if (!settleAbortable(SETTLE_MS_HOMING)) return;
  }

  // 2) RETROCESO (sentido positivo) hasta liberar el sensor
  s.fwd = true; s.maxSteps = b.backoffSteps; s.vTarget = spsFromUs(b.homeBoUs); s.stopOn = 0;
  e = runAxis(s, done);
  if (e == RE_HARD || e == RE_STOP_REQ) { if (e == RE_STOP_REQ) rt[a].err = AE_STOPPED; return; }
  if (!settleAbortable(SETTLE_MS_HOMING)) return;
  if (limitRaw(a)) { homeFail(a, AE_BACKOFF_FAIL); return; }

  // 3) REAPROXIMACIÓN LENTA hasta activar el sensor
  s.fwd = false; s.maxSteps = (b.backoffSteps * 2 > 200) ? b.backoffSteps * 2 : 200; s.vTarget = spsFromUs(b.feedUs); s.stopOn = 1;
  e = runAxis(s, done);
  if (e == RE_HARD || e == RE_STOP_REQ) { if (e == RE_STOP_REQ) rt[a].err = AE_STOPPED; return; }
  if (e != RE_SWITCH) { homeFail(a, AE_REAPPROACH_FAIL); return; }
  if (!settleAbortable(SETTLE_MS_HOMING)) return;

  // 4) OFFSET desde el sensor hasta el cero de máquina
  s.fwd = true; s.maxSteps = b.softOffsetSteps; s.vTarget = spsFromUs(b.homeBoUs); s.stopOn = 0;
  e = runAxis(s, done);
  if (e == RE_HARD || e == RE_STOP_REQ) { if (e == RE_STOP_REQ) rt[a].err = AE_STOPPED; return; }

  // 5) Cero de máquina y de trabajo
  rt[a].steps = 0;
  rt[a].woff = 0;
  rt[a].target = 0;
  rt[a].homed = 1;
  if (hardStopReq || alarmCode != AL_NONE || !actuatorsEnabled) rt[a].homed = 0;   // E-STOP justo al final
  else doneId = b.id;
}

static void runDwellBlock(const Block& b) {
  motionMode = MM_MOVE;
  const uint32_t t0 = nowMs();
  while ((uint32_t)(nowMs() - t0) < b.dwellMs) {
    if (abortCheck() != 0) return;
    vTaskDelay(1);
  }
  doneId = b.id;
}

// Jog continuo de un eje con dead-man (manual_ping), rampa y límites suaves.
static void runJogRequest() {
  const uint8_t a = jogAxisReq;
  const bool fwd = jogFwdReq != 0;
  jogPending = 0;
  if (a >= AXIS_COUNT) return;
  const AxisCfg& c = cfg[a];
  const float spm = c.stepsPerMm;

  RunSpec s;
  memset(&s, 0, sizeof(s));
  s.ax = a;
  s.fwd = fwd;
  s.maxSteps = 0xFFFFFFFFUL;
  s.vTarget = spsFromUs(c.manualUs);
  s.vMin = clampf(c.startMmS * spm, MIN_START_SPS, s.vTarget);
  s.accel = c.accelMmS2 * spm;
  s.jog = true;
  s.useBounds = true;
  if (rt[a].homed) {
    const int32_t tol = (int32_t)lroundf(SOFT_LIMIT_TOL_MM * spm);
    s.low = -tol;
    s.high = (int32_t)lroundf(c.maxTravelMm * spm) + tol;
    s.hardLimit = true;
  } else {
    // Recuperación: eje sin referenciar sobre el sensor; solo se permite alejarse (+) un tramo corto
    s.low = INT32_MIN / 2;
    s.high = rt[a].steps + (int32_t)lroundf(20.0f * spm);
    s.hardLimit = false;
  }

  motionMode = MM_JOG;
  jogAxisRun = a;
  jogFwdRun = fwd ? 1 : 0;
  rt[a].err = AE_NONE;
  rt[a].target = rt[a].steps;
  uint32_t done = 0;
  const RunEnd e = runAxis(s, done);
  if (e == RE_DEADMAN) rt[a].err = AE_DEADMAN;
  rt[a].target = rt[a].steps;
  resyncPlannedAxis(a);
  jogAxisRun = 0xFF;
  jogStopReq = 0;
  motionMode = MM_NONE;
}

static void executeBlock(const Block& b) {
  switch (b.type) {
    case BT_MOVE:  runMoveBlock(b); break;
    case BT_HOME:  runHomeBlock(b); break;
    case BT_DWELL: runDwellBlock(b); break;
    default: break;
  }
  motionMode = MM_NONE;
  blockActive = false;
  forceStatusPush = true;
}

static void taskMotors(void* arg) {
  (void)arg;
  for (;;) {
    if (hardStopReq || softStopReq) housekeepStop();
    if (alarmCode != AL_NONE || !actuatorsEnabled) {
      if (qCount > 0 || jogPending || motionMode != MM_NONE) housekeepStop();
      vTaskDelay(1);
      continue;
    }
    if ((uint32_t)(nowMs() - enabledAtMs) < ENABLE_SETTLE_MS) { vTaskDelay(1); continue; }
    if (jogPending) { runJogRequest(); continue; }
    Block b{};
    if (dequeueBlock(b)) { executeBlock(b); continue; }
    vTaskDelay(1);
  }
}

// ============================================================================
//  10. SALIDA HACIA EL CLIENTE, TELEMETRÍA Y CONFIGURACIÓN
// ============================================================================
// Salida NO bloqueante. Cada línea se copia ENTERA al búfer de transmisión o se descarta ENTERA
// (nunca se envía una línea truncada). 'netFlush()' vacía el búfer con send(MSG_DONTWAIT): un
// cliente lento o colgado nunca bloquea loop() (V1 podía bloquearse segundos en client.print()).
static char     txBuf[1024];
static uint16_t txLen = 0;
static uint32_t txStallSince = 0;

static bool netEnqueue(const char* s, size_t n, bool droppable) {
  if (!ses.active) return false;
  const size_t freeB = sizeof(txBuf) - txLen;
  if (droppable) { if (freeB < n + 320) return false; }       // se reserva espacio para ACK/NACK
  else if (freeB < n) return false;
  memcpy(txBuf + txLen, s, n);
  txLen = (uint16_t)(txLen + n);
  return true;
}

static void netSendBuf(OutBuf& o, bool droppable = false) {
  if (!o.ovf) netEnqueue(o.p, o.len, droppable);
}

static char gAckExtra[48];
static char gNackDetail[24];

static void replyAck(const char* cmd) {
  char b[96];
  OutBuf o(b, sizeof(b));
  if (gAckExtra[0]) o.add("ACK|%s|OK|%s\n", cmd, gAckExtra);
  else o.add("ACK|%s|OK\n", cmd);
  netSendBuf(o);
}

static void replyNack(const char* cmd, const char* reason) {
  char b[128];
  OutBuf o(b, sizeof(b));
  if (gNackDetail[0]) o.add("NACK|%s|%s|%s\n", cmd, reason, gNackDetail);
  else o.add("NACK|%s|%s\n", cmd, reason);
  netSendBuf(o);
}

static void buildStatus(OutBuf& o) {
  const uint32_t depth = (uint32_t)qCount + (blockActive ? 1u : 0u);
  o.add("ST|%s|%d|%u|%lu|%lu", MS_NAME[currentState()], actuatorsEnabled ? 1 : 0, (unsigned)alarmCode,
        (unsigned long)depth, (unsigned long)doneId);
  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    const int32_t st = rt[a].steps;
    const int32_t wo = rt[a].woff;
    const int32_t tg = rt[a].target;
    const float spm = cfg[a].stepsPerMm;
    const unsigned flags = (rt[a].homed ? 1u : 0u) | (cfg[a].calibrated ? 2u : 0u) |
                           (rt[a].moving ? 4u : 0u) | (rt[a].limit ? 8u : 0u);
    o.add("|%.3f,%.3f,%ld,%u,%u,%u", (double)((float)(st - wo) / spm), (double)((float)(tg - wo) / spm),
          (long)st, flags, (unsigned)rt[a].moveDir, (unsigned)rt[a].err);
  }
  o.add("\n");
}

static void sendStatus(bool droppable) {
  char b[320];
  OutBuf o(b, sizeof(b));
  buildStatus(o);
  netSendBuf(o, droppable);
}

static void buildAxisConfig(OutBuf& o, uint8_t a) {
  const AxisCfg& c = cfg[a];
  o.add("CF|%c|%.4f|%.3f|%lu|%lu|%d|%d|%ld|%ld|%ld|%ld|%ld|%.3f|%.3f|%.3f|%.2f|%d|%s\n", AXIS_CHAR[a],
        (double)c.stepsPerMm, (double)c.maxTravelMm, (unsigned long)c.backoffSteps,
        (unsigned long)c.softOffsetSteps, c.useSCurve ? 1 : 0, c.dirFwdLevel == HIGH ? 1 : 0,
        (long)c.seekUs, (long)c.feedUs, (long)c.homeBoUs, (long)c.manualUs, (long)c.jogUs,
        (double)c.startMmS, (double)c.cruiseMmS, (double)c.endMmS, (double)c.accelMmS2,
        c.calibrated ? 1 : 0, c.lastCal);
}

static void sendAxisConfig(uint8_t a) {
  char b[256];
  OutBuf o(b, sizeof(b));
  buildAxisConfig(o, a);
  netSendBuf(o);
}

static void sendGlobalConfig() {
  char b[96];
  OutBuf o(b, sizeof(b));
  o.add("CF|G|%lu|%s|%d|%d|%d\n", (unsigned long)linkWdMs, FW_VERSION, PROTO_VERSION, (int)tokenState,
        (int)ipOct4);
  netSendBuf(o);
}

static void sendAllConfig() {
  sendGlobalConfig();
  for (uint8_t a = 0; a < AXIS_COUNT; a++) sendAxisConfig(a);
}

static void sendAlarmEvent() {
  char b[96];
  OutBuf o(b, sizeof(b));
  const int8_t ax = alarmAxis;
  o.add("EV|ALARM|%u|%c|%s\n", (unsigned)alarmCode, (ax >= 0 && ax < AXIS_COUNT) ? AXIS_CHAR[ax] : '-',
        AL_TEXT[alarmCode < sizeof(AL_TEXT) / sizeof(AL_TEXT[0]) ? alarmCode : 0]);
  netSendBuf(o);
}

// ============================================================================
//  11. API INTERNA DE COMANDOS (compartida por NET-LOG y dialecto GRBL)
//      Se ejecuta SIEMPRE en la tarea loop().
// ============================================================================
static Rc apiEnable() {
  if (alarmCode != AL_NONE || physEstopActive()) return RC_ALARM;
  if (actuatorsEnabled) return RC_OK;
  enabledAtMs = nowMs();
  digitalWrite(PIN_ENABLE_ACTUATORS, LOW);
  actuatorsEnabled = true;
  forceStatusPush = true;
  return RC_OK;
}

// Sin par de sujeción no hay garantía de posición: se pierde el referenciado.
static Rc apiDisable() {
  digitalWrite(PIN_ENABLE_ACTUATORS, HIGH);
  actuatorsEnabled = false;
  for (uint8_t a = 0; a < AXIS_COUNT; a++) rt[a].homed = 0;
  hardStopReq = true;
  forceStatusPush = true;
  return RC_OK;
}

static Rc apiEstop() {
  raiseAlarm(AL_ESTOP, -1, true);
  return RC_OK;
}

static Rc apiStop() {
  softStopReq = true;
  jogPending = 0;
  jogStopReq = 1;
  forceStatusPush = true;
  return RC_OK;
}

static Rc apiClearAlarm() {
  if (machineBusy() || hardStopReq || softStopReq) return RC_BUSY;
  if (physEstopActive()) return RC_ALARM;
  portENTER_CRITICAL(&sysMux);
  alarmCode = AL_NONE;
  alarmAxis = -1;
  alarmEventPending = false;
  portEXIT_CRITICAL(&sysMux);
  for (uint8_t a = 0; a < AXIS_COUNT; a++) rt[a].err = AE_NONE;
  forceStatusPush = true;
  return RC_OK;
}

static Rc apiHome(uint8_t a, uint32_t id) { return planHome(a, id); }

static Rc apiMoveAxis(uint8_t a, float v, bool relative, float feed, uint32_t id) {
  float t[AXIS_COUNT] = {NAN, NAN, NAN, NAN};
  t[a] = v;
  int8_t bad;
  Rc rc = planMove(t, relative, feed, id, &bad);
  if (rc != RC_OK && bad >= 0) { gNackDetail[0] = AXIS_CHAR[bad]; gNackDetail[1] = 0; }
  return rc;
}

static Rc apiMoveMulti(const float t[AXIS_COUNT], float feed, uint32_t id) {
  int8_t bad;
  Rc rc = planMove(t, false, feed, id, &bad);
  if (rc != RC_OK && bad >= 0) { gNackDetail[0] = AXIS_CHAR[bad]; gNackDetail[1] = 0; }
  return rc;
}

static Rc apiJogStart(uint8_t a, bool fwd) {
  if (!actuatorsEnabled) return RC_DISABLED;
  if (alarmCode != AL_NONE) return RC_ALARM;
  if (qCount > 0 || blockActive || (motionMode != MM_NONE && motionMode != MM_JOG)) return RC_BUSY;
  if (!rt[a].homed && !(fwd && limitRaw(a))) return RC_NOT_HOMED;     // recuperación: solo salir del sensor
  jogLastPing = nowMs();
  if (motionMode == MM_JOG && !jogPending && jogAxisRun == a && jogFwdRun == (fwd ? 1 : 0)) return RC_OK;
  if (motionMode != MM_JOG) jogStopReq = 0;
  jogAxisReq = a;
  jogFwdReq = fwd ? 1 : 0;
  jogPending = 1;
  forceStatusPush = true;
  return RC_OK;
}

static Rc apiJogStop(int8_t a) {
  if (jogPending && (a < 0 || (uint8_t)a == jogAxisReq)) jogPending = 0;
  if (motionMode == MM_JOG && (a < 0 || (uint8_t)a == jogAxisRun)) jogStopReq = 1;
  return RC_OK;
}

static Rc apiSetZero(uint8_t a, float valueMm) {
  if (machineBusy()) return RC_BUSY;
  if (!rt[a].homed) return RC_NOT_HOMED;
  rt[a].woff = rt[a].steps - (int32_t)lroundf(valueMm * cfg[a].stepsPerMm);
  forceStatusPush = true;
  return RC_OK;
}

static bool inRangeF(float v, float lo, float hi) { return isfinite(v) && v >= lo && v <= hi; }

// Cambios de configuración: solo con el equipo en reposo (la tarea de motores lee cfg sin bloqueo).
static Rc apiSetCalibration(uint8_t a, float spm, float maxTravel, const char* date) {
  if (machineBusy()) return RC_BUSY;
  if (!inRangeF(spm, 1.0f, 100000.0f) || !inRangeF(maxTravel, 1.0f, 2000.0f)) return RC_BAD_ARGS;
  cfg[a].stepsPerMm = spm;
  cfg[a].maxTravelMm = maxTravel;
  cfg[a].calibrated = 1;
  sanitizeText(cfg[a].lastCal, sizeof(cfg[a].lastCal), (date && date[0]) ? date : "Sin datos");
  markCfgDirty();
  return RC_OK;
}

static Rc apiSetLimits(uint8_t a, float maxTravel, long bo, long so) {
  if (machineBusy()) return RC_BUSY;
  if (!inRangeF(maxTravel, 1.0f, 2000.0f) || bo < 0 || bo > 1000000L || so < 0 || so > 1000000L) return RC_BAD_ARGS;
  cfg[a].maxTravelMm = maxTravel;
  cfg[a].backoffSteps = (uint32_t)bo;
  cfg[a].softOffsetSteps = (uint32_t)so;
  markCfgDirty();
  return RC_OK;
}

static Rc apiSetProfile(uint8_t a, float st, float cr, float en, float ac) {
  if (machineBusy()) return RC_BUSY;
  if (!inRangeF(st, 0.05f, 500.0f) || !inRangeF(cr, 0.1f, 1000.0f) || !inRangeF(en, 0.05f, 500.0f) ||
      !inRangeF(ac, 1.0f, 20000.0f) || cr < st || cr < en) return RC_BAD_ARGS;
  cfg[a].startMmS = st;
  cfg[a].cruiseMmS = cr;
  cfg[a].endMmS = en;
  cfg[a].accelMmS2 = ac;
  markCfgDirty();
  return RC_OK;
}

static Rc apiSetMode(uint8_t a, bool scurve) {
  if (machineBusy()) return RC_BUSY;
  cfg[a].useSCurve = scurve ? 1 : 0;
  markCfgDirty();
  return RC_OK;
}

static Rc apiSetManualSpeed(uint8_t a, long man, long jog) {
  if (machineBusy()) return RC_BUSY;
  if (man < (long)MIN_STEP_US || man > 200000L || jog < (long)MIN_STEP_US || jog > 200000L) return RC_BAD_ARGS;
  cfg[a].manualUs = (int32_t)man;
  cfg[a].jogUs = (int32_t)jog;
  markCfgDirty();
  return RC_OK;
}

static Rc apiSetHomingSpeed(uint8_t a, long seek, long feed, long bo) {
  if (machineBusy()) return RC_BUSY;
  if (seek < (long)MIN_STEP_US || seek > 200000L || feed < (long)MIN_STEP_US || feed > 200000L ||
      bo < (long)MIN_STEP_US || bo > 200000L) return RC_BAD_ARGS;
  cfg[a].seekUs = (int32_t)seek;
  cfg[a].feedUs = (int32_t)feed;
  cfg[a].homeBoUs = (int32_t)bo;
  markCfgDirty();
  return RC_OK;
}

static Rc apiInvertDir(uint8_t a) {
  if (machineBusy()) return RC_BUSY;
  cfg[a].dirFwdLevel = (cfg[a].dirFwdLevel == HIGH) ? LOW : HIGH;
  rt[a].homed = 0;                                   // el sentido de HOMING cambia con la inversión
  portENTER_CRITICAL(&qMux);
  plannedHomed[a] = false;
  portEXIT_CRITICAL(&qMux);
  markCfgDirty();
  return RC_OK;
}

static Rc apiSetWatchdog(long ms) {
  if (ms != 0 && (ms < 100 || ms > 5000)) return RC_BAD_ARGS;
  linkWdMs = (uint32_t)ms;
  saveLinkWd();
  return RC_OK;
}

// ============================================================================
//  12. SESIÓN DE RED: transmisión, vida del cliente y autenticación
// ============================================================================
static bool authorizedNow() {
  return ses.active && (tokenState == TOK_OPEN || (tokenState == TOK_SET && ses.authed));
}

static void sessionClose() {
  if (cl) cl.stop();
  memset(&ses, 0, sizeof(ses));
  txLen = 0;
  txStallSince = 0;
  grblHold = false;
  sessionAuthorized = false;       // la tarea de motores lo ve: movimiento en curso => AL_COMM_LOST
  forceStatusPush = true;
}

static void netFlush() {
  if (!ses.active || txLen == 0) return;
  const int fd = cl.fd();
  if (fd < 0) { sessionClose(); return; }
  const int r = send(fd, txBuf, txLen, MSG_DONTWAIT);
  if (r > 0) {
    if ((uint16_t)r < txLen) memmove(txBuf, txBuf + r, txLen - (uint16_t)r);
    txLen = (uint16_t)(txLen - (uint16_t)r);
    txStallSince = 0;
  } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOMEM) {
    sessionClose();
  } else {
    const uint32_t now = nowMs();
    if (txStallSince == 0) txStallSince = now;
    else if ((uint32_t)(now - txStallSince) > 2000) sessionClose();   // cliente colgado
  }
}

// recv(MSG_PEEK) distingue "sin datos" de "el par cerró" (NetworkClient::connected() no lo hace bien).
static bool peerAlive() {
  const int fd = cl.fd();
  if (fd < 0) return false;
  uint8_t d;
  const int r = recv(fd, &d, 1, MSG_PEEK | MSG_DONTWAIT);
  if (r == 0) return false;
  if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN) return false;
  return true;
}

static bool hmacHex(const char* key, const char* msg, char out[65]) {
  uint8_t mac[32];
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!info) return false;
  if (mbedtls_md_hmac(info, (const uint8_t*)key, strlen(key), (const uint8_t*)msg, strlen(msg), mac) != 0) return false;
  toHex(mac, 32, out);
  return true;
}

static void sessionOpen(const WiFiClient& c) {
  cl = c;
  cl.setNoDelay(true);
  memset(&ses, 0, sizeof(ses));
  ses.active = true;
  ses.lastRxMs = nowMs();
  txLen = 0;
  txStallSince = 0;
  grblRel = false;
  grblMotionG1 = false;
  grblFeedMmMin = 0.0f;
  grblHold = false;
  uint8_t r[16];
  esp_fill_random(r, sizeof(r));
  toHex(r, sizeof(r), ses.nonce);
  sessionAuthorized = authorizedNow();
  char b[112];
  OutBuf o(b, sizeof(b));
  o.add("HELLO|CNC-XYZW|%s|%d|%d|%s\n", FW_VERSION, PROTO_VERSION, (int)tokenState, ses.nonce);
  netSendBuf(o);
  netFlush();
}

static void authFailure() {
  authFailCount++;
  if (authFailCount >= AUTH_MAX_FAILS) {
    authLocked = true;
    authLockUntilMs = nowMs() + AUTH_LOCKOUT_MS;
    replyNack("AUTH", "locked");
    netFlush();
    sessionClose();
  } else {
    replyNack("AUTH", "bad_credentials");
  }
}

static void handleAuth(const char* resp) {
  if (tokenState == TOK_OPEN) { replyAck("AUTH"); sendAllConfig(); return; }
  if (tokenState != TOK_SET) { replyNack("AUTH", "not_provisioned"); return; }
  char expect[65];
  if (!hmacHex(token, ses.nonce, expect)) { replyNack("AUTH", "internal"); return; }
  char got[65];
  size_t n = 0;
  for (; resp[n] && n < 64; n++) got[n] = (char)tolower((unsigned char)resp[n]);
  got[n] = 0;
  const bool lenOk = (n == 64 && resp[n] == 0);
  if (lenOk && ctEqual(expect, got, 64)) {
    ses.authed = true;
    authFailCount = 0;
    sessionAuthorized = true;
    lastLinkMs = nowMs();
    replyAck("AUTH");
    sendAllConfig();
    forceStatusPush = true;
  } else {
    authFailure();
  }
}

// ============================================================================
//  13. COMANDOS NET-LOG (CMD|nombre|args...)
// ============================================================================
static bool isOmitted(const char* s) { return !s[0] || (s[0] == '-' && !s[1]); }

static bool parseOptId(int n, char** a, int idx, uint32_t& id) {
  id = 0;
  if (idx >= n || !a[idx][0]) return true;
  long v;
  if (!parseLongStrict(a[idx], v) || v <= 0 || v >= 0x7FFFFFFFL) return false;
  id = (uint32_t)v;
  return true;
}

static void setAckId(uint32_t id) { snprintf(gAckExtra, sizeof(gAckExtra), "%lu", (unsigned long)id); }

typedef Rc (*CmdFn)(int n, char** a);

static Rc cmdEnable(int, char**)  { return apiEnable(); }
static Rc cmdDisable(int, char**) { return apiDisable(); }
static Rc cmdEstop(int, char**)   { return apiEstop(); }
static Rc cmdStop(int, char**)    { return apiStop(); }
static Rc cmdClear(int, char**)   { return apiClearAlarm(); }
static Rc cmdPing(int, char**)    { jogLastPing = nowMs(); return RC_OK; }

static Rc cmdHome(int n, char** a) {
  uint8_t ax;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  if (n == 2) return RC_BAD_ARGS;
  uint32_t reqId;
  if (!parseOptId(n, a, 3, reqId)) return RC_BAD_ARGS;
  long bo = -1, so = -1;
  if (n >= 3) {
    if (!isOmitted(a[1])) { if (!parseLongStrict(a[1], bo) || bo < 0 || bo > 1000000L) return RC_BAD_ARGS; }
    if (!isOmitted(a[2])) { if (!parseLongStrict(a[2], so) || so < 0 || so > 1000000L) return RC_BAD_ARGS; }
  }
  const uint32_t oldBo = cfg[ax].backoffSteps, oldSo = cfg[ax].softOffsetSteps;
  if (bo >= 0) cfg[ax].backoffSteps = (uint32_t)bo;
  if (so >= 0) cfg[ax].softOffsetSteps = (uint32_t)so;
  const uint32_t id = assignId(reqId);
  const Rc rc = apiHome(ax, id);
  if (rc == RC_OK) {
    setAckId(id);
    if (cfg[ax].backoffSteps != oldBo || cfg[ax].softOffsetSteps != oldSo) markCfgDirty();
  } else {
    cfg[ax].backoffSteps = oldBo;
    cfg[ax].softOffsetSteps = oldSo;
  }
  return rc;
}

static Rc moveAxisCmd(int n, char** a, bool relative) {
  uint8_t ax;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  float v;
  if (!parseFloatStrict(a[1], v)) return RC_BAD_ARGS;
  uint32_t reqId;
  if (!parseOptId(n, a, 2, reqId)) return RC_BAD_ARGS;
  const uint32_t id = assignId(reqId);
  const Rc rc = apiMoveAxis(ax, v, relative, 0.0f, id);
  if (rc == RC_OK) setAckId(id);
  return rc;
}
static Rc cmdMoveAbs(int n, char** a) { return moveAxisCmd(n, a, false); }
static Rc cmdMoveRel(int n, char** a) { return moveAxisCmd(n, a, true); }

static Rc cmdMoveMulti(int n, char** a) {
  float t[AXIS_COUNT];
  bool any = false;
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    if (isOmitted(a[i])) { t[i] = NAN; continue; }
    if (!parseFloatStrict(a[i], t[i])) return RC_BAD_ARGS;
    any = true;
  }
  if (!any) return RC_BAD_ARGS;
  float feed = 0.0f;
  if (n > 4 && a[4][0]) {
    if (!parseFloatStrict(a[4], feed) || feed < 0.0f || feed > 10000.0f) return RC_BAD_ARGS;
  }
  uint32_t reqId;
  if (!parseOptId(n, a, 5, reqId)) return RC_BAD_ARGS;
  const uint32_t id = assignId(reqId);
  const Rc rc = apiMoveMulti(t, feed, id);
  if (rc == RC_OK) setAckId(id);
  return rc;
}

static Rc cmdDwell(int n, char** a) {
  long ms;
  if (!parseLongStrict(a[0], ms) || ms < 0) return RC_BAD_ARGS;
  uint32_t reqId;
  if (!parseOptId(n, a, 1, reqId)) return RC_BAD_ARGS;
  const uint32_t id = assignId(reqId);
  const Rc rc = planDwell((uint32_t)ms, id);
  if (rc == RC_OK) setAckId(id);
  return rc;
}

static Rc cmdManualStart(int, char** a) {
  uint8_t ax;
  bool fwd;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  if (!parseDirTok(a[1], fwd)) return RC_BAD_ARGS;
  return apiJogStart(ax, fwd);
}

static Rc cmdManualStop(int n, char** a) {
  int8_t ax = -1;
  if (n >= 1 && a[0][0] && strcmp(a[0], "all") != 0) {
    uint8_t t;
    if (!parseAxisTok(a[0], t)) return RC_BAD_AXIS;
    ax = (int8_t)t;
  }
  return apiJogStop(ax);
}

static Rc cmdSetZero(int n, char** a) {
  uint8_t ax;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  float v = 0.0f;
  if (n >= 2 && a[1][0] && !parseFloatStrict(a[1], v)) return RC_BAD_ARGS;
  return apiSetZero(ax, v);
}

static Rc cmdSetCal(int n, char** a) {
  uint8_t ax;
  float spm, mx;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  if (!parseFloatStrict(a[1], spm) || !parseFloatStrict(a[2], mx)) return RC_BAD_ARGS;
  return apiSetCalibration(ax, spm, mx, n >= 4 ? a[3] : "");
}

static Rc cmdSetLimits(int, char** a) {
  uint8_t ax;
  float mx;
  long bo, so;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  if (!parseFloatStrict(a[1], mx) || !parseLongStrict(a[2], bo) || !parseLongStrict(a[3], so)) return RC_BAD_ARGS;
  return apiSetLimits(ax, mx, bo, so);
}

static Rc cmdSetProfile(int, char** a) {
  uint8_t ax;
  float v[4];
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  for (int i = 0; i < 4; i++) if (!parseFloatStrict(a[1 + i], v[i])) return RC_BAD_ARGS;
  return apiSetProfile(ax, v[0], v[1], v[2], v[3]);
}

static Rc cmdSetMode(int, char** a) {
  uint8_t ax;
  long m;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  if (!parseLongStrict(a[1], m) || (m != 0 && m != 1)) return RC_BAD_ARGS;
  return apiSetMode(ax, m == 1);
}

static Rc cmdSetManualSpeed(int, char** a) {
  uint8_t ax;
  long man, jog;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  if (!parseLongStrict(a[1], man) || !parseLongStrict(a[2], jog)) return RC_BAD_ARGS;
  return apiSetManualSpeed(ax, man, jog);
}

static Rc cmdSetHomingSpeed(int, char** a) {
  uint8_t ax;
  long seek, feed, bo;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  if (!parseLongStrict(a[1], seek) || !parseLongStrict(a[2], feed) || !parseLongStrict(a[3], bo)) return RC_BAD_ARGS;
  return apiSetHomingSpeed(ax, seek, feed, bo);
}

static Rc cmdInvertDir(int, char** a) {
  uint8_t ax;
  if (!parseAxisTok(a[0], ax)) return RC_BAD_AXIS;
  return apiInvertDir(ax);
}

static Rc cmdSetWatchdog(int, char** a) {
  long ms;
  if (!parseLongStrict(a[0], ms)) return RC_BAD_ARGS;
  return apiSetWatchdog(ms);
}

static Rc cmdDeprecated(int, char**) { return RC_DEPRECATED; }

struct CmdDef {
  const char* name;
  int8_t      minArgs;
  int8_t      maxArgs;
  bool        preAuth;      // permitido sin autenticación
  bool        cfgChange;    // tras OK se reenvía la configuración
  CmdFn       fn;
};

static const CmdDef CMDS[] = {
  {"emergency_stop",         0, 0, true,  false, cmdEstop},
  {"stop",                   0, 0, false, false, cmdStop},
  {"clear_alarm",            0, 0, false, false, cmdClear},
  {"enable_actuators",       0, 0, false, false, cmdEnable},
  {"disable_actuators",      0, 0, false, false, cmdDisable},
  {"home_axis",              1, 4, false, true,  cmdHome},
  {"move_axis_abs",          2, 3, false, false, cmdMoveAbs},
  {"move_axis_rel",          2, 3, false, false, cmdMoveRel},
  {"move_multi_abs",         4, 6, false, false, cmdMoveMulti},
  {"dwell",                  1, 2, false, false, cmdDwell},
  {"manual_start",           2, 2, false, false, cmdManualStart},
  {"manual_ping",            0, 0, false, false, cmdPing},
  {"manual_stop",            0, 1, false, false, cmdManualStop},
  {"set_zero_axis",          1, 2, false, false, cmdSetZero},
  {"set_calibration_axis",   3, 4, false, true,  cmdSetCal},
  {"set_limits_axis",        4, 4, false, true,  cmdSetLimits},
  {"set_profile_axis",       5, 5, false, true,  cmdSetProfile},
  {"set_axis_mode",          2, 2, false, true,  cmdSetMode},
  {"set_manual_speed_axis",  3, 3, false, true,  cmdSetManualSpeed},
  {"set_homing_speed_axis",  4, 4, false, true,  cmdSetHomingSpeed},
  {"invert_axis_dir",        1, 1, false, true,  cmdInvertDir},
  {"set_watchdog",           1, 1, false, true,  cmdSetWatchdog},
  {"reset_step_counter",     0, 8, false, false, cmdDeprecated},
  {"set_scurve_profile_axis",0, 8, false, false, cmdDeprecated},
};

static void dispatchCmd(char* name, int n, char** a) {
  char shown[28];
  sanitizeText(shown, sizeof(shown), name);
  gAckExtra[0] = 0;
  gNackDetail[0] = 0;
  if (!strcmp(name, "emergency_stop")) {          // sin validar nada más: la parada de emergencia no se rechaza
    apiEstop();
    replyAck("emergency_stop");
    return;
  }
  const CmdDef* d = nullptr;
  for (size_t i = 0; i < sizeof(CMDS) / sizeof(CMDS[0]); i++) {
    if (!strcmp(name, CMDS[i].name)) { d = &CMDS[i]; break; }
  }
  if (!d) { replyNack(shown, RC_TEXT[RC_UNKNOWN]); return; }
  if (!d->preAuth && !authorizedNow()) { replyNack(shown, RC_TEXT[RC_AUTH]); return; }
  if (n < d->minArgs || n > d->maxArgs) { replyNack(shown, RC_TEXT[RC_BAD_ARGS]); return; }
  const Rc rc = d->fn(n, a);
  if (rc == RC_OK) {
    replyAck(d->name);
    if (d->cfgChange) sendAllConfig();
  } else {
    replyNack(d->name, RC_TEXT[rc]);
  }
}

// Línea del protocolo NET-LOG: AUTH | PING | GET_STATUS | GET_CONFIG | CMD|...
static void handleNetLine(char* line) {
  char* argv[16];
  const int argc = splitFields(line, argv, 16);
  gAckExtra[0] = 0;
  gNackDetail[0] = 0;
  if (argc < 0) { replyNack("line", "too_many_fields"); return; }
  const char* c0 = argv[0];
  if (!strcmp(c0, "CMD")) {
    if (argc < 2) { replyNack("CMD", RC_TEXT[RC_BAD_ARGS]); return; }
    ses.netlog = true;
    dispatchCmd(argv[1], argc - 2, argv + 2);
  } else if (!strcmp(c0, "PING")) {
    ses.netlog = true;
    replyAck("PING");
  } else if (!strcmp(c0, "AUTH")) {
    handleAuth(argc > 1 ? argv[1] : "");
  } else if (!strcmp(c0, "GET_STATUS")) {
    ses.netlog = true;
    sendStatus(false);
  } else if (!strcmp(c0, "GET_CONFIG")) {
    ses.netlog = true;
    if (!authorizedNow()) replyNack("GET_CONFIG", RC_TEXT[RC_AUTH]);
    else sendAllConfig();
  } else {
    char shown[28];
    sanitizeText(shown, sizeof(shown), c0);
    replyNack(shown, RC_TEXT[RC_UNKNOWN]);
  }
}

// ============================================================================
//  14. DIALECTO GRBL (subconjunto real, solo sesiones autorizadas)
//      Compatible con lo que envía la GUI en modo GRBL ($J=, $H, $X, !, ?, 0x85)
//      y con G-code básico: G0 G1 G4 G90 G91 G92 G21 G94 G17 G54, M0-M9/M30
//      (sin efecto). NO soporta: G20, arcos, compensaciones, spindle real.
// ============================================================================
struct GWord { char l; float v; };

static bool isRealtimeChar(uint8_t c) { return c == '?' || c == '!' || c == '~' || c == 0x18 || c == 0x85; }

static void grblSend(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void grblSend(const char* fmt, ...) {
  char b[160];
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  if (n > 0 && (size_t)n < sizeof(b)) netEnqueue(b, (size_t)n, false);
}

static void grblReply(int err) {
  if (err == 0) grblSend("ok\r\n");
  else grblSend("error:%d\r\n", err);
}

static int grblErrFromRc(Rc rc) {
  switch (rc) {
    case RC_OK:         return 0;
    case RC_BAD_ARGS:   return 2;
    case RC_SOFT_LIMIT: return 15;
    case RC_BUSY:       return 8;
    case RC_NOT_HOMED:
    case RC_ALARM:
    case RC_DISABLED:   return 9;
    default:            return 20;
  }
}

static void grblStatusReport() {
  const uint8_t ms = currentState();
  const char* st = "Idle";
  if (ms == MS_ALARM) st = "Alarm";
  else if (ms == MS_HOMING) st = "Home";
  else if (ms == MS_MANUAL) st = "Jog";
  else if (ms == MS_RUNNING) st = "Run";
  const int depth = (int)qCount + (blockActive ? 1 : 0);
  float w[AXIS_COUNT];
  for (uint8_t a = 0; a < AXIS_COUNT; a++) w[a] = (float)(rt[a].steps - rt[a].woff) / cfg[a].stepsPerMm;
  grblSend("<%s|WPos:%.3f,%.3f,%.3f,%.3f|Bf:%d,127|FS:0,0>\r\n", st, (double)w[0], (double)w[1], (double)w[2],
           (double)w[3], (int)QCAP - depth);
}

static void grblBanner() { grblSend("\r\nGrbl 1.1h-CNCXYZW-%s ['$' for help]\r\n", FW_VERSION); }

static void handleRealtime(uint8_t c) {
  switch (c) {
    case '?':  grblStatusReport(); break;
    case '!':  apiStop(); break;                // parada controlada + vaciado de cola
    case '~':  break;                           // reanudar: sin efecto (no existe pausa)
    case 0x85: apiStop(); break;                // cancelar jog
    case 0x18:
      apiStop();
      grblRel = false;
      grblMotionG1 = false;
      grblFeedMmMin = 0.0f;
      grblBanner();
      break;
    default: break;
  }
}

// Divide el bloque en palabras letra+número. Devuelve el nº de palabras, o -1 (error:1) / -2 (error:2) / -3 (error:20).
static int grblWords(char* s, GWord* w, int maxW) {
  int n = 0;
  while (*s) {
    char c = *s;
    if (c == ' ' || c == '\t' || c == '\r') { s++; continue; }
    if (c == ';') break;
    if (c == '(') { while (*s && *s != ')') s++; if (*s == ')') s++; continue; }
    c = (char)toupper((unsigned char)c);
    if (c < 'A' || c > 'Z') return -1;
    s++;
    char* e = nullptr;
    const double v = strtod(s, &e);
    if (e == s || !isfinite(v)) return -2;
    s = e;
    if (n >= maxW) return -3;
    w[n].l = c;
    w[n].v = (float)v;
    n++;
  }
  return n;
}

// 0 = ok, >0 = error GRBL, -1 = retener la línea (cola llena)
static int grblRunWords(const GWord* w, int nw, bool jog) {
  bool rel = grblRel;
  bool g1 = grblMotionG1;
  float feed = grblFeedMmMin;
  float t[AXIS_COUNT] = {NAN, NAN, NAN, NAN};
  bool hasAxis = false, g4 = false, g92 = false;
  float pSec = 0.0f;

  for (int i = 0; i < nw; i++) {
    const float v = w[i].v;
    switch (w[i].l) {
      case 'G': {
        const long g = lroundf(v * 10.0f);
        switch (g) {
          case 0:   g1 = false; break;
          case 10:  g1 = true; break;
          case 40:  g4 = true; break;
          case 170: case 210: case 540: case 940: break;
          case 900: rel = false; break;
          case 910: rel = true; break;
          case 920: g92 = true; break;
          default:  return 20;
        }
        break;
      }
      case 'M': {
        const long m = lroundf(v);
        if (!(m == 0 || m == 1 || m == 2 || m == 3 || m == 4 || m == 5 || m == 7 || m == 8 || m == 9 || m == 30)) return 20;
        break;
      }
      case 'X': t[AXIS_X] = v; hasAxis = true; break;
      case 'Y': t[AXIS_Y] = v; hasAxis = true; break;
      case 'Z': t[AXIS_Z] = v; hasAxis = true; break;
      case 'A':
      case 'W': t[AXIS_W] = v; hasAxis = true; break;
      case 'F': if (!(v > 0.0f)) return 2; feed = v; break;
      case 'P': pSec = v; break;
      case 'S': case 'T': case 'N': break;
      default:  return 20;
    }
  }
  if (!jog) { grblRel = rel; grblMotionG1 = g1; grblFeedMmMin = feed; }

  if (g4) {
    if (!(pSec >= 0.0f) || pSec > 600.0f) return 2;
    const Rc rc = planDwell((uint32_t)(pSec * 1000.0f + 0.5f), assignId(0));
    return rc == RC_FULL ? (jog ? 8 : -1) : grblErrFromRc(rc);
  }
  if (g92) {
    if (!hasAxis) return 20;
    for (uint8_t a = 0; a < AXIS_COUNT; a++) {
      if (isnan(t[a])) continue;
      const Rc rc = apiSetZero(a, t[a]);
      if (rc != RC_OK) return grblErrFromRc(rc);
    }
    return 0;
  }
  if (!hasAxis) return 0;
  if ((jog || g1) && feed <= 0.0f) return 22;
  const float feedMmS = (jog || g1) ? feed / 60.0f : 0.0f;       // G0 = velocidades máximas de cada eje
  int8_t bad;
  const Rc rc = planMove(t, rel, feedMmS, assignId(0), &bad);
  if (rc == RC_FULL) return jog ? 8 : -1;
  return grblErrFromRc(rc);
}

static int grblSystem(const char* line) {
  if (!strcmp(line, "$H")) {
    for (uint8_t a = 0; a < AXIS_COUNT; a++) {
      const Rc rc = planHome(a, assignId(0));
      if (rc != RC_OK) return grblErrFromRc(rc);
    }
    return 0;
  }
  if (!strcmp(line, "$X")) return grblErrFromRc(apiClearAlarm());
  if (!strcmp(line, "$$")) {
    for (uint8_t a = 0; a < AXIS_COUNT; a++) grblSend("$%d=%.3f\r\n", 100 + a, (double)cfg[a].stepsPerMm);
    for (uint8_t a = 0; a < AXIS_COUNT; a++) grblSend("$%d=%.3f\r\n", 110 + a, (double)(cfg[a].cruiseMmS * 60.0f));
    for (uint8_t a = 0; a < AXIS_COUNT; a++) grblSend("$%d=%.3f\r\n", 120 + a, (double)cfg[a].accelMmS2);
    for (uint8_t a = 0; a < AXIS_COUNT; a++) grblSend("$%d=%.3f\r\n", 130 + a, (double)cfg[a].maxTravelMm);
    return 0;
  }
  if (!strcmp(line, "$I")) { grblSend("[VER:1.1h.CNCXYZW-%s:]\r\n[OPT:V,%d,127]\r\n", FW_VERSION, (int)QCAP); return 0; }
  if (!strcmp(line, "$G")) { grblSend("[GC:G0 G54 G17 G21 %s G94 M5 M9 T0 F%.0f S0]\r\n", grblRel ? "G91" : "G90", (double)grblFeedMmMin); return 0; }
  if (!strncmp(line, "$J=", 3)) {
    char buf[MAX_LINE_LEN];
    strncpy(buf, line + 3, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    GWord w[16];
    const int n = grblWords(buf, w, 16);
    if (n == -1) return 1;
    if (n == -2) return 2;
    if (n < 0) return 20;
    if (machineBusy()) return 8;                  // $J= solo en reposo (como GRBL)
    return grblRunWords(w, n, true);
  }
  return 3;
}

// Devuelve 0 ok, >0 error, -1 retener.
static int handleGrblLine(const char* line) {
  ses.grbl = true;
  if (!authorizedNow()) { replyNack("grbl", RC_TEXT[RC_AUTH]); return -2; }       // -2 = ya respondido
  if (line[0] == '$') return grblSystem(line);
  char buf[MAX_LINE_LEN];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  GWord w[16];
  const int n = grblWords(buf, w, 16);
  if (n == -1) return 1;
  if (n == -2) return 2;
  if (n < 0) return 20;
  return grblRunWords(w, n, false);
}

// ============================================================================
//  15. RECEPCIÓN DE RED
// ============================================================================
static bool isNetLogLine(const char* l) {
  static const char* const K[] = {"AUTH", "PING", "GET_STATUS", "GET_CONFIG", "CMD"};
  for (size_t i = 0; i < sizeof(K) / sizeof(K[0]); i++) {
    const size_t n = strlen(K[i]);
    if (!strncmp(l, K[i], n) && (l[n] == 0 || l[n] == '|')) return true;
  }
  return false;
}

// Si es GRBL y la cola está llena, la línea queda retenida (grblHold) y se reintenta en cada vuelta.
static void processLine(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;
  if (authorizedNow()) lastLinkMs = nowMs();
  if (isNetLogLine(line)) { handleNetLine(line); return; }
  const int r = handleGrblLine(line);
  if (r == -1) { grblHold = true; return; }
  if (r != -2) grblReply(r);
}

static void onNetByte(uint8_t c) {
  // Caracteres de tiempo real GRBL: al inicio de línea siempre; en cualquier posición si la sesión ya habló GRBL
  if (isRealtimeChar(c) && !ses.rxOverflow && (ses.rxLen == 0 || ses.grbl)) {
    if (authorizedNow()) { ses.grbl = true; lastLinkMs = nowMs(); handleRealtime(c); }
    return;
  }
  if (c == '\r') return;
  if (c == '\n') {
    if (ses.rxOverflow) {
      gNackDetail[0] = 0;
      replyNack("line", RC_TEXT[RC_TOO_LONG]);
      ses.rxOverflow = false;
      ses.rxLen = 0;
      return;
    }
    ses.rx[ses.rxLen] = 0;
    processLine(ses.rx);
    if (!grblHold) ses.rxLen = 0;
    return;
  }
  if (ses.rxOverflow) return;
  if (ses.rxLen < MAX_LINE_LEN - 1) ses.rx[ses.rxLen++] = (char)c;
  else ses.rxOverflow = true;
}

static void netPoll() {
  if (!ses.active) return;
  if (grblHold) {
    const int r = handleGrblLine(ses.rx);
    if (r == -1) {
      // Con la cola llena solo se atienden caracteres de tiempo real que sean el siguiente byte
      while (ses.active && cl.available() > 0) {
        const int c = cl.peek();
        if (c < 0 || !isRealtimeChar((uint8_t)c)) break;
        cl.read();
        if (authorizedNow()) { lastLinkMs = nowMs(); handleRealtime((uint8_t)c); }
      }
      return;
    }
    grblHold = false;
    ses.rxLen = 0;
    if (r != -2) grblReply(r);
  }
  int guard = 0;
  while (ses.active && !grblHold && guard++ < 400) {
    const int c = cl.read();
    if (c < 0) break;
    ses.lastRxMs = nowMs();
    onNetByte((uint8_t)c);
  }
  if (ses.active && authorizedNow() && guard > 1) lastLinkMs = nowMs();
}

// ============================================================================
//  16. MANTENIMIENTO POR USB (115200, líneas '\n')
//      Los secretos son de SOLO ESCRITURA: ningún comando devuelve la clave
//      WiFi ni el token. Los campos con texto libre viajan codificados (%XX).
// ============================================================================
static void serAck(const char* cmd) { Serial.printf("ACK|%s|OK\n", cmd); }
static void serNack(const char* cmd, const char* why) { Serial.printf("NACK|%s|%s\n", cmd, why); }

static void serialDumpNvs() {
  Serial.print("NVS_DATA|{");
  Serial.printf("\"fw\":\"%s\",\"proto\":%d,", FW_VERSION, PROTO_VERSION);
  Serial.print("\"current_ip\":\"");
  jsonEscapeTo(Serial, WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "");
  Serial.print("\",\"ssid\":\"");
  jsonEscapeTo(Serial, wifiSsid);
  Serial.printf("\",\"pass_set\":%s,\"devname\":\"", wifiPass[0] ? "true" : "false");
  jsonEscapeTo(Serial, wifiDevName);
  Serial.printf("\",\"ip_oct4\":%u,\"wd_ms\":%lu,\"token_state\":%d,\"axes\":{", (unsigned)ipOct4,
                (unsigned long)linkWdMs, (int)tokenState);
  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    const AxisCfg& c = cfg[a];
    Serial.printf("%s\"%c\":{\"spm\":%.4f,\"max\":%.3f,\"bo\":%lu,\"so\":%lu,\"sc\":%d,\"seek\":%ld,\"feed\":%ld,"
                  "\"bo_us\":%ld,\"man_us\":%ld,\"jog_us\":%ld,\"dfl\":%d,\"start\":%.3f,\"cruise\":%.3f,"
                  "\"end\":%.3f,\"accel\":%.2f,\"cal\":%d,\"lcal\":\"",
                  a ? "," : "", AXIS_CHAR[a], (double)c.stepsPerMm, (double)c.maxTravelMm,
                  (unsigned long)c.backoffSteps, (unsigned long)c.softOffsetSteps, c.useSCurve ? 1 : 0,
                  (long)c.seekUs, (long)c.feedUs, (long)c.homeBoUs, (long)c.manualUs, (long)c.jogUs,
                  c.dirFwdLevel == HIGH ? 1 : 0, (double)c.startMmS, (double)c.cruiseMmS, (double)c.endMmS,
                  (double)c.accelMmS2, c.calibrated ? 1 : 0);
    jsonEscapeTo(Serial, c.lastCal);
    Serial.print("\"}");
  }
  Serial.print("}}\n");
}

static void serialScanWifi() {
  if (machineBusy()) { serNack("SCAN_WIFI", RC_TEXT[RC_BUSY]); return; }
  const int n = WiFi.scanNetworks();
  Serial.print("WIFI_LIST|");
  for (int i = 0; i < n; i++) {
    if (i) Serial.print(',');
    urlEncodeTo(Serial, WiFi.SSID(i).c_str());
  }
  Serial.print('\n');
  WiFi.scanDelete();
}

static bool printableToken(const char* s) {
  for (; *s; s++) if ((uint8_t)*s < 0x21 || (uint8_t)*s > 0x7E) return false;
  return true;
}

static void serialSetWifi(int argc, char** argv) {
  if (argc != 5) { serNack("SET_WIFI_NVS", RC_TEXT[RC_BAD_ARGS]); return; }
  if (machineBusy()) { serNack("SET_WIFI_NVS", RC_TEXT[RC_BUSY]); return; }
  for (int i = 1; i <= 3; i++) urlDecodeInPlace(argv[i]);
  const size_t ls = strlen(argv[1]), lp = strlen(argv[2]), ld = strlen(argv[3]);
  long ip4;
  if (ls < 1 || ls > 32 || ld < 1 || ld > 32 || !parseLongStrict(argv[4], ip4)) { serNack("SET_WIFI_NVS", RC_TEXT[RC_BAD_ARGS]); return; }
  if (!(ip4 == 0 || (ip4 >= 2 && ip4 <= 254))) { serNack("SET_WIFI_NVS", "bad_ip"); return; }
  const bool clearPass = (!strcmp(argv[2], "-"));
  if (!clearPass && lp != 0 && (lp < 8 || lp > 63)) { serNack("SET_WIFI_NVS", "bad_pass_len"); return; }
  for (const char* c = argv[3]; *c; c++) {
    if (!(isalnum((unsigned char)*c) || *c == '-')) { serNack("SET_WIFI_NVS", "bad_devname"); return; }
  }
  strncpy(wifiSsid, argv[1], sizeof(wifiSsid) - 1);
  wifiSsid[sizeof(wifiSsid) - 1] = 0;
  if (clearPass) wifiPass[0] = 0;
  else if (lp > 0) { strncpy(wifiPass, argv[2], sizeof(wifiPass) - 1); wifiPass[sizeof(wifiPass) - 1] = 0; }
  strncpy(wifiDevName, argv[3], sizeof(wifiDevName) - 1);
  wifiDevName[sizeof(wifiDevName) - 1] = 0;
  ipOct4 = (uint8_t)ip4;
  saveWifiConfig();
  serAck("SET_WIFI_NVS");
}

static void serialLoadAxis(int argc, char** argv) {
  if (argc != 13 && argc != 17) { serNack("LOAD_AXIS_PARAM", RC_TEXT[RC_BAD_ARGS]); return; }
  if (machineBusy()) { serNack("LOAD_AXIS_PARAM", RC_TEXT[RC_BUSY]); return; }
  uint8_t a;
  if (!parseAxisTok(argv[1], a)) { serNack("LOAD_AXIS_PARAM", RC_TEXT[RC_BAD_AXIS]); return; }
  AxisCfg t = cfg[a];
  float f[4] = {0, 0, 0, 0};
  long l[8];
  if (!parseFloatStrict(argv[2], t.stepsPerMm) || !parseFloatStrict(argv[3], t.maxTravelMm)) { serNack("LOAD_AXIS_PARAM", RC_TEXT[RC_BAD_ARGS]); return; }
  for (int i = 0; i < 9; i++) {
    long v;
    if (!parseLongStrict(argv[4 + i], v)) { serNack("LOAD_AXIS_PARAM", RC_TEXT[RC_BAD_ARGS]); return; }
    if (i < 8) l[i] = v;
    else if (v != 0 && v != 1) { serNack("LOAD_AXIS_PARAM", RC_TEXT[RC_BAD_ARGS]); return; }
    else t.dirFwdLevel = v ? HIGH : LOW;
  }
  // l: bo, so, sc, seek, feed, bo_us, man_us, jog_us
  if (argc == 17) {
    for (int i = 0; i < 4; i++) if (!parseFloatStrict(argv[13 + i], f[i])) { serNack("LOAD_AXIS_PARAM", RC_TEXT[RC_BAD_ARGS]); return; }
    t.startMmS = f[0]; t.cruiseMmS = f[1]; t.endMmS = f[2]; t.accelMmS2 = f[3];
  }
  const bool ok = inRangeF(t.stepsPerMm, 1.0f, 100000.0f) && inRangeF(t.maxTravelMm, 1.0f, 2000.0f) &&
                  l[0] >= 0 && l[0] <= 1000000L && l[1] >= 0 && l[1] <= 1000000L && (l[2] == 0 || l[2] == 1) &&
                  l[3] >= (long)MIN_STEP_US && l[3] <= 200000L && l[4] >= (long)MIN_STEP_US && l[4] <= 200000L &&
                  l[5] >= (long)MIN_STEP_US && l[5] <= 200000L && l[6] >= (long)MIN_STEP_US && l[6] <= 200000L &&
                  l[7] >= (long)MIN_STEP_US && l[7] <= 200000L &&
                  inRangeF(t.startMmS, 0.05f, 500.0f) && inRangeF(t.cruiseMmS, 0.1f, 1000.0f) &&
                  inRangeF(t.endMmS, 0.05f, 500.0f) && inRangeF(t.accelMmS2, 1.0f, 20000.0f) &&
                  t.cruiseMmS >= t.startMmS && t.cruiseMmS >= t.endMmS;
  if (!ok) { serNack("LOAD_AXIS_PARAM", RC_TEXT[RC_BAD_ARGS]); return; }
  t.backoffSteps = (uint32_t)l[0];
  t.softOffsetSteps = (uint32_t)l[1];
  t.useSCurve = (uint8_t)l[2];
  t.seekUs = (int32_t)l[3];
  t.feedUs = (int32_t)l[4];
  t.homeBoUs = (int32_t)l[5];
  t.manualUs = (int32_t)l[6];
  t.jogUs = (int32_t)l[7];
  if (t.dirFwdLevel != cfg[a].dirFwdLevel) rt[a].homed = 0;
  cfg[a] = t;
  saveConfigIfDirty();
  serAck("LOAD_AXIS_PARAM");
}

static void handleSerialLine(char* line) {
  char* argv[20];
  const int argc = splitFields(line, argv, 20);
  if (argc < 0) { serNack("line", "too_many_fields"); return; }
  const char* cmd = argv[0];
  if (!cmd[0]) return;

  if (!strcmp(cmd, "ESTOP")) { apiEstop(); serAck("ESTOP"); }
  else if (!strcmp(cmd, "VERSION")) { Serial.printf("VERSION|CNC-XYZW|%s|%d\n", FW_VERSION, PROTO_VERSION); }
  else if (!strcmp(cmd, "STATUS")) { char b[320]; OutBuf o(b, sizeof(b)); buildStatus(o); if (!o.ovf) Serial.print(b); }
  else if (!strcmp(cmd, "SCAN_WIFI")) serialScanWifi();
  else if (!strcmp(cmd, "DUMP_NVS")) serialDumpNvs();
  else if (!strcmp(cmd, "SET_WIFI_NVS")) serialSetWifi(argc, argv);
  else if (!strcmp(cmd, "LOAD_AXIS_PARAM")) serialLoadAxis(argc, argv);
  else if (!strcmp(cmd, "SET_TOKEN")) {
    if (argc != 2) { serNack("SET_TOKEN", RC_TEXT[RC_BAD_ARGS]); return; }
    urlDecodeInPlace(argv[1]);
    const size_t n = strlen(argv[1]);
    if (n < TOKEN_MIN_LEN || n > TOKEN_MAX_LEN || !printableToken(argv[1])) { serNack("SET_TOKEN", "bad_token"); return; }
    strncpy(token, argv[1], sizeof(token) - 1);
    token[sizeof(token) - 1] = 0;
    tokenState = TOK_SET;
    saveToken();
    authFailCount = 0;
    authLocked = false;
    sessionClose();
    serAck("SET_TOKEN");
  }
  else if (!strcmp(cmd, "SET_OPEN_MODE")) {
    if (argc != 2 || strcmp(argv[1], "CONFIRM") != 0) { serNack("SET_OPEN_MODE", "confirm_required"); return; }
    token[0] = 0;
    tokenState = TOK_OPEN;
    saveToken();
    sessionClose();
    serAck("SET_OPEN_MODE");
  }
  else if (!strcmp(cmd, "FACTORY_RESET")) {
    if (argc != 2 || strcmp(argv[1], "CONFIRM") != 0) { serNack("FACTORY_RESET", "confirm_required"); return; }
    if (machineBusy()) { serNack("FACTORY_RESET", RC_TEXT[RC_BUSY]); return; }
    if (prefs.begin("cnc_xyzw", false)) { prefs.clear(); prefs.end(); }
    serAck("FACTORY_RESET");
    delay(200);
    ESP.restart();
  }
  else if (!strcmp(cmd, "RESTART_ESP")) {
    if (machineBusy()) { serNack("RESTART_ESP", RC_TEXT[RC_BUSY]); return; }
    serAck("RESTART_ESP");
    delay(200);
    ESP.restart();
  }
  else { char shown[28]; sanitizeText(shown, sizeof(shown), cmd); serNack(shown, RC_TEXT[RC_UNKNOWN]); }
}

static void serialPoll() {
  int guard = 0;
  while (Serial.available() > 0 && guard++ < 256) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      if (serialOverflow) {
        serNack("line", RC_TEXT[RC_TOO_LONG]);
        serialOverflow = false;
        serialLen = 0;
        continue;
      }
      serialBuf[serialLen] = 0;
      handleSerialLine(serialBuf);
      serialLen = 0;
      continue;
    }
    if (serialOverflow) continue;
    if (serialLen < sizeof(serialBuf) - 1) serialBuf[serialLen++] = (char)c;
    else serialOverflow = true;
  }
}

// ============================================================================
//  17. WIFI NO BLOQUEANTE Y ACEPTACIÓN DE CLIENTES
// ============================================================================
static void wifiBeginNow() {
  if (!wifiSsid[0]) return;
  WiFi.begin(wifiSsid, wifiPass[0] ? wifiPass : nullptr);
  lastWifiAttemptMs = nowMs();
}

static void wifiStart() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                      // sin ahorro de energía: menor latencia
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(wifiDevName);
  if (ipOct4 >= 2 && ipOct4 <= 254) {
    WiFi.config(IPAddress(192, 168, 1, ipOct4), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0),
                IPAddress(8, 8, 8, 8), IPAddress(1, 1, 1, 1));
  }
  wifiBeginNow();
}

static void wifiManage(uint32_t now) {
  const wl_status_t st = WiFi.status();
  if (st != lastWifiStatus) {
    lastWifiStatus = st;
    if (st == WL_CONNECTED) {
      Serial.printf("WIFI|connected|%s\n", WiFi.localIP().toString().c_str());
      if (serverStarted) server.end();
      server.begin();
      server.setNoDelay(true);
      serverStarted = true;
    } else {
      Serial.printf("WIFI|status|%d\n", (int)st);
    }
  }
  if (st != WL_CONNECTED && wifiSsid[0] && (uint32_t)(now - lastWifiAttemptMs) > 10000) {
    WiFi.disconnect(false, false);
    wifiBeginNow();
  }
}

static void netAccept(uint32_t now) {
  if (authLocked && (int32_t)(now - authLockUntilMs) >= 0) { authLocked = false; authFailCount = 0; }
  if (!serverStarted || WiFi.status() != WL_CONNECTED) return;
  WiFiClient nc = server.accept();
  if (!nc) return;
  if (authLocked) { nc.print("NACK|AUTH|locked\n"); nc.stop(); return; }
  if (ses.active) {
    const bool stale = (uint32_t)(now - ses.lastRxMs) > STALE_CLIENT_MS;
    if (!(stale || !authorizedNow())) { nc.print("NACK|connect|busy\n"); nc.stop(); return; }
    sessionClose();
  }
  sessionOpen(nc);
}

static uint8_t grblAlarmNumber(uint8_t code) {
  switch (code) {
    case AL_HARD_LIMIT: return 1;
    case AL_HOME_FAIL:  return 9;
    default:            return 3;
  }
}

static void telemetryTick(uint32_t now) {
  if (!authorizedNow()) return;
  if (alarmEventPending) {
    alarmEventPending = false;
    if (ses.netlog) sendAlarmEvent();
    if (ses.grbl) grblSend("ALARM:%u\r\n", (unsigned)grblAlarmNumber(alarmCode));
  }
  if (!ses.netlog) return;
  const uint32_t since = (uint32_t)(now - ses.lastTelemetryMs);
  if (since >= TELEMETRY_MS || (forceStatusPush && since >= 5)) {
    forceStatusPush = false;
    ses.lastTelemetryMs = now;
    sendStatus(true);
  }
}

// ============================================================================
//  18. SETUP / LOOP
// ============================================================================
void setup() {
  // 1) Drivers deshabilitados ANTES de cualquier otra cosa (el pin flota durante el arranque:
  //    ver recomendación de pull-up externo en la documentación).
  digitalWrite(PIN_ENABLE_ACTUATORS, HIGH);
  pinMode(PIN_ENABLE_ACTUATORS, OUTPUT);
  Serial.begin(115200);

  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    stepMask[a] = 1UL << PIN_STEP[a];
    digitalWrite(PIN_STEP[a], LOW);
    pinMode(PIN_STEP[a], OUTPUT);
    digitalWrite(PIN_DIR[a], LOW);
    pinMode(PIN_DIR[a], OUTPUT);
    pinMode(PIN_LIMIT[a], PIN_LIMIT[a] >= 34 ? INPUT : INPUT_PULLUP);   // GPIO34-39: solo entrada, sin pull-up interno
  }
#if PIN_ESTOP_INPUT >= 0
  pinMode(PIN_ESTOP_INPUT, INPUT_PULLUP);
#endif

  loadConfig();
  for (uint8_t a = 0; a < AXIS_COUNT; a++) {
    rt[a].steps = 0; rt[a].woff = 0; rt[a].target = 0; rt[a].homed = 0;
    rt[a].moving = 0; rt[a].moveDir = 0; rt[a].err = AE_NONE; rt[a].limit = limitRaw(a) ? 1 : 0;
    planned[a] = 0;
    plannedHomed[a] = false;
    setDirPin(a, true);
  }

  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);
  xTaskCreatePinnedToCore(taskMotors, "TaskMotors", 6144, nullptr, 5, &taskMotorsHandle, 1);

  wifiStart();
  Serial.printf("BOOT|CNC-XYZW|%s|%d|token=%d|wifi=%s\n", FW_VERSION, PROTO_VERSION, (int)tokenState,
                wifiSsid[0] ? "configured" : "not_configured");
}

void loop() {
  const uint32_t now = nowMs();
  serialPoll();
  wifiManage(now);
  netAccept(now);
  if (ses.active) {
    netPoll();
    static uint32_t lastPeerCheck = 0;
    if ((uint32_t)(now - lastPeerCheck) >= 20) {
      lastPeerCheck = now;
      if (ses.active && !peerAlive()) sessionClose();
    }
  }
  sessionAuthorized = authorizedNow();

  for (uint8_t a = 0; a < AXIS_COUNT; a++) rt[a].limit = limitRaw(a) ? 1 : 0;
  if (physEstopActive() && alarmCode != AL_ESTOP && alarmCode != AL_PHYS_ESTOP) raiseAlarm(AL_PHYS_ESTOP, -1, true);

  telemetryTick(now);
  netFlush();

  if (cfgDirty && !machineBusy() && (uint32_t)(now - cfgDirtyAtMs) >= 300) saveConfigIfDirty();
  delay(1);
}
