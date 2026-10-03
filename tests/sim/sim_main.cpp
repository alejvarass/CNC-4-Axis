// ============================================================================
//  Simulador de host del firmware CNC_V2.ino
//  - Compila el .ino TAL CUAL (no se copia ni se modifica) sobre un entorno
//    simulado (ver mock/Arduino.h) y una máquina virtual de 4 ejes:
//      * cuenta pasos reales a partir de los pulsos STEP/DIR/ENABLE;
//      * final de carrera NC en el extremo negativo (activo = ALTO);
//      * topes mecánicos (cuenta "choques");
//      * registra el tiempo mínimo DIR->STEP (debe ser >= 20 us).
//  - Control por stdin (una orden por línea), respuestas por stdout:
//      "CTL|..." (control) y "S|..." (salida del puerto serie USB).
//  - El TCP del firmware es REAL (sockets POSIX, puerto SIM_PORT o 5000).
// ============================================================================
#include "mock/Arduino.h"
#include <sys/prctl.h>

#ifndef PIN_ESTOP_INPUT
#define PIN_ESTOP_INPUT 33
#endif

// ---------- objetos globales del entorno simulado ----------
SimTask* sim_motor_task = nullptr;
void (*sim_timer_isr)() = nullptr;
std::atomic<int64_t> sim_timer_deadline{0};
std::atomic<int64_t> sim_timer_base{0};
std::recursive_mutex& sim_big_lock() { static std::recursive_mutex m; return m; }
SimSerial Serial;
SimWiFi WiFi;
SimEsp ESP;
static std::mutex g_out;

static void outLine(const char* tag, const std::string& s) {
  std::lock_guard<std::mutex> g(g_out);
  fprintf(stdout, "%s|%s\n", tag, s.c_str());
  fflush(stdout);
}

size_t SimSerial::write(uint8_t c) {
  if (c == '\n') { outLine("S", line); line.clear(); }
  else if (c != '\r') line.push_back((char)c);
  return 1;
}

// ---------- NVS en archivo ----------
std::map<std::string, std::string>& Preferences::store() { static std::map<std::string, std::string> m; return m; }
static bool g_nvsLoaded = false;
void Preferences::load() {
  if (g_nvsLoaded) return;
  g_nvsLoaded = true;
  const char* path = getenv("SIM_NVS");
  if (!path) return;
  FILE* f = fopen(path, "r");
  if (!f) return;
  char line[600];
  while (fgets(line, sizeof(line), f)) {
    char* tab = strchr(line, '\t');
    if (!tab) continue;
    *tab = 0;
    char* v = tab + 1;
    v[strcspn(v, "\n")] = 0;
    store()[line] = v;
  }
  fclose(f);
}
void Preferences::persist() {
  const char* path = getenv("SIM_NVS");
  if (!path) return;
  FILE* f = fopen(path, "w");
  if (!f) return;
  for (auto& kv : store()) fprintf(f, "%s\t%s\n", kv.first.c_str(), kv.second.c_str());
  fclose(f);
}

void WiFiServer::begin() {
  end();
  const char* envPort = getenv("SIM_PORT");
  const uint16_t port = envPort ? (uint16_t)atoi(envPort) : port_;
  lfd_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
  int one = 1;
  setsockopt(lfd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  a.sin_port = htons(port);
  if (bind(lfd_, (sockaddr*)&a, sizeof(a)) < 0 || listen(lfd_, 4) < 0) {
    outLine("CTL", std::string("ERROR bind/listen ") + strerror(errno));
    exit(3);
  }
}

void SimEsp::restart() {
  outLine("CTL", "RESTART");
  fflush(stdout);
  _exit(42);
}

// ---------- el firmware, sin modificar ----------
#include "../../CNC_V2.ino"

// ---------- máquina virtual ----------
struct Machine {
  std::atomic<int32_t> pos[4], switchAt[4], hardMin[4], hardMax[4];
  std::atomic<int> dirLevel[4], posLevel[4];
  std::atomic<bool> enableLow{false};
  std::atomic<uint64_t> pulses[4], pulsesPos[4], pulsesNeg[4], crash[4];
  std::atomic<uint64_t> lostDisabled{0};
  std::atomic<int64_t> lastDirChangeUs[4];
  std::atomic<bool> dirChanged[4];
  std::atomic<int64_t> minDirSetupUs{INT64_MAX};
  std::atomic<bool> stuck[4];
  std::atomic<bool> physEstop{false};
  std::atomic<int> pinLevel[40];
  std::atomic<int> maxEnableLowStepGapUs{0};
  Machine() {
    for (int a = 0; a < 4; a++) {
      pos[a] = 300; switchAt[a] = 100; hardMin[a] = 0; hardMax[a] = 100000;
      dirLevel[a] = LOW; posLevel[a] = HIGH; pulses[a] = pulsesPos[a] = pulsesNeg[a] = crash[a] = 0;
      lastDirChangeUs[a] = 0; dirChanged[a] = false; stuck[a] = false;
    }
    for (auto& p : pinLevel) p = 0;
  }
} M;

void sim_pin_mode(uint8_t, uint8_t) {}

void sim_digital_write(uint8_t pin, uint8_t level) {
  if (pin < 40) M.pinLevel[pin] = level;
  if (pin == PIN_ENABLE_ACTUATORS) M.enableLow = (level == LOW);
  for (int a = 0; a < 4; a++) {
    if (pin == PIN_DIR[a]) {
      if (M.dirLevel[a] != level) { M.dirLevel[a] = level; M.lastDirChangeUs[a] = esp_timer_get_time(); M.dirChanged[a] = true; }
    }
  }
}

uint8_t sim_digital_read(uint8_t pin) {
  for (int a = 0; a < 4; a++) {
    if (pin == PIN_LIMIT[a]) return (M.pos[a] <= M.switchAt[a] || M.stuck[a]) ? HIGH : LOW;
  }
#if PIN_ESTOP_INPUT >= 0
  if (pin == PIN_ESTOP_INPUT) return M.physEstop ? HIGH : LOW;
#endif
  return pin < 40 ? (uint8_t)M.pinLevel[pin].load() : 0;
}

static void doStep(int a) {
  if (!M.enableLow) { M.lostDisabled++; return; }
  if (M.dirChanged[a]) {
    const int64_t d = esp_timer_get_time() - M.lastDirChangeUs[a];
    int64_t cur = M.minDirSetupUs;
    while (d < cur && !M.minDirSetupUs.compare_exchange_weak(cur, d)) { }
    M.dirChanged[a] = false;
  }
  const int dir = (M.dirLevel[a] == M.posLevel[a]) ? 1 : -1;
  const int32_t np = M.pos[a] + dir;
  if (np < M.hardMin[a] || np > M.hardMax[a]) { M.crash[a]++; return; }
  M.pos[a] = np;
  M.pulses[a]++;
  if (dir > 0) M.pulsesPos[a]++; else M.pulsesNeg[a]++;
}

void sim_reg_write(uint32_t reg, uint32_t val) {
  if (reg != GPIO_OUT_W1TS_REG) return;
  for (int a = 0; a < 4; a++) if (val & (1u << PIN_STEP[a])) doStep(a);
}

// ---------- canal de control (stdin) ----------
static std::atomic<bool> g_quit{false};

static void controlThread() {
  char buf[1024];
  while (!g_quit && fgets(buf, sizeof(buf), stdin)) {
    buf[strcspn(buf, "\r\n")] = 0;
    char cmd[32] = {0};
    int a = 0;
    long v1 = 0, v2 = 0;
    if (sscanf(buf, "%31s", cmd) < 1) continue;
    if (!strcmp(cmd, "PHYS") && sscanf(buf, "%*s %d %ld", &a, &v1) == 2) { M.pos[a] = (int32_t)v1; outLine("CTL", "OK"); }
    else if (!strcmp(cmd, "SWITCH") && sscanf(buf, "%*s %d %ld", &a, &v1) == 2) { M.switchAt[a] = (int32_t)v1; outLine("CTL", "OK"); }
    else if (!strcmp(cmd, "HARD") && sscanf(buf, "%*s %d %ld %ld", &a, &v1, &v2) == 3) { M.hardMin[a] = (int32_t)v1; M.hardMax[a] = (int32_t)v2; outLine("CTL", "OK"); }
    else if (!strcmp(cmd, "STUCK") && sscanf(buf, "%*s %d %ld", &a, &v1) == 2) { M.stuck[a] = v1 != 0; outLine("CTL", "OK"); }
    else if (!strcmp(cmd, "WIRING") && sscanf(buf, "%*s %d %ld", &a, &v1) == 2) { M.posLevel[a] = (int)v1; outLine("CTL", "OK"); }
    else if (!strcmp(cmd, "ESTOP_PIN") && sscanf(buf, "%*s %ld", &v1) == 1) { M.physEstop = v1 != 0; outLine("CTL", "OK"); }
    else if (!strcmp(cmd, "RESETSTATS")) {
      for (int i = 0; i < 4; i++) { M.pulses[i] = M.pulsesPos[i] = M.pulsesNeg[i] = M.crash[i] = 0; }
      M.lostDisabled = 0; M.minDirSetupUs = INT64_MAX;
      outLine("CTL", "OK");
    }
    else if (!strcmp(cmd, "GET")) {
      char j[900];
      snprintf(j, sizeof(j),
               "{\"pos\":[%d,%d,%d,%d],\"pulses\":[%llu,%llu,%llu,%llu],\"crash\":[%llu,%llu,%llu,%llu],"
               "\"lost_disabled\":%llu,\"min_dir_setup_us\":%lld,\"enable_low\":%d,"
               "\"limit\":[%d,%d,%d,%d]}",
               (int)M.pos[0], (int)M.pos[1], (int)M.pos[2], (int)M.pos[3],
               (unsigned long long)M.pulses[0].load(), (unsigned long long)M.pulses[1].load(),
               (unsigned long long)M.pulses[2].load(), (unsigned long long)M.pulses[3].load(),
               (unsigned long long)M.crash[0].load(), (unsigned long long)M.crash[1].load(),
               (unsigned long long)M.crash[2].load(), (unsigned long long)M.crash[3].load(),
               (unsigned long long)M.lostDisabled.load(),
               (long long)(M.minDirSetupUs.load() == INT64_MAX ? -1 : M.minDirSetupUs.load()),
               M.enableLow ? 1 : 0,
               M.pos[0] <= M.switchAt[0], M.pos[1] <= M.switchAt[1], M.pos[2] <= M.switchAt[2], M.pos[3] <= M.switchAt[3]);
      outLine("CTL", j);
    }
    else if (!strcmp(cmd, "SERIAL")) { Serial.feed(strlen(buf) > 7 ? buf + 7 : ""); outLine("CTL", "OK"); }
    else if (!strcmp(cmd, "PING")) outLine("CTL", "PONG");
    else if (!strcmp(cmd, "QUIT")) { g_quit = true; outLine("CTL", "BYE"); fflush(stdout); _exit(0); }
    else outLine("CTL", std::string("ERR unknown ") + cmd);
  }
  // stdin cerrado: terminar
  _exit(0);
}

static void timerThread() {
  prctl(PR_SET_TIMERSLACK, 1UL);
  while (!g_quit) {
    const int64_t d = sim_timer_deadline.load();
    if (d != 0) {
      if (esp_timer_get_time() >= d) {
        int64_t exp = d;
        if (sim_timer_deadline.compare_exchange_strong(exp, 0) && sim_timer_isr) sim_timer_isr();
        continue;
      }
      struct timespec ts = {0, 20000};
      nanosleep(&ts, nullptr);
    } else {
      struct timespec ts = {0, 50000};
      nanosleep(&ts, nullptr);
    }
  }
}

int main() {
  signal(SIGPIPE, SIG_IGN);
  std::thread(timerThread).detach();
  std::thread(controlThread).detach();
  setup();
  outLine("CTL", "READY");
  for (;;) loop();
  return 0;
}
