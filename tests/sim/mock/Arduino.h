// ============================================================================
//  Entorno simulado (host Linux) para compilar CNC_V2.ino SIN hardware.
//  Reproduce solo lo que usa el firmware: Arduino/GPIO, FreeRTOS (hilos POSIX),
//  temporizador de hardware, Preferences (NVS en archivo), WiFi/TCP real
//  (sockets POSIX), mbedtls HMAC (OpenSSL) y Serial (stdin/stdout).
//  NO es un emulador del ESP32: valida la LÓGICA del firmware (protocolo,
//  planificador, seguridad), no los tiempos de silicio ni el WiFi real.
// ============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <map>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <thread>
#include <atomic>
#include <chrono>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>

#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// ---------------------------------------------------------------- GPIO ------
void    sim_digital_write(uint8_t pin, uint8_t level);
uint8_t sim_digital_read(uint8_t pin);
void    sim_reg_write(uint32_t reg, uint32_t val);
void    sim_pin_mode(uint8_t pin, uint8_t mode);
inline void digitalWrite(uint8_t pin, uint8_t level) { sim_digital_write(pin, level); }
inline int  digitalRead(uint8_t pin) { return sim_digital_read(pin); }
inline void pinMode(uint8_t pin, uint8_t mode) { sim_pin_mode(pin, mode); }
#define GPIO_OUT_W1TS_REG 1u
#define GPIO_OUT_W1TC_REG 2u
#define REG_WRITE(reg, val) sim_reg_write((reg), (val))

// ---------------------------------------------------------------- tiempo ----
inline int64_t esp_timer_get_time() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}
inline void delayMicroseconds(uint32_t us) {
  const int64_t end = esp_timer_get_time() + us;
  if (us > 300) usleep(us - 100);
  while (esp_timer_get_time() < end) { }
}
inline void delay(uint32_t ms) { usleep(ms * 1000u); }
inline void esp_fill_random(void* buf, size_t n) {
  FILE* f = fopen("/dev/urandom", "rb");
  if (f) { size_t r = fread(buf, 1, n, f); (void)r; fclose(f); }
}

// ---------------------------------------------------------------- FreeRTOS --
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portYIELD_FROM_ISR()

struct portMUX_TYPE { int dummy; };
#define portMUX_INITIALIZER_UNLOCKED {0}
std::recursive_mutex& sim_big_lock();
#define portENTER_CRITICAL(m) sim_big_lock().lock()
#define portEXIT_CRITICAL(m) sim_big_lock().unlock()

struct SimTask {
  std::mutex m;
  std::condition_variable cv;
  uint32_t count = 0;
};
typedef SimTask* TaskHandle_t;

inline void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t* woken) {
  { std::lock_guard<std::mutex> g(t->m); t->count++; }
  t->cv.notify_one();
  if (woken) *woken = pdFALSE;
}
inline uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, TickType_t ticks) {
  // Solo se usa desde la tarea de motores: se asocia al primer SimTask creado.
  extern SimTask* sim_motor_task;
  SimTask* t = sim_motor_task;
  std::unique_lock<std::mutex> lk(t->m);
  if (t->count == 0 && ticks > 0) t->cv.wait_for(lk, std::chrono::milliseconds(ticks), [&] { return t->count > 0; });
  const uint32_t c = t->count;
  if (c > 0) t->count = clearOnExit ? 0 : c - 1;
  return c;
}
inline void vTaskDelay(TickType_t ticks) { usleep(ticks * 1000u); }

extern SimTask* sim_motor_task;
inline int xTaskCreatePinnedToCore(void (*fn)(void*), const char*, uint32_t, void* arg, int, TaskHandle_t* out, int) {
  SimTask* t = new SimTask();
  sim_motor_task = t;
  if (out) *out = t;
  std::thread([fn, arg] { fn(arg); }).detach();
  return 1;
}

// Temporizador de hardware: un hilo dispara la ISR cuando vence la alarma
struct hw_timer_t { int dummy; };
extern void (*sim_timer_isr)();
extern std::atomic<int64_t> sim_timer_deadline;   // 0 = desarmado
extern std::atomic<int64_t> sim_timer_base;
inline hw_timer_t* timerBegin(uint32_t) { static hw_timer_t t; return &t; }
inline void timerAttachInterrupt(hw_timer_t*, void (*fn)()) { sim_timer_isr = fn; }
inline void timerWrite(hw_timer_t*, uint64_t) { sim_timer_base = esp_timer_get_time(); }
inline void timerAlarm(hw_timer_t*, uint64_t v, bool, uint64_t) { sim_timer_deadline = sim_timer_base.load() + (int64_t)v; }

// ---------------------------------------------------------------- Print/Serial
class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  size_t write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
  size_t print(const char* s) { size_t n = 0; while (*s) { write((uint8_t)*s++); n++; } return n; }
  size_t print(char c) { write((uint8_t)c); return 1; }
  size_t println(const char* s) { size_t n = print(s); write('\n'); return n + 1; }
  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    char b[1024];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n < 0) return 0;
    if ((size_t)n >= sizeof(b)) n = sizeof(b) - 1;
    for (int i = 0; i < n; i++) write((uint8_t)b[i]);
    return (size_t)n;
  }
};

struct SimSerial : public Print {
  std::mutex m;
  std::string line;
  std::deque<uint8_t> in;
  size_t write(uint8_t c) override;
  void begin(unsigned long) {}
  int available() { std::lock_guard<std::mutex> g(m); return (int)in.size(); }
  int read() { std::lock_guard<std::mutex> g(m); if (in.empty()) return -1; int c = in.front(); in.pop_front(); return c; }
  void feed(const std::string& s) { std::lock_guard<std::mutex> g(m); for (char c : s) in.push_back((uint8_t)c); in.push_back('\n'); }
};
extern SimSerial Serial;

// ---------------------------------------------------------------- String/IP --
class String {
public:
  std::string s;
  String() {}
  String(const char* c) : s(c) {}
  const char* c_str() const { return s.c_str(); }
};

class IPAddress {
public:
  uint8_t o[4] = {0, 0, 0, 0};
  IPAddress() {}
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) { o[0] = a; o[1] = b; o[2] = c; o[3] = d; }
  String toString() const { char b[20]; snprintf(b, sizeof(b), "%u.%u.%u.%u", o[0], o[1], o[2], o[3]); return String(b); }
};

// ---------------------------------------------------------------- Preferences
class Preferences {
  std::string ns_;
  bool open_ = false;
  static std::map<std::string, std::string>& store();
  static void persist();
  static void load();
  std::string K(const char* k) const { return ns_ + "/" + k; }
public:
  bool begin(const char* ns, bool = false) { load(); ns_ = ns; open_ = true; return true; }
  void end() { open_ = false; }
  bool isKey(const char* k) { return store().count(K(k)) > 0; }
  void remove(const char* k) { store().erase(K(k)); persist(); }
  void clear() {
    for (auto it = store().begin(); it != store().end();) {
      if (it->first.rfind(ns_ + "/", 0) == 0) it = store().erase(it); else ++it;
    }
    persist();
  }
  std::string get(const char* k, const std::string& d) { auto it = store().find(K(k)); return it == store().end() ? d : it->second; }
  void put(const char* k, const std::string& v) { store()[K(k)] = v; persist(); }
  uint8_t getUChar(const char* k, uint8_t d) { return (uint8_t)atoi(get(k, std::to_string(d)).c_str()); }
  uint32_t getUInt(const char* k, uint32_t d) { return (uint32_t)strtoul(get(k, std::to_string(d)).c_str(), nullptr, 10); }
  int32_t getInt(const char* k, int32_t d) { return (int32_t)atoi(get(k, std::to_string(d)).c_str()); }
  bool getBool(const char* k, bool d) { return atoi(get(k, d ? "1" : "0").c_str()) != 0; }
  float getFloat(const char* k, float d) { char b[40]; snprintf(b, sizeof(b), "%.9g", (double)d); return (float)atof(get(k, b).c_str()); }
  size_t getString(const char* k, char* out, size_t max) {
    std::string v = get(k, out);
    snprintf(out, max, "%s", v.c_str());
    return strlen(out);
  }
  size_t putUChar(const char* k, uint8_t v) { put(k, std::to_string(v)); return 1; }
  size_t putUInt(const char* k, uint32_t v) { put(k, std::to_string(v)); return 4; }
  size_t putInt(const char* k, int32_t v) { put(k, std::to_string(v)); return 4; }
  size_t putBool(const char* k, bool v) { put(k, v ? "1" : "0"); return 1; }
  size_t putFloat(const char* k, float v) { char b[40]; snprintf(b, sizeof(b), "%.9g", (double)v); put(k, b); return 4; }
  size_t putString(const char* k, const char* v) { put(k, v); return strlen(v); }
};

// ---------------------------------------------------------------- WiFi/TCP ---
enum wl_status_t { WL_NO_SHIELD = 255, WL_IDLE_STATUS = 0, WL_NO_SSID_AVAIL = 1, WL_SCAN_COMPLETED = 2,
                   WL_CONNECTED = 3, WL_CONNECT_FAILED = 4, WL_CONNECTION_LOST = 5, WL_DISCONNECTED = 6 };
#define WIFI_STA 1

class WiFiClient {
  int fd_ = -1;
public:
  WiFiClient() {}
  explicit WiFiClient(int fd) : fd_(fd) {}
  explicit operator bool() const { return fd_ >= 0; }
  int fd() const { return fd_; }
  void stop() { if (fd_ >= 0) { close(fd_); fd_ = -1; } }
  int setNoDelay(bool on) { int v = on ? 1 : 0; return setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &v, sizeof(v)); }
  int available() { if (fd_ < 0) return 0; int c = 0; if (ioctl(fd_, FIONREAD, &c) < 0) return 0; return c; }
  int read() { if (fd_ < 0) return -1; uint8_t c; int r = recv(fd_, &c, 1, MSG_DONTWAIT); return r == 1 ? c : -1; }
  int peek() { if (fd_ < 0) return -1; uint8_t c; int r = recv(fd_, &c, 1, MSG_DONTWAIT | MSG_PEEK); return r == 1 ? c : -1; }
  size_t print(const char* s) { if (fd_ < 0) return 0; ssize_t r = send(fd_, s, strlen(s), MSG_NOSIGNAL); return r < 0 ? 0 : (size_t)r; }
};

class WiFiServer {
  int lfd_ = -1;
  uint16_t port_;
public:
  explicit WiFiServer(uint16_t port) : port_(port) {}
  void begin();
  void end() { if (lfd_ >= 0) { close(lfd_); lfd_ = -1; } }
  void setNoDelay(bool) {}
  WiFiClient accept() {
    if (lfd_ < 0) return WiFiClient();
    int c = accept4(lfd_, nullptr, nullptr, SOCK_NONBLOCK);
    return c >= 0 ? WiFiClient(c) : WiFiClient();
  }
};

struct SimWiFi {
  wl_status_t status() { return WL_CONNECTED; }
  IPAddress localIP() { return IPAddress(127, 0, 0, 1); }
  void persistent(bool) {}
  void mode(int) {}
  void setSleep(bool) {}
  void setAutoReconnect(bool) {}
  void setHostname(const char*) {}
  bool config(IPAddress, IPAddress, IPAddress, IPAddress, IPAddress) { return true; }
  void begin(const char*, const char*) {}
  void disconnect(bool, bool) {}
  int scanNetworks() { return 3; }
  String SSID(int i) { const char* n[] = {"Red Taller", "Starlink_x,y", "CNC|lab"}; return String(n[i % 3]); }
  void scanDelete() {}
};
extern SimWiFi WiFi;

struct SimEsp { void restart(); };
extern SimEsp ESP;
