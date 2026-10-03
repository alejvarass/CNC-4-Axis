#include <WiFi.h>
#include <Preferences.h>
#include <math.h>

const uint16_t SERVER_PORT = 5000;
const size_t MAX_LINE_LEN = 256;
const uint8_t PIN_ENABLE_ACTUATORS = 27; // LOW = Habilitado

String wifi_ssid    = "Starlink_ale";
String wifi_pass    = "abelarielV26";
String wifi_devname = "CNC-XYZW-NETLOG";

IPAddress ESP32_IP(192, 168, 1, 167);
IPAddress ESP32_GATEWAY(192, 168, 1, 1);
IPAddress ESP32_SUBNET(255, 255, 255, 0);
IPAddress ESP32_DNS1(8, 8, 8, 8);
IPAddress ESP32_DNS2(1, 1, 1, 1);

WiFiServer server(SERVER_PORT);
WiFiClient cl;
Preferences prefs;

enum AxisId { AXIS_X=0, AXIS_Y=1, AXIS_Z=2, AXIS_W=3, AXIS_COUNT=4 };
const char* AXIS_NAME[AXIS_COUNT] = {"x","y","z","w"};

struct SCurveProfile {
  float startSpeedMmS;
  float cruiseSpeedMmS;
  float endSpeedMmS;
  float rampRatio;
};

struct AxisHW {
  uint8_t pinStep;
  uint8_t pinDir;
  uint8_t pinLimitHome;
  bool dirPositive;
};

struct AxisState {
  float pos = 0.0f;
  float targetPos = 0.0f;
  float woff = 0.0f;
  float stepsPerMm = 568.0f;
  float maxTravel = 110.0f;
  
  int64_t stepCount = 0;

  uint32_t softLimitOffsetSteps = 2840;
  uint32_t backoffSteps = 1136;

  bool homed = false;
  bool calibrated = false;
  bool firstRun = true;
  bool useSCurve = true;
  uint8_t dirForwardLevel = HIGH;
  String lastCalibration = "Sin datos";

  bool limitHome = false;
  bool isMoving = false;
  String moveDir = "none";

  volatile bool manualForward = false;
  volatile bool manualBackward = false;
  volatile bool manualStop = false;

  int homingSeekUs = 1200;
  int homingFeedUs = 2800;
  int homingBackoffUs = 1500;
  int manualUs = 1000;
  int jogUs = 1000;

  volatile bool runAbsMoveRequested = false;
  volatile bool runHomeRequested = false;
  volatile bool isJogMove = false;
  volatile float pendingTargetPos = 0.0f;
  
  SCurveProfile scurve = {1.5f, 15.0f, 1.5f, 0.25f};
  String lastError = "";
};

AxisHW hw[AXIS_COUNT] = {
  {23, 22, 34, LOW},   // X 
  {21, 17, 25, LOW},   // Y
  {16, 4,  14, LOW},   // Z
  {13, 15, 18, LOW}    // W
};

AxisState ax[AXIS_COUNT];

enum MachineState { IDLE, HOMING, CALIBRATING, RUNNING, MANUAL, ALARM };
volatile MachineState machineState = IDLE;
volatile bool actuatorsEnabled = false;
volatile bool forceStatusPush = false;

volatile unsigned long lastManualCommandMs = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 250;

volatile bool runMultiAbsRequested = false;
float pendingMultiTarget[AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
int pendingMultiFeedUs = 1000;

unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 60; 

TaskHandle_t TaskMotorsHandle = NULL;
SemaphoreHandle_t stateMutex = NULL;
hw_timer_t * stepTimer = NULL;

char tcpBuffer[MAX_LINE_LEN];
uint16_t tcpBufIdx = 0;

char serialBuffer[MAX_LINE_LEN];
uint16_t serialBufIdx = 0;

void IRAM_ATTR onStepTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (TaskMotorsHandle != NULL) {
    vTaskNotifyGiveFromISR(TaskMotorsHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

inline void waitStepHardwareTimer(uint32_t delayUs) {
  if (delayUs < 40) {
    delayMicroseconds(delayUs);
    return;
  }
  timerWrite(stepTimer, 0);
  timerAlarm(stepTimer, (uint64_t)delayUs, false, 0);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void setActuatorsState(bool enable) {
  actuatorsEnabled = enable;
  digitalWrite(PIN_ENABLE_ACTUATORS, enable ? LOW : HIGH);
}

inline void pulseAxis(AxisId a, bool isForward) {
  if (!actuatorsEnabled) return;
  digitalWrite(hw[a].pinStep, HIGH);
  delayMicroseconds(4);
  digitalWrite(hw[a].pinStep, LOW);
  
  if (isForward) {
    ax[a].stepCount++;
  } else {
    ax[a].stepCount--;
  }
  ax[a].pos = (float)ax[a].stepCount / ax[a].stepsPerMm;
}

inline bool inLim(AxisId a, float v){ 
  return !(v < -0.05f || v > (ax[a].maxTravel + 0.05f)); 
}

void clearManualFlags(AxisId a){
  ax[a].manualForward = false;
  ax[a].manualBackward = false;
  ax[a].isMoving = false;
  ax[a].moveDir = "none";
}

void clearAllManualFlags(){
  for(int i=0; i<AXIS_COUNT; i++) clearManualFlags((AxisId)i);
}

void stopAllMotion() {
  clearAllManualFlags();
  for(int i = 0; i < AXIS_COUNT; i++) {
    ax[i].manualStop = true;
    ax[i].runAbsMoveRequested = false;
    ax[i].runHomeRequested = false;
    ax[i].isMoving = false;
    ax[i].moveDir = "none";
  }
  runMultiAbsRequested = false;
  machineState = IDLE;
}

void refreshAxisInputs(AxisId a) {
  ax[a].limitHome = (digitalRead(hw[a].pinLimitHome) == HIGH);
}

void refreshAllInputs(){
  for(int i=0; i<AXIS_COUNT; i++) refreshAxisInputs((AxisId)i);
}

void setAxisDirection(AxisId a, bool isForward) {
  uint8_t level = isForward ? ax[a].dirForwardLevel 
                            : (ax[a].dirForwardLevel == HIGH ? LOW : HIGH);
  digitalWrite(hw[a].pinDir, level);
}

AxisId parseAxis(const char* s){
  if(!s) return AXIS_X;
  char c = tolower(s[0]);
  if(c=='x') return AXIS_X;
  if(c=='y') return AXIS_Y;
  if(c=='z') return AXIS_Z;
  if(c=='w') return AXIS_W;
  return AXIS_X;
}

const char* machineStateStr(){
  switch(machineState){
    case IDLE: return "idle";
    case HOMING: return "homing";
    case CALIBRATING: return "calibrating";
    case RUNNING: return "running";
    case MANUAL: return "manual";
    case ALARM: return "alarm";
    default: return "unknown";
  }
}

void loadNVS(){
  if (!prefs.begin("cnc_xyzw", false)) return;
  wifi_ssid    = prefs.getString("w_ssid", wifi_ssid);
  wifi_pass    = prefs.getString("w_pass", wifi_pass);
  wifi_devname = prefs.getString("w_devname", wifi_devname);

  for(int i=0; i<AXIS_COUNT; i++){
    String k = String(AXIS_NAME[i]);
    ax[i].firstRun = prefs.getBool((k + "_fr").c_str(), true);
    ax[i].dirForwardLevel = prefs.getUChar((k + "_dfl").c_str(), HIGH);
    ax[i].stepsPerMm = prefs.getFloat((k+"_spm").c_str(), 568.0f);
    ax[i].maxTravel = prefs.getFloat((k+"_max").c_str(), 110.0f);
    ax[i].calibrated = prefs.getBool((k+"_cal").c_str(), false);
    ax[i].lastCalibration = prefs.getString((k + "_lcal").c_str(), "Sin datos");
    
    ax[i].backoffSteps = prefs.getUInt((k+"_bo_st").c_str(), 1136);
    ax[i].softLimitOffsetSteps = prefs.getUInt((k+"_so_st").c_str(), 2840);
    ax[i].useSCurve = prefs.getBool((k+"_uscurve").c_str(), true);

    ax[i].homingSeekUs = prefs.getInt((k+"_hseek").c_str(), 1200);
    ax[i].homingFeedUs = prefs.getInt((k+"_hfeed").c_str(), 2800);
    ax[i].homingBackoffUs = prefs.getInt((k+"_hbo").c_str(), 1500);
    ax[i].manualUs = prefs.getInt((k+"_man").c_str(), 1000);
    ax[i].jogUs = prefs.getInt((k+"_jog").c_str(), 1000);

    ax[i].scurve.startSpeedMmS = prefs.getFloat((k+"_sc_s").c_str(), 1.5f);
    ax[i].scurve.cruiseSpeedMmS = prefs.getFloat((k+"_sc_c").c_str(), 15.0f);
    ax[i].scurve.endSpeedMmS = prefs.getFloat((k+"_sc_e").c_str(), 1.5f);
    ax[i].scurve.rampRatio = prefs.getFloat((k+"_sc_r").c_str(), 0.25f);
  }
  prefs.end();
}

void saveNVS(){
  prefs.begin("cnc_xyzw", false);
  prefs.putString("w_ssid", wifi_ssid);
  prefs.putString("w_pass", wifi_pass);
  prefs.putString("w_devname", wifi_devname);

  for(int i=0; i<AXIS_COUNT; i++){
    String k = String(AXIS_NAME[i]);
    prefs.putBool((k + "_fr").c_str(), ax[i].firstRun);
    prefs.putUChar((k + "_dfl").c_str(), ax[i].dirForwardLevel);
    prefs.putFloat((k+"_spm").c_str(), ax[i].stepsPerMm);
    prefs.putFloat((k+"_max").c_str(), ax[i].maxTravel);
    prefs.putBool((k+"_cal").c_str(), ax[i].calibrated);
    prefs.putString((k + "_lcal").c_str(), ax[i].lastCalibration);
    
    prefs.putUInt((k+"_bo_st").c_str(), ax[i].backoffSteps);
    prefs.putUInt((k+"_so_st").c_str(), ax[i].softLimitOffsetSteps);
    prefs.putBool((k+"_uscurve").c_str(), ax[i].useSCurve);

    prefs.putInt((k+"_hseek").c_str(), ax[i].homingSeekUs);
    prefs.putInt((k+"_hfeed").c_str(), ax[i].homingFeedUs);
    prefs.putInt((k+"_hbo").c_str(), ax[i].homingBackoffUs);
    prefs.putInt((k+"_man").c_str(), ax[i].manualUs);
    prefs.putInt((k+"_jog").c_str(), ax[i].jogUs);

    prefs.putFloat((k+"_sc_s").c_str(), ax[i].scurve.startSpeedMmS);
    prefs.putFloat((k+"_sc_c").c_str(), ax[i].scurve.cruiseSpeedMmS);
    prefs.putFloat((k+"_sc_e").c_str(), ax[i].scurve.endSpeedMmS);
    prefs.putFloat((k+"_sc_r").c_str(), ax[i].scurve.rampRatio);
  }
  prefs.end();
}

void connectWiFiBlocking(){
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(wifi_devname.c_str());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.config(ESP32_IP, ESP32_GATEWAY, ESP32_SUBNET, ESP32_DNS1, ESP32_DNS2);
  WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
  
  unsigned long startAtt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAtt < 8000) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
    server.setNoDelay(true);
  }
}

void sendCompactStatus(WiFiClient& c) {
  if (!c || !c.connected()) return;
  refreshAllInputs();

  char outBuf[650];
  int n = snprintf(outBuf, sizeof(outBuf), "ST|%s|%d", machineStateStr(), actuatorsEnabled ? 1 : 0);

  for (int i = 0; i < AXIS_COUNT; i++) {
    n += snprintf(outBuf + n, sizeof(outBuf) - n,
      "|%.3f,%.3f,%ld,%d,%d,%d,%d,%d,%s,%.2f,%.2f,%lu,%lu,%d,%d,%d,%d,%d,%s,%.2f,%.2f,%.2f,%.2f,%s",
      ax[i].pos, ax[i].targetPos, (long)ax[i].stepCount,
      ax[i].homed ? 1 : 0, ax[i].calibrated ? 1 : 0, ax[i].firstRun ? 1 : 0, ax[i].dirForwardLevel,
      ax[i].isMoving ? 1 : 0, ax[i].moveDir.c_str(), ax[i].stepsPerMm, ax[i].maxTravel,
      (unsigned long)ax[i].backoffSteps, (unsigned long)ax[i].softLimitOffsetSteps, ax[i].useSCurve ? 1 : 0,
      ax[i].manualUs, ax[i].jogUs, ax[i].homingSeekUs, ax[i].homingBackoffUs,
      ax[i].lastCalibration.length() > 0 ? ax[i].lastCalibration.c_str() : "None",
      ax[i].scurve.startSpeedMmS, ax[i].scurve.cruiseSpeedMmS, ax[i].scurve.endSpeedMmS, ax[i].scurve.rampRatio,
      ax[i].lastError.length() > 0 ? ax[i].lastError.c_str() : "None"
    );
  }
  strncat(outBuf, "\n", sizeof(outBuf) - strlen(outBuf) - 1);
  c.print(outBuf);
}

void runContinuousManual(AxisId a) {
  if (!actuatorsEnabled) return;
  bool dirPos = ax[a].manualForward;
  if (!dirPos && !ax[a].manualBackward) return;

  machineState = MANUAL;
  ax[a].isMoving = true;
  ax[a].moveDir = dirPos ? "forward" : "backward";
  setAxisDirection(a, dirPos);

  while (actuatorsEnabled && (dirPos ? ax[a].manualForward : ax[a].manualBackward) && !ax[a].manualStop) {
    if (millis() - lastManualCommandMs > WATCHDOG_TIMEOUT_MS) {
      ax[a].lastError = "watchdog deadman timeout";
      break;
    }
    if (digitalRead(hw[a].pinLimitHome) == HIGH) {
      ax[a].lastError = "switch hit";
      break;
    }

    float nextPos = ax[a].pos + (dirPos ? (1.0f / ax[a].stepsPerMm) : -(1.0f / ax[a].stepsPerMm));
    if (ax[a].homed && !inLim(a, nextPos)) break;

    pulseAxis(a, dirPos);
    ax[a].targetPos = ax[a].pos;

    waitStepHardwareTimer(ax[a].manualUs);
  }

  clearManualFlags(a);
  machineState = IDLE;
  forceStatusPush = true;
}

bool moveAbsExecution(AxisId a, float target, bool isJog){
  if(!actuatorsEnabled){ ax[a].lastError="actuators disabled"; return false; }
  if(ax[a].homed && !inLim(a, target)){ ax[a].lastError="soft limit violation"; return false; }

  float dz = target - ax[a].pos;
  bool dirPos = dz >= 0.0f;
  uint32_t steps = (uint32_t) llabs((long long)(dz * ax[a].stepsPerMm));
  if(!steps) return true;

  machineState = RUNNING;
  ax[a].targetPos = target;
  ax[a].isMoving = true;
  ax[a].moveDir = dirPos ? "forward" : "backward";
  ax[a].manualStop = false;
  setAxisDirection(a, dirPos);

  float spm = ax[a].stepsPerMm;
  float f_start = max(60.0f, ax[a].scurve.startSpeedMmS * spm);
  float f_cruise = max(f_start, ax[a].scurve.cruiseSpeedMmS * spm);
  float f_end = max(60.0f, ax[a].scurve.endSpeedMmS * spm);

  long rampSteps = (long)(steps * ax[a].scurve.rampRatio);
  if (rampSteps < 1) rampSteps = 1;
  if (rampSteps * 2 > (long)steps) rampSteps = steps / 2;

  for(uint32_t i=0; i<steps; i++){
    if(ax[a].manualStop || !actuatorsEnabled) break;

    if(digitalRead(hw[a].pinLimitHome) == HIGH){
      ax[a].lastError = "switch hit";
      break;
    }

    pulseAxis(a, dirPos);
    
    int delayUs;
    if (ax[a].useSCurve) {
      float currentFreq;
      if (i < rampSteps) {
        float factor = 0.5f * (1.0f - cosf(((float)i / (float)rampSteps) * M_PI));
        currentFreq = f_start + (f_cruise - f_start) * factor;
      } else if (i >= (steps - rampSteps)) {
        float factor = 0.5f * (1.0f - cosf(((float)(steps - 1 - i) / (float)rampSteps) * M_PI));
        currentFreq = f_end + (f_cruise - f_end) * factor;
      } else {
        currentFreq = f_cruise;
      }
      delayUs = (int)(1000000.0f / currentFreq);
    } else {
      delayUs = ax[a].jogUs;
    }

    waitStepHardwareTimer(constrain(delayUs, 30, 20000));
  }

  ax[a].targetPos = ax[a].pos;
  ax[a].isMoving = false;
  ax[a].moveDir = "none";
  machineState = IDLE;
  forceStatusPush = true;
  return true;
}

bool moveMultiAbsExecution(const float targets[AXIS_COUNT], int feedUs) {
  if (!actuatorsEnabled) return false;

  uint32_t deltaSteps[AXIS_COUNT];
  bool dirPos[AXIS_COUNT];
  uint32_t maxSteps = 0;

  for (int i = 0; i < AXIS_COUNT; i++) {
    if (ax[i].homed && !inLim((AxisId)i, targets[i])) {
      ax[i].lastError = "soft limit violation";
      return false;
    }
    float d = targets[i] - ax[i].pos;
    dirPos[i] = (d >= 0.0f);
    deltaSteps[i] = (uint32_t)llabs((long long)(d * ax[i].stepsPerMm));
    if (deltaSteps[i] > maxSteps) maxSteps = deltaSteps[i];

    setAxisDirection((AxisId)i, dirPos[i]);
    ax[i].targetPos = targets[i];
    ax[i].isMoving = (deltaSteps[i] > 0);
    ax[i].moveDir = dirPos[i] ? "forward" : "backward";
  }

  if (maxSteps == 0) return true;

  machineState = RUNNING;
  long errAccum[AXIS_COUNT] = {0, 0, 0, 0};

  for (uint32_t step = 0; step < maxSteps; step++) {
    if (!actuatorsEnabled) break;

    bool abortMove = false;
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (ax[i].manualStop) abortMove = true;
      if (digitalRead(hw[i].pinLimitHome) == HIGH) {
        ax[i].lastError = "switch hit";
        abortMove = true;
      }
    }
    if (abortMove) break;

    for (int i = 0; i < AXIS_COUNT; i++) {
      if (deltaSteps[i] > 0) {
        errAccum[i] += deltaSteps[i];
        if (errAccum[i] >= (long)maxSteps) {
          errAccum[i] -= maxSteps;
          pulseAxis((AxisId)i, dirPos[i]);
        }
      }
    }

    waitStepHardwareTimer(feedUs);
  }

  for (int i = 0; i < AXIS_COUNT; i++) {
    ax[i].targetPos = ax[i].pos;
    ax[i].isMoving = false;
    ax[i].moveDir = "none";
  }
  machineState = IDLE;
  forceStatusPush = true;
  return true;
}

bool homeAxisExecution(AxisId a) {
  if (!actuatorsEnabled) return false;

  machineState = HOMING;
  ax[a].homed = false;
  ax[a].isMoving = true;
  ax[a].lastError = "";
  ax[a].manualStop = false;

  setAxisDirection(a, false);
  ax[a].moveDir = "backward";
  uint32_t maxSearchSteps = (uint32_t)(ax[a].maxTravel * ax[a].stepsPerMm * 2.0f);
  bool sensorHit = false;

  for (uint32_t i = 0; i < maxSearchSteps; i++) {
    if (!actuatorsEnabled || ax[a].manualStop) { machineState = IDLE; ax[a].isMoving = false; return false; }
    if (digitalRead(hw[a].pinLimitHome) == HIGH) { sensorHit = true; break; }
    
    pulseAxis(a, false);
    waitStepHardwareTimer(ax[a].homingSeekUs);
  }

  if (!sensorHit) {
    machineState = IDLE;
    ax[a].lastError = "seek: sensor not found";
    ax[a].isMoving = false;
    forceStatusPush = true;
    return false;
  }

  delay(120);

  setAxisDirection(a, true);
  ax[a].moveDir = "forward";
  for (uint32_t i = 0; i < ax[a].backoffSteps; i++) {
    if (!actuatorsEnabled || ax[a].manualStop) { machineState = IDLE; ax[a].isMoving = false; return false; }
    pulseAxis(a, true);
    waitStepHardwareTimer(ax[a].homingBackoffUs);
  }

  delay(120);
  if (digitalRead(hw[a].pinLimitHome) == HIGH) {
    machineState = IDLE;
    ax[a].lastError = "back-off failed";
    ax[a].isMoving = false;
    forceStatusPush = true;
    return false;
  }

  setAxisDirection(a, false);
  ax[a].moveDir = "backward";
  sensorHit = false;
  uint32_t maxFeedSteps = ax[a].backoffSteps * 2;

  for (uint32_t i = 0; i < maxFeedSteps; i++) {
    if (!actuatorsEnabled || ax[a].manualStop) { machineState = IDLE; ax[a].isMoving = false; return false; }
    if (digitalRead(hw[a].pinLimitHome) == HIGH) { sensorHit = true; break; }
    pulseAxis(a, false);
    waitStepHardwareTimer(ax[a].homingFeedUs);
  }

  if (!sensorHit) {
    machineState = IDLE;
    ax[a].lastError = "re-approach missed";
    ax[a].isMoving = false;
    forceStatusPush = true;
    return false;
  }

  delay(120);

  setAxisDirection(a, true);
  ax[a].moveDir = "forward";
  for (uint32_t i = 0; i < ax[a].softLimitOffsetSteps; i++) {
    if (!actuatorsEnabled || ax[a].manualStop) { machineState = IDLE; ax[a].isMoving = false; return false; }
    pulseAxis(a, true);
    waitStepHardwareTimer(ax[a].homingBackoffUs);
  }

  ax[a].stepCount = 0;
  ax[a].pos = 0.000f;
  ax[a].targetPos = 0.000f;
  ax[a].woff = 0.000f;
  ax[a].homed = true;
  ax[a].firstRun = false;
  ax[a].isMoving = false;
  ax[a].moveDir = "none";
  saveNVS();

  machineState = IDLE;
  forceStatusPush = true;
  return true;
}

void TaskMotors(void * pvParameters) {
  for(;;) {
    if (actuatorsEnabled) {
      if (xSemaphoreTake(stateMutex, (TickType_t)5) == pdTRUE) {
        if (runMultiAbsRequested) {
          runMultiAbsRequested = false;
          float tgt[AXIS_COUNT];
          int fUs = pendingMultiFeedUs;
          for (int i = 0; i < AXIS_COUNT; i++) tgt[i] = pendingMultiTarget[i];
          xSemaphoreGive(stateMutex);
          moveMultiAbsExecution(tgt, fUs);
        } else {
          for (int i = 0; i < AXIS_COUNT; i++) {
            if (ax[i].runAbsMoveRequested) {
              ax[i].runAbsMoveRequested = false;
              float pTgt = ax[i].pendingTargetPos;
              bool jg = ax[i].isJogMove;
              xSemaphoreGive(stateMutex);
              moveAbsExecution((AxisId)i, pTgt, jg);
              break;
            }
            if (ax[i].runHomeRequested) {
              ax[i].runHomeRequested = false;
              xSemaphoreGive(stateMutex);
              homeAxisExecution((AxisId)i);
              break;
            }
            if (ax[i].manualForward || ax[i].manualBackward) {
              xSemaphoreGive(stateMutex);
              runContinuousManual((AxisId)i);
              break;
            }
          }
          xSemaphoreGive(stateMutex);
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void parseSerialCommand(char* line) {
  if (strcmp(line, "SCAN_WIFI") == 0) {
    int n = WiFi.scanNetworks();
    Serial.print("WIFI_LIST|");
    for (int i = 0; i < n; ++i) {
      Serial.print(WiFi.SSID(i));
      if (i < n - 1) Serial.print(",");
    }
    Serial.println();
    WiFi.scanDelete();
    return;
  }

  if (strncmp(line, "SET_WIFI_NVS|", 13) == 0) {
    char* ssid = strtok(line + 13, "|");
    char* pass = strtok(NULL, "|");
    char* devname = strtok(NULL, "|");
    if (ssid) {
      wifi_ssid = String(ssid);
      wifi_pass = pass ? String(pass) : "";
      wifi_devname = (devname && strlen(devname) > 0) ? String(devname) : "CNC-XYZW-NETLOG";
      saveNVS();
      Serial.println("ACK|SET_WIFI_NVS|OK");
    } else {
      Serial.println("ACK|SET_WIFI_NVS|FAIL");
    }
    return;
  }

  if (strcmp(line, "DUMP_NVS") == 0) {
    Serial.print("NVS_DATA|{");
    // Incluir IP actual del ESP32 en el volcado para que el py de mantenimiento la lea
    String currentIpStr = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
    Serial.printf("\"current_ip\":\"%s\",", currentIpStr.c_str());
    Serial.printf("\"ssid\":\"%s\",", wifi_ssid.c_str());
    Serial.printf("\"pass\":\"%s\",", wifi_pass.c_str());
    Serial.printf("\"devname\":\"%s\",", wifi_devname.c_str());
    Serial.print("\"axes\":{");
    for (int i = 0; i < AXIS_COUNT; i++) {
      Serial.printf("\"%s\":{\"spm\":%.3f,\"max\":%.2f,\"bo\":%lu,\"so\":%lu,\"sc\":%d,\"seek\":%d,\"feed\":%d,\"bo_us\":%d,\"man_us\":%d,\"jog_us\":%d,\"dfl\":%d,\"sc_s\":%.2f,\"sc_c\":%.2f,\"sc_e\":%.2f,\"sc_r\":%.2f}",
        AXIS_NAME[i], ax[i].stepsPerMm, ax[i].maxTravel, (unsigned long)ax[i].backoffSteps,
        (unsigned long)ax[i].softLimitOffsetSteps, ax[i].useSCurve ? 1 : 0, ax[i].homingSeekUs,
        ax[i].homingFeedUs, ax[i].homingBackoffUs, ax[i].manualUs, ax[i].jogUs,
        ax[i].dirForwardLevel, ax[i].scurve.startSpeedMmS, ax[i].scurve.cruiseSpeedMmS,
        ax[i].scurve.endSpeedMmS, ax[i].scurve.rampRatio);
      if (i < AXIS_COUNT - 1) Serial.print(",");
    }
    Serial.println("}}");
    return;
  }

  if (strncmp(line, "LOAD_AXIS_PARAM|", 16) == 0) {
    char* axStr = strtok(line + 16, "|");
    char* spmStr = strtok(NULL, "|");
    char* maxStr = strtok(NULL, "|");
    char* boStr = strtok(NULL, "|");
    char* soStr = strtok(NULL, "|");
    char* scStr = strtok(NULL, "|");
    char* seekStr = strtok(NULL, "|");
    char* feedStr = strtok(NULL, "|");
    char* boUsStr = strtok(NULL, "|");
    char* manUsStr = strtok(NULL, "|");
    char* jogUsStr = strtok(NULL, "|");
    char* dflStr = strtok(NULL, "|");

    if (axStr) {
      AxisId a = parseAxis(axStr);
      if (spmStr) ax[a].stepsPerMm = atof(spmStr);
      if (maxStr) ax[a].maxTravel = atof(maxStr);
      if (boStr) ax[a].backoffSteps = atoi(boStr);
      if (soStr) ax[a].softLimitOffsetSteps = atoi(soStr);
      if (scStr) ax[a].useSCurve = (atoi(scStr) == 1);
      if (seekStr) ax[a].homingSeekUs = atoi(seekStr);
      if (feedStr) ax[a].homingFeedUs = atoi(feedStr);
      if (boUsStr) ax[a].homingBackoffUs = atoi(boUsStr);
      if (manUsStr) ax[a].manualUs = atoi(manUsStr);
      if (jogUsStr) ax[a].jogUs = atoi(jogUsStr);
      if (dflStr) ax[a].dirForwardLevel = atoi(dflStr);
      saveNVS();
      Serial.println("ACK|LOAD_AXIS_PARAM|OK");
    }
    return;
  }

  if (strcmp(line, "RESTART_ESP") == 0) {
    Serial.println("ACK|RESTART_ESP|OK");
    delay(200);
    ESP.restart();
    return;
  }
}

void parseCompactLine(WiFiClient& client, char* line) {
  if (line[0] == '$' || line[0] == '?' || line[0] == '~' || line[0] == '!' || line[0] == 'G') {
    if (line[0] == '?') {
      client.printf("<%s|WPos:%.3f,%.3f,%.3f,%.3f|FS:0,0>\r\n", 
                    machineStateStr(), ax[0].pos, ax[1].pos, ax[2].pos, ax[3].pos);
      return;
    }
    if (line[0] == '$' && line[1] == 'H') {
      for (int i = 0; i < AXIS_COUNT; i++) ax[i].runHomeRequested = true;
      client.print("ok\r\n");
      return;
    }
    if (line[0] == '!') {
      stopAllMotion();
      client.print("ok\r\n");
      return;
    }
    if (line[0] == '~') {
      client.print("ok\r\n");
      return;
    }
    client.print("ok\r\n");
    return;
  }

  char* tag = strtok(line, "|");
  if (!tag) return;

  if (strcmp(tag, "GET_STATUS") == 0) {
    sendCompactStatus(client);
    return;
  }

  if (strcmp(tag, "CMD") == 0) {
    char* cmdName = strtok(NULL, "|");
    if (!cmdName) return;

    if (strcmp(cmdName, "enable_actuators") == 0) {
      setActuatorsState(true);
      client.print("ACK|enable_actuators|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "emergency_stop") == 0) {
      stopAllMotion();
      setActuatorsState(false);
      client.print("ACK|emergency_stop|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "invert_axis_dir") == 0) {
      char* axStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].dirForwardLevel = (ax[a].dirForwardLevel == HIGH) ? LOW : HIGH;
      saveNVS();
      client.print("ACK|invert_axis_dir|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "manual_start") == 0) {
      char* axStr = strtok(NULL, "|");
      char* dirStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].manualStop = false;
      if (strcmp(dirStr, "forward") == 0 || strcmp(dirStr, "up") == 0 || strcmp(dirStr, "right") == 0) {
        ax[a].manualBackward = false;
        ax[a].manualForward = true;
      } else {
        ax[a].manualForward = false;
        ax[a].manualBackward = true;
      }
      lastManualCommandMs = millis();
      client.print("ACK|manual_start|OK\n");
      return;
    }
    if (strcmp(cmdName, "manual_ping") == 0) {
      lastManualCommandMs = millis();
      client.print("ACK|manual_ping|OK\n");
      return;
    }
    if (strcmp(cmdName, "manual_stop") == 0) {
      char* axStr = strtok(NULL, "|");
      if (axStr) {
        AxisId a = parseAxis(axStr);
        ax[a].manualStop = true;
        ax[a].manualForward = false;
        ax[a].manualBackward = false;
      } else {
        stopAllMotion();
      }
      client.print("ACK|manual_stop|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "home_axis") == 0) {
      char* axStr = strtok(NULL, "|");
      char* boStr = strtok(NULL, "|");
      char* soStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      if (boStr) ax[a].backoffSteps = atoi(boStr);
      if (soStr) ax[a].softLimitOffsetSteps = atoi(soStr);
      saveNVS();
      ax[a].runHomeRequested = true;
      client.print("ACK|home_axis|OK\n");
      return;
    }
    if (strcmp(cmdName, "set_zero_axis") == 0) {
      char* axStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].stepCount = 0;
      ax[a].pos = 0.0f;
      ax[a].targetPos = 0.0f;
      ax[a].woff = 0.0f;
      ax[a].homed = true;
      client.print("ACK|set_zero_axis|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "move_axis_abs") == 0) {
      char* axStr = strtok(NULL, "|");
      char* valStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].pendingTargetPos = atof(valStr);
      ax[a].isJogMove = false;
      ax[a].runAbsMoveRequested = true;
      client.print("ACK|move_axis_abs|OK\n");
      return;
    }
    if (strcmp(cmdName, "move_multi_abs") == 0) {
      char* xStr = strtok(NULL, "|");
      char* yStr = strtok(NULL, "|");
      char* zStr = strtok(NULL, "|");
      char* wStr = strtok(NULL, "|");
      char* fStr = strtok(NULL, "|");
      if (xStr && yStr && zStr && wStr) {
        pendingMultiTarget[0] = atof(xStr);
        pendingMultiTarget[1] = atof(yStr);
        pendingMultiTarget[2] = atof(zStr);
        pendingMultiTarget[3] = atof(wStr);
        pendingMultiFeedUs = fStr ? atoi(fStr) : 1000;
        runMultiAbsRequested = true;
        client.print("ACK|move_multi_abs|OK\n");
      }
      return;
    }
    if (strcmp(cmdName, "move_axis_rel") == 0) {
      char* axStr = strtok(NULL, "|");
      char* valStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].pendingTargetPos = ax[a].pos + atof(valStr);
      ax[a].isJogMove = true;
      ax[a].runAbsMoveRequested = true;
      client.print("ACK|move_axis_rel|OK\n");
      return;
    }
    if (strcmp(cmdName, "reset_step_counter") == 0) {
      char* axStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].stepCount = 0;
      ax[a].pos = 0.0f;
      client.print("ACK|reset_step_counter|OK\n");
      return;
    }
    if (strcmp(cmdName, "set_calibration_axis") == 0) {
      char* axStr = strtok(NULL, "|");
      char* spmStr = strtok(NULL, "|");
      char* mtStr = strtok(NULL, "|");
      char* dateStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].stepsPerMm = atof(spmStr);
      ax[a].maxTravel = atof(mtStr);
      ax[a].calibrated = true;
      if (dateStr) ax[a].lastCalibration = String(dateStr);
      saveNVS();
      client.print("ACK|set_calibration_axis|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "set_scurve_profile_axis") == 0) {
      char* axStr = strtok(NULL, "|");
      char* sStr = strtok(NULL, "|");
      char* cStr = strtok(NULL, "|");
      char* eStr = strtok(NULL, "|");
      char* rStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].scurve.startSpeedMmS = atof(sStr);
      ax[a].scurve.cruiseSpeedMmS = atof(cStr);
      ax[a].scurve.endSpeedMmS = atof(eStr);
      ax[a].scurve.rampRatio = atof(rStr);
      saveNVS();
      client.print("ACK|set_scurve_profile_axis|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "set_axis_mode") == 0) {
      char* axStr = strtok(NULL, "|");
      char* modeStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].useSCurve = (atoi(modeStr) == 1);
      saveNVS();
      client.print("ACK|set_axis_mode|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "set_manual_speed_axis") == 0) {
      char* axStr = strtok(NULL, "|");
      char* manStr = strtok(NULL, "|");
      char* jogStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      if (manStr) ax[a].manualUs = atoi(manStr);
      if (jogStr) ax[a].jogUs = atoi(jogStr);
      saveNVS();
      client.print("ACK|set_manual_speed_axis|OK\n");
      sendCompactStatus(client);
      return;
    }
    if (strcmp(cmdName, "set_homing_speed_axis") == 0) {
      char* axStr = strtok(NULL, "|");
      char* seekStr = strtok(NULL, "|");
      char* feedStr = strtok(NULL, "|");
      char* boStr = strtok(NULL, "|");
      AxisId a = parseAxis(axStr);
      ax[a].homingSeekUs = atoi(seekStr);
      ax[a].homingFeedUs = atoi(feedStr);
      ax[a].homingBackoffUs = atoi(boStr);
      saveNVS();
      client.print("ACK|set_homing_speed_axis|OK\n");
      sendCompactStatus(client);
      return;
    }
  }
}

void setup(){
  Serial.begin(115200);
  delay(150);

  stateMutex = xSemaphoreCreateMutex();

  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);

  loadNVS();

  pinMode(PIN_ENABLE_ACTUATORS, OUTPUT);
  setActuatorsState(false);

  for (int i = 0; i < AXIS_COUNT; i++){
    pinMode(hw[i].pinStep, OUTPUT);
    pinMode(hw[i].pinDir, OUTPUT);
    if (hw[i].pinLimitHome == 34 || hw[i].pinLimitHome == 35 || 
        hw[i].pinLimitHome == 36 || hw[i].pinLimitHome == 39) {
      pinMode(hw[i].pinLimitHome, INPUT);
    } else {
      pinMode(hw[i].pinLimitHome, INPUT_PULLUP);
    }
    digitalWrite(hw[i].pinStep, LOW);
  }

  connectWiFiBlocking();
  xTaskCreatePinnedToCore(TaskMotors, "TaskMotors", 8192, NULL, 1, &TaskMotorsHandle, 1);
}

void loop(){
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialBufIdx > 0) {
        serialBuffer[serialBufIdx] = '\0';
        parseSerialCommand(serialBuffer);
        serialBufIdx = 0;
      }
    } else {
      if (serialBufIdx < (MAX_LINE_LEN - 1)) {
        serialBuffer[serialBufIdx++] = ch;
      }
    }
  }

  if (!cl || !cl.connected()) {
    cl.stop();
    cl = server.accept();
    if (cl) {
      tcpBufIdx = 0;
      cl.setNoDelay(true);
    } else {
      delay(2);
      return;
    }
  }

  if (cl && cl.connected()) {
    while (cl.available() > 0) {
      char ch = (char)cl.read();
      if (ch == '\n' || ch == '\r') {
        if (tcpBufIdx > 0) {
          tcpBuffer[tcpBufIdx] = '\0';
          parseCompactLine(cl, tcpBuffer);
          tcpBufIdx = 0;
        }
      } else {
        if (tcpBufIdx < (MAX_LINE_LEN - 1)) {
          tcpBuffer[tcpBufIdx++] = ch;
        }
      }
    }

    refreshAllInputs();

    if (forceStatusPush || (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)) {
      lastTelemetryMs = millis();
      forceStatusPush = false;
      sendCompactStatus(cl);
    }
    delay(1);
  }
}