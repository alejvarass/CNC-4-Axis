#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <math.h>

const char* WIFI_SSID = "Starlink_ale";
const char* WIFI_PASS = "abelarielV26";
const uint16_t SERVER_PORT = 5000;
const size_t MAX_LINE_LEN = 3072;
const uint8_t PIN_ENABLE_ACTUATORS = 2;

IPAddress ESP32_IP(192, 168, 1, 167);
IPAddress ESP32_GATEWAY(192, 168, 1, 1);
IPAddress ESP32_SUBNET(255, 255, 255, 0);
IPAddress ESP32_DNS1(8, 8, 8, 8);
IPAddress ESP32_DNS2(1, 1, 1, 1);

WiFiServer server(SERVER_PORT);
Preferences prefs;

enum AxisId { AXIS_X=0, AXIS_Y=1, AXIS_Z=2, AXIS_W=3, AXIS_COUNT=4 };
const char* AXIS_NAME[AXIS_COUNT] = {"x","y","z","w"};

struct AxisHW {
  uint8_t pinStep;
  uint8_t pinDir;
  uint8_t pinLimitHome;
};

struct AxisState {
  float pos = 0.0f;
  float targetPos = 0.0f;
  float woff = 0.0f;
  float stepsPerMm = 568.0f;
  float maxTravel = 110.0f;
  
  int64_t stepCount = 0;
  int64_t calibrationStepCount = 0;

  uint32_t softLimitOffsetSteps = 4000;
  uint32_t backoffSteps = 1200;

  bool homed = false;
  bool calibrated = false;
  bool firstRun = true;
  bool isCalibrating = false;
  uint8_t dirForwardLevel = HIGH;
  String lastCalibration = "Sin datos";

  bool limitHome = false;
  bool isMoving = false;
  String moveDir = "none";

  bool manualForward = false;
  bool manualBackward = false;
  bool manualStop = false;

  int homingSeekUs = 250;
  int homingFeedUs = 1200;
  int homingBackoffUs = 400;
  int manualUs = 150;
  
  String lastError = "";
};

AxisHW hw[AXIS_COUNT] = {
  {23, 22, 34},
  {21, 17, 25},
  {16, 4, 14},
  {2, 15, 18}
};

AxisState ax[AXIS_COUNT];

enum MachineState { IDLE, HOMING, CALIBRATING, RUNNING, MANUAL, ALARM };
MachineState machineState = IDLE;
bool actuatorsEnabled = false;

uint32_t disconnectTimeMs = 0;
bool disconnectTimerActive = false;

struct QueuedCmd {
  String id;
  uint32_t seq;
  String cmd;
  String paramsJson;
};

const int CMD_QUEUE_SIZE = 24;
QueuedCmd q[CMD_QUEUE_SIZE];
int qHead = 0, qTail = 0, qCount = 0;

String tcpJogBuffer = "";

volatile uint32_t limitCount[AXIS_COUNT] = {0, 0, 0, 0};
volatile bool limitTriggered[AXIS_COUNT] = {false, false, false, false};

void IRAM_ATTR isrLimitX(){ limitTriggered[AXIS_X]=true; limitCount[AXIS_X]++; }
void IRAM_ATTR isrLimitY(){ limitTriggered[AXIS_Y]=true; limitCount[AXIS_Y]++; }
void IRAM_ATTR isrLimitZ(){ limitTriggered[AXIS_Z]=true; limitCount[AXIS_Z]++; }
void IRAM_ATTR isrLimitW(){ limitTriggered[AXIS_W]=true; limitCount[AXIS_W]++; }

void setActuatorsState(bool enable) {
  actuatorsEnabled = enable;
  digitalWrite(PIN_ENABLE_ACTUATORS, enable ? LOW : HIGH);
}

void resetAllAxesHomed() {
  for(int i = 0; i < AXIS_COUNT; i++) {
    ax[i].homed = false;
  }
}

bool enq(const QueuedCmd& c){
  if (qCount >= CMD_QUEUE_SIZE) return false;
  q[qTail] = c;
  qTail = (qTail + 1) % CMD_QUEUE_SIZE;
  qCount++;
  return true;
}

bool deq(QueuedCmd& o){
  if (!qCount) return false;
  o = q[qHead];
  qHead = (qHead + 1) % CMD_QUEUE_SIZE;
  qCount--;
  return true;
}

void clearQueue(){ qHead = qTail = qCount = 0; }

void sendLine(WiFiClient& c, JsonDocument& d){
  String o;
  serializeJson(d, o);
  o += "\n";
  c.print(o);
}

void pulseAxis(AxisId a, bool isForward) {
  if (!actuatorsEnabled) return;
  digitalWrite(hw[a].pinStep, HIGH);
  delayMicroseconds(2);
  digitalWrite(hw[a].pinStep, LOW);
  
  if (isForward) {
    if (ax[a].stepCount < INT64_MAX) ax[a].stepCount++;
  } else {
    if (ax[a].stepCount > INT64_MIN) ax[a].stepCount--;
  }
  
  if (ax[a].isCalibrating) {
    if (isForward) {
      if (ax[a].calibrationStepCount < INT64_MAX) ax[a].calibrationStepCount++;
    } else {
      if (ax[a].calibrationStepCount > INT64_MIN) ax[a].calibrationStepCount--;
    }
  }
}

bool inLim(AxisId a, float v){ return !(v < 0.0f || v > ax[a].maxTravel); }

void clearManualFlags(AxisId a){
  ax[a].manualForward = false;
  ax[a].manualBackward = false;
  ax[a].manualStop = false;
}

void clearAllManualFlags(){
  for(int i=0; i<AXIS_COUNT; i++) clearManualFlags((AxisId)i);
}

bool axisAnyManual(AxisId a){
  return (ax[a].manualForward || ax[a].manualBackward);
}

void setAxisDirection(AxisId a, bool isForward) {
  uint8_t level = isForward ? ax[a].dirForwardLevel 
                            : (ax[a].dirForwardLevel == HIGH ? LOW : HIGH);
  digitalWrite(hw[a].pinDir, level);
}

AxisId parseAxis(const char* s){
  if(!s) return AXIS_X;
  String a = String(s); a.toLowerCase();
  if(a=="x") return AXIS_X;
  if(a=="y") return AXIS_Y;
  if(a=="z") return AXIS_Z;
  if(a=="w") return AXIS_W;
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

int wifiQualityFromRSSI(int rssi) { 
  if (rssi <= -100) return 0; 
  if (rssi >= -50) return 100; 
  return 2 * (rssi + 100); 
}

void connectWiFiBlocking(){
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  WiFi.config(ESP32_IP, ESP32_GATEWAY, ESP32_SUBNET, ESP32_DNS1, ESP32_DNS2);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  while (WiFi.status() != WL_CONNECTED) { 
    delay(300); 
    Serial.print("."); 
  }

  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  server.begin();
  server.setNoDelay(true);
}

void loadNVS(){
  prefs.begin("cnc_xyzw", true);
  for(int i=0; i<AXIS_COUNT; i++){
    String k = String(AXIS_NAME[i]);
    ax[i].firstRun = prefs.getBool((k + "_fr").c_str(), true);
    ax[i].dirForwardLevel = prefs.getUChar((k + "_dfl").c_str(), HIGH);
    ax[i].stepsPerMm = prefs.getFloat((k+"_spm").c_str(), 568.0f);
    ax[i].maxTravel = prefs.getFloat((k+"_max").c_str(), 110.0f);
    ax[i].calibrated = prefs.getBool((k+"_cal").c_str(), false);
    ax[i].lastCalibration = prefs.getString((k + "_lcal").c_str(), "Sin datos");
    
    ax[i].backoffSteps = prefs.getUInt((k+"_bo_st").c_str(), 1200);
    ax[i].softLimitOffsetSteps = prefs.getUInt((k+"_so_st").c_str(), 4000);

    ax[i].homingSeekUs = prefs.getInt((k+"_hseek").c_str(), 250);
    ax[i].homingBackoffUs = prefs.getInt((k+"_hbo").c_str(), 400);
    ax[i].manualUs = prefs.getInt((k+"_man").c_str(), 150);
  }
  prefs.end();
}

void saveNVS(){
  prefs.begin("cnc_xyzw", false);
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

    prefs.putInt((k+"_hseek").c_str(), ax[i].homingSeekUs);
    prefs.putInt((k+"_hbo").c_str(), ax[i].homingBackoffUs);
    prefs.putInt((k+"_man").c_str(), ax[i].manualUs);
  }
  prefs.end();
}

void pollTcp(WiFiClient& cl) {
  if (!cl || !cl.connected()) return;

  while (cl.available() > 0) {
    char ch = cl.read();
    
    if (tcpJogBuffer.length() >= MAX_LINE_LEN) {
      tcpJogBuffer = "";
      continue;
    }
    
    if (ch == '\n') {
      tcpJogBuffer.trim();
      if (tcpJogBuffer.length() > 0) {
        StaticJsonDocument<1536> d;
        if (!deserializeJson(d, tcpJogBuffer)) {
          String type = String(d["type"].is<const char*>() ? d["type"].as<const char*>() : "");
          String id = String(d["id"].is<const char*>() ? d["id"].as<const char*>() : "");
          
          if (!d["seq"].is<int>()) {
            tcpJogBuffer = "";
            continue;
          }
          uint32_t seq = d["seq"].as<uint32_t>();
          JsonVariantConst p = d["payload"];

          if (type == "cmd" && !p.isNull()) {
            const char* cmd = p["cmd"].is<const char*>() ? p["cmd"].as<const char*>() : "";
            String cmdStr = String(cmd);
            
            // EMERGENCY STOP - MÁXIMA PRIORIDAD
            if (cmdStr == "emergency_stop") {
              clearAllManualFlags();
              setActuatorsState(false);
              machineState = ALARM;
              clearQueue();
              Serial.println("EMERGENCY STOP RECIBIDO");
            }
            // MANUAL STOP - DETIENE MOVIMIENTO ACTUAL
            else if (cmdStr == "manual_stop") {
              clearAllManualFlags();
              for(int i=0; i<AXIS_COUNT; i++) {
                ax[i].isMoving = false;
                ax[i].moveDir = "none";
                ax[i].manualStop = true;
              }
              if (machineState == MANUAL) machineState = IDLE;
              clearQueue();
              Serial.println("MANUAL STOP RECIBIDO");
            } 
            else if (cmdStr == "enable_actuators") {
              setActuatorsState(true);
            } else {
              JsonObjectConst params = p["params"].as<JsonObjectConst>();
              String pj; serializeJson(params, pj);
              QueuedCmd qc{id, seq, cmdStr, pj};
              enq(qc);
            }
          }
        }
      }
      tcpJogBuffer = "";
    } else if (ch != '\r') {
      tcpJogBuffer += ch;
    }
  }
}

void runManualStep(AxisId a) {
  if (!actuatorsEnabled) return;

  bool fwd = ax[a].manualForward;
  bool bwd = ax[a].manualBackward;

  if (!fwd && !bwd) {
    ax[a].isMoving = false;
    ax[a].moveDir = "none";
    if (machineState == MANUAL) machineState = IDLE;
    return;
  }

  machineState = MANUAL;
  bool dirPos = fwd;
  ax[a].isMoving = true;
  ax[a].moveDir = dirPos ? "forward" : "backward";

  if (!dirPos && digitalRead(hw[a].pinLimitHome) == HIGH) {
    ax[a].lastError = "home switch hit";
    ax[a].isMoving = false;
    ax[a].moveDir = "none";
    clearManualFlags(a);
    machineState = ALARM;
    return;
  }

  float nextPos = ax[a].pos + (dirPos ? (1.0f / ax[a].stepsPerMm) : -(1.0f / ax[a].stepsPerMm));
  if (!inLim(a, nextPos)) {
    ax[a].isMoving = false;
    ax[a].moveDir = "none";
    clearManualFlags(a);
    machineState = IDLE;
    return;
  }

  setAxisDirection(a, dirPos);
  pulseAxis(a, dirPos);
  ax[a].pos = nextPos;
  ax[a].targetPos = nextPos;
  
  delayMicroseconds(ax[a].manualUs <= 0 ? 150 : ax[a].manualUs);
}

bool moveAbs(AxisId a, float target){
  if(!actuatorsEnabled){ ax[a].lastError="actuators disabled"; return false; }
  if(!ax[a].homed){ ax[a].lastError="not homed"; return false; }
  if(!inLim(a, target)){ ax[a].lastError="soft limit violation"; return false; }

  float dz = target - ax[a].pos;
  bool dirPos = dz >= 0;
  uint64_t steps = (uint64_t) llabs((long long)(dz * ax[a].stepsPerMm));
  
  if (steps > 0xFFFFFFFFULL) {
    ax[a].lastError = "move distance too large";
    return false;
  }
  
  if(!steps) return true;

  ax[a].targetPos = target;
  ax[a].isMoving = true;
  ax[a].moveDir = dirPos ? "forward" : "backward";

  setAxisDirection(a, dirPos);

  for(uint64_t i=0; i<steps; i++){
    // VERIFICAR STOP EN CADA ITERACIÓN
    if(ax[a].manualStop || !actuatorsEnabled) {
      ax[a].isMoving = false;
      ax[a].moveDir = "none";
      return false;
    }

    if(!dirPos && digitalRead(hw[a].pinLimitHome) == HIGH){
      ax[a].lastError = "home switch hit in operation";
      ax[a].isMoving = false;
      ax[a].moveDir = "none";
      return false;
    }

    pulseAxis(a, dirPos);
    ax[a].pos += (dirPos ? (1.0f/ax[a].stepsPerMm) : -(1.0f/ax[a].stepsPerMm));
    delayMicroseconds(ax[a].manualUs <= 0 ? 150 : ax[a].manualUs);
    yield();
  }

  ax[a].pos = target;
  ax[a].isMoving = false;
  ax[a].moveDir = "none";
  return true;
}

bool moveRel(AxisId a, float dz){
  if(!actuatorsEnabled){ ax[a].lastError="actuators disabled"; return false; }
  if(!ax[a].homed){ ax[a].lastError="not homed"; return false; }
  return moveAbs(a, ax[a].pos + dz);
}

// HOMING CON 4 PASOS ESTÁNDAR
bool homeAxis(AxisId a, uint32_t backoffSteps, uint32_t softOffsetSteps) {
  if (!actuatorsEnabled) {
    ax[a].lastError = "actuators disabled";
    return false;
  }

  clearManualFlags(a);
  ax[a].manualStop = false;
  machineState = HOMING;
  ax[a].homed = false;
  ax[a].isMoving = true;
  ax[a].lastError = "";
  
  if (softOffsetSteps > 0 && softOffsetSteps <= 200000) {
    ax[a].softLimitOffsetSteps = softOffsetSteps;
  } else if (softOffsetSteps > 200000) {
    ax[a].lastError = "soft offset too large";
    machineState = ALARM;
    ax[a].isMoving = false;
    return false;
  }
  
  if (backoffSteps > 0) ax[a].backoffSteps = backoffSteps;

  Serial.print("PASO 1 SEEK - Eje: ");
  Serial.println(AXIS_NAME[a]);

  // PASO 1: BÚSQUEDA RÁPIDA
  setAxisDirection(a, false);
  ax[a].moveDir = "backward";
  
  uint32_t maxSearchSteps = (uint32_t)(ax[a].maxTravel * ax[a].stepsPerMm * 2.0f);
  bool sensorHit = false;

  for (uint32_t i = 0; i < maxSearchSteps; i++) {
    if (!actuatorsEnabled || ax[a].manualStop) { 
      machineState = ALARM; 
      ax[a].isMoving = false; 
      return false; 
    }
    
    // LECTURA DIRECTA DEL SENSOR (NO esperar ISR)
    if (digitalRead(hw[a].pinLimitHome) == HIGH) { 
      sensorHit = true;
      Serial.println("PASO 1: Sensor detectado");
      break; 
    }

    pulseAxis(a, false);
    // Velocidad rápida en búsqueda
    delayMicroseconds(100);  // MÁS RÁPIDO
    yield();
  }

  if (!sensorHit) {
    machineState = ALARM;
    ax[a].lastError = "seek: sensor not found";
    ax[a].isMoving = false;
    Serial.println("ERROR PASO 1");
    return false;
  }

  delay(50);  // Pequeña pausa

  // PASO 2: RETROCESO (~3mm)
  uint32_t pulloffSteps = (uint32_t)(3.0f * ax[a].stepsPerMm);
  
  setAxisDirection(a, true);
  ax[a].moveDir = "forward";
  
  Serial.print("PASO 2 PULLOFF - Pasos: ");
  Serial.println(pulloffSteps);

  for (uint32_t i = 0; i < pulloffSteps; i++) {
    if (!actuatorsEnabled || ax[a].manualStop) { 
      machineState = ALARM; 
      ax[a].isMoving = false; 
      return false; 
    }
    pulseAxis(a, true);
    delayMicroseconds(200);  // MÁS LENTO
    yield();
  }

  delay(50);

  // Verificar que sensor está liberado
  if (digitalRead(hw[a].pinLimitHome) == HIGH) {
    machineState = ALARM;
    ax[a].lastError = "back-off: sensor still engaged";
    ax[a].isMoving = false;
    Serial.println("ERROR PASO 2");
    return false;
  }

  // PASO 3: REAPROXIMACIÓN LENTA
  setAxisDirection(a, false);
  ax[a].moveDir = "backward";
  sensorHit = false;

  uint32_t maxFeedSteps = pulloffSteps * 2;
  
  Serial.print("PASO 3 FEED - Máximo: ");
  Serial.println(maxFeedSteps);

  for (uint32_t i = 0; i < maxFeedSteps; i++) {
    if (!actuatorsEnabled || ax[a].manualStop) { 
      machineState = ALARM; 
      ax[a].isMoving = false; 
      return false; 
    }
    
    // Lectura directa del sensor
    if (digitalRead(hw[a].pinLimitHome) == HIGH) { 
      sensorHit = true;
      Serial.println("PASO 3: Sensor detectado");
      break; 
    }

    pulseAxis(a, false);
    // Velocidad MUCHO MÁS LENTA para precisión
    delayMicroseconds(500);  // MUY LENTO
    yield();
  }

  if (!sensorHit) {
    machineState = ALARM;
    ax[a].lastError = "re-approach: sensor missed";
    ax[a].isMoving = false;
    Serial.println("ERROR PASO 3");
    return false;
  }

  delay(50);

  // PASO 4: FIJACIÓN DEL ORIGEN
  setAxisDirection(a, true);
  ax[a].moveDir = "forward";
  
  Serial.print("PASO 4 FIXATION - Offset: ");
  Serial.println(ax[a].softLimitOffsetSteps);

  for (uint32_t i = 0; i < ax[a].softLimitOffsetSteps; i++) {
    if (!actuatorsEnabled || ax[a].manualStop) { 
      machineState = ALARM; 
      ax[a].isMoving = false; 
      return false; 
    }
    pulseAxis(a, true);
    delayMicroseconds(200);
    yield();
  }

  // ORIGEN ESTABLECIDO
  ax[a].pos = 0.000f;
  ax[a].targetPos = 0.000f;
  ax[a].woff = 0.000f;
  ax[a].stepCount = 0;
  ax[a].homed = true;
  ax[a].isMoving = false;
  ax[a].moveDir = "none";
  ax[a].manualStop = false;
  machineState = IDLE;

  Serial.print("HOMING COMPLETADO - Eje: ");
  Serial.println(AXIS_NAME[a]);

  saveNVS();
  return true;
}

void buildStatus(JsonObject p){
  int rssi = WiFi.isConnected() ? WiFi.RSSI() : -127;
  p["machine_state"] = machineStateStr();
  p["actuators_enabled"] = actuatorsEnabled;
  p["wifi_rssi_dbm"] = rssi;
  p["wifi_quality_pct"] = WiFi.isConnected() ? wifiQualityFromRSSI(rssi) : 0;
  p["wifi_ssid"] = WiFi.isConnected() ? WiFi.SSID() : "";
  p["wifi_ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : "";
  p["queue_len"] = qCount;

  JsonObject axes = p.createNestedObject("axes");
  for(int i=0; i<AXIS_COUNT; i++){
    JsonObject a = axes.createNestedObject(AXIS_NAME[i]);
    a["pos"] = ax[i].pos;
    a["target_pos"] = ax[i].targetPos;
    a["step_count"] = ax[i].stepCount;
    a["calibration_step_count"] = ax[i].calibrationStepCount;
    a["work_offset"] = ax[i].woff;
    a["homed"] = ax[i].homed;
    a["calibrated"] = ax[i].calibrated;
    a["first_run"] = ax[i].firstRun;
    a["dir_forward_level"] = ax[i].dirForwardLevel;
    a["last_calibration"] = ax[i].lastCalibration;
    a["is_moving"] = ax[i].isMoving;
    a["move_dir"] = ax[i].moveDir;
    a["limit_home"] = ax[i].limitHome;
    a["steps_per_mm"] = ax[i].stepsPerMm;
    a["max_travel"] = ax[i].maxTravel;
    a["manual_us"] = ax[i].manualUs;
    a["home_seek_us"] = ax[i].homingSeekUs;
    a["home_backoff_us"] = ax[i].homingBackoffUs;
    a["last_error"] = ax[i].lastError;
  }
}

void ack(WiFiClient& c, const String& id, uint32_t seq){
  StaticJsonDocument<3072> d;
  d["type"] = "ack"; 
  d["id"] = id; 
  d["seq"] = seq; 
  d["ts"] = (uint32_t)(millis()/1000);
  JsonObject p = d.createNestedObject("payload"); 
  buildStatus(p); 
  sendLine(c, d);
}

void nack(WiFiClient& c, const String& id, uint32_t seq, const String& reason){
  StaticJsonDocument<512> d;
  d["type"] = "nack"; 
  d["id"] = id; 
  d["seq"] = seq; 
  d["ts"] = (uint32_t)(millis()/1000);
  JsonObject p = d.createNestedObject("payload"); 
  p["reason"] = reason; 
  sendLine(c, d);
}

void statusMsg(WiFiClient& c, const String& id, uint32_t seq){
  StaticJsonDocument<3072> d;
  d["type"] = "status"; 
  d["id"] = id; 
  d["seq"] = seq; 
  d["ts"] = (uint32_t)(millis()/1000);
  JsonObject p = d.createNestedObject("payload"); 
  buildStatus(p); 
  sendLine(c, d);
}

bool handle(const char* cmd, JsonObjectConst p, String& er){
  String c = String(cmd);
  AxisId a = parseAxis(p["axis"].is<const char*>() ? p["axis"].as<const char*>() : "x");

  if (c == "home_axis") {
    uint32_t bo = ax[a].backoffSteps;
    uint32_t so = ax[a].softLimitOffsetSteps;
    
    if (p["backoff_steps"].is<unsigned int>() || p["backoff_steps"].is<int>()) bo = p["backoff_steps"].as<unsigned int>();
    if (p["soft_offset_steps"].is<unsigned int>() || p["soft_offset_steps"].is<int>()) so = p["soft_offset_steps"].as<unsigned int>();
    
    return homeAxis(a, bo, so);
  }

  if (c == "reset_step_counter") {
    ax[a].calibrationStepCount = 0;
    return true;
  }

  if (c == "test_dir_pulse") {
    if (!actuatorsEnabled) {
      er = "actuadores deshabilitados";
      return false;
    }
    setAxisDirection(a, true);
    long totalSteps = 5000;
    for (long i = 0; i < totalSteps; i++) {
      pulseAxis(a, true);
      delayMicroseconds(150);
      yield();
    }
    return true;
  }

  if (c == "confirm_axis_direction") {
    bool isCorrect = p["is_correct"] | true;
    if (!isCorrect) {
      ax[a].dirForwardLevel = (ax[a].dirForwardLevel == HIGH) ? LOW : HIGH;
    }
    ax[a].firstRun = false;
    saveNVS();
    return true;
  }

  if (c == "set_zero_axis") {
    ax[a].woff = ax[a].pos;
    ax[a].lastError = "";
    return true;
  }

  if (c == "move_axis_abs") {
    float posVal = 0.0f;
    if (p["pos"].is<float>() || p["pos"].is<int>()) posVal = p["pos"].as<float>();
    else { er = "params pos"; return false; }

    machineState = RUNNING;
    bool ok = moveAbs(a, posVal);
    if (ok) machineState = IDLE;
    else machineState = ALARM;
    if (!ok) er = ax[a].lastError;
    return ok;
  }

  if (c == "move_axis_rel") {
    float deltaVal = 0.0f;
    if (p["delta"].is<float>() || p["delta"].is<int>()) deltaVal = p["delta"].as<float>();
    else { er = "params delta"; return false; }

    machineState = RUNNING;
    bool ok = moveRel(a, deltaVal);
    if (ok) machineState = IDLE;
    else machineState = ALARM;
    if (!ok) er = ax[a].lastError;
    return ok;
  }

  if (c == "manual_start") {
    ax[a].manualStop = false;
    clearManualFlags(a);
    const char* dir = p["dir"].is<const char*>() ? p["dir"].as<const char*>() : "";
    String d = String(dir); d.toLowerCase();
    if (d == "forward" || d == "up" || d == "right") ax[a].manualForward = true;
    else if (d == "backward" || d == "down" || d == "left") ax[a].manualBackward = true;
    else { er = "invalid dir"; return false; }
    ax[a].lastError = "";
    return true;
  }

  if (c == "manual_stop") {
    ax[a].manualStop = true;
    clearManualFlags(a);
    ax[a].isMoving = false;
    ax[a].moveDir = "none";
    if (machineState == MANUAL) machineState = IDLE;
    return true;
  }

  if (c == "enable_actuators") {
    setActuatorsState(true);
    return true;
  }

  if (c == "set_calibration_axis") {
    float spm = 0.0f, mt = 0.0f;
    if (p["steps_per_mm"].is<float>() || p["steps_per_mm"].is<int>()) spm = p["steps_per_mm"].as<float>();
    if (p["max_travel"].is<float>() || p["max_travel"].is<int>()) mt = p["max_travel"].as<float>();
    if (p["last_calibration"].is<const char*>()) {
      ax[a].lastCalibration = String(p["last_calibration"].as<const char*>());
    }

    if (spm <= 1.0f || mt <= 1.0f) { er = "invalid calibration values"; return false; }

    ax[a].stepsPerMm = spm;
    ax[a].maxTravel = mt;
    ax[a].calibrated = true;
    ax[a].isCalibrating = false;
    saveNVS();
    if(machineState == CALIBRATING) machineState = IDLE;
    return true;
  }

  if (c == "start_calibration") {
    AxisId aid = parseAxis(p["axis"].is<const char*>() ? p["axis"].as<const char*>() : "x");
    ax[aid].isCalibrating = true;
    ax[aid].calibrationStepCount = 0;
    machineState = CALIBRATING;
    return true;
  }

  if (c == "stop_calibration") {
    AxisId aid = parseAxis(p["axis"].is<const char*>() ? p["axis"].as<const char*>() : "x");
    ax[aid].isCalibrating = false;
    machineState = IDLE;
    return true;
  }

  if (c == "set_manual_speed_axis") {
    if (!p["delay_us"].is<int>()) { er = "params delay_us"; return false; }
    int v = p["delay_us"].as<int>();
    if (v < 5 || v > 20000) { er = "manual speed range"; return false; }
    ax[a].manualUs = v;
    saveNVS();
    return true;
  }

  if (c == "set_homing_speed_axis") {
    if (!p["seek_us"].is<int>() || !p["backoff_us"].is<int>()) { er = "params seek/backoff"; return false; }
    int s = p["seek_us"].as<int>(), b = p["backoff_us"].as<int>();
    if (s < 5 || s > 20000 || b < 5 || b > 20000) { er = "homing speed range"; return false; }
    ax[a].homingSeekUs = s;
    ax[a].homingBackoffUs = b;
    saveNVS();
    return true;
  }

  if (c == "get_status") return true;

  er = "comando desconocido";
  return false;
}

void processOne(WiFiClient& cl){
  QueuedCmd it;
  if(!deq(it)) return;

  StaticJsonDocument<1024> pd;
  if(deserializeJson(pd, it.paramsJson)){
    nack(cl, it.id, it.seq, "params json invalido");
    return;
  }

  String er;
  bool ok = handle(it.cmd.c_str(), pd.as<JsonObjectConst>(), er);
  if(!ok){ nack(cl, it.id, it.seq, er); return; }

  ack(cl, it.id, it.seq);
  statusMsg(cl, it.id, it.seq);
}

void setup(){
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_ENABLE_ACTUATORS, OUTPUT);
  setActuatorsState(false);

  for (int i=0; i<AXIS_COUNT; i++){
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

  attachInterrupt(digitalPinToInterrupt(hw[AXIS_X].pinLimitHome), isrLimitX, RISING);
  attachInterrupt(digitalPinToInterrupt(hw[AXIS_Y].pinLimitHome), isrLimitY, RISING);
  attachInterrupt(digitalPinToInterrupt(hw[AXIS_Z].pinLimitHome), isrLimitZ, RISING);
  attachInterrupt(digitalPinToInterrupt(hw[AXIS_W].pinLimitHome), isrLimitW, RISING);

  loadNVS();
  connectWiFiBlocking();
}

void loop(){
  static WiFiClient cl;
  static uint32_t lastStatusTime = 0;

  if (!cl || !cl.connected()) {
    cl.stop();
    
    if (!disconnectTimerActive) {
      disconnectTimeMs = millis();
      disconnectTimerActive = true;
    } else if (millis() - disconnectTimeMs >= 30000) {
      resetAllAxesHomed();
      setActuatorsState(false);
      clearQueue();
      disconnectTimerActive = false;
    }

    cl = server.available();
    if (cl) {
      disconnectTimerActive = false;
      clearQueue();
      tcpJogBuffer = "";
      cl.setNoDelay(true);
    } else {
      delay(2);
      return;
    }
  }

  if (cl && cl.connected()) {
    disconnectTimerActive = false;
    pollTcp(cl);

    for (int i=0; i<AXIS_COUNT; i++) {
      if (axisAnyManual((AxisId)i) && machineState != HOMING && machineState != RUNNING) {
        runManualStep((AxisId)i);
      }
    }

    processOne(cl);
    
    uint32_t now = millis();
    if (now - lastStatusTime >= 30) {
      lastStatusTime = now;
      StaticJsonDocument<3072> d;
      d["type"] = "status"; 
      d["id"] = "auto"; 
      d["seq"] = 0; 
      d["ts"] = (uint32_t)(millis()/1000);
      JsonObject p = d.createNestedObject("payload"); 
      buildStatus(p);
      sendLine(cl, d);
    }
    
    delay(0);
  }
}
