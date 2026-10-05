/*
 * CNC XYZW Versión 6.5.1 (Pure GRBL Engine)
 * - Límites duales por hardware configurables por NVS ($21)
 * - Soporte nativo de Ciclo Touch Probe G38.2/G38.3 ($29)
 * - Perfil trapezoidal real y dead-man de red
 * - Manejo libre sin obligatoriedad de homing previo ($20=0)
 */

#include "Config.h"
#include "Motion.h"
#include "Planner.h"
#include "GCodeParser.h"
#include <WiFi.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_random.h>

#define WDT_TIMEOUT_SECONDS 5

String wifi_ssid = "";
String wifi_pass = "";
String wifi_devname = "CNC-XYZW-GRBL";
String softap_pass = "";
uint8_t ip_oct4 = 167;

WiFiServer server(SERVER_PORT);
WiFiClient client;
Preferences prefs;

char tcpBuffer[MAX_LINE_LEN];
uint16_t tcpIdx = 0;
char serialBuffer[MAX_LINE_LEN];
uint16_t serialIdx = 0;

void migrateOldNVS() {
  Preferences oldPrefs;
  if (oldPrefs.begin("cnc_xyzw", true)) {
    prefs.begin("cnc_grbl", false);
    for (int i = 0; i < AXIS_COUNT; i++) {
      String p = String(AXIS_CHARS[i]);
      if (!prefs.isKey((p + "_spm").c_str()) && oldPrefs.isKey((p + "_spm").c_str()))
        prefs.putFloat((p + "_spm").c_str(), oldPrefs.getFloat((p + "_spm").c_str(), 568.0f));
      if (!prefs.isKey((p + "_rate").c_str()) && oldPrefs.isKey((p + "_rate").c_str()))
        prefs.putFloat((p + "_rate").c_str(), oldPrefs.getFloat((p + "_rate").c_str(), 1320.0f));
      if (!prefs.isKey((p + "_acc").c_str()) && oldPrefs.isKey((p + "_acc").c_str()))
        prefs.putFloat((p + "_acc").c_str(), oldPrefs.getFloat((p + "_acc").c_str(), 30.0f));
      if (!prefs.isKey((p + "_max").c_str()) && oldPrefs.isKey((p + "_max").c_str()))
        prefs.putFloat((p + "_max").c_str(), oldPrefs.getFloat((p + "_max").c_str(), 110.0f));
      if (!prefs.isKey((p + "_bl").c_str()) && oldPrefs.isKey((p + "_bl").c_str()))
        prefs.putFloat((p + "_bl").c_str(), oldPrefs.getFloat((p + "_bl").c_str(), 0.0f));
      if (!prefs.isKey((p + "_dir").c_str()) && oldPrefs.isKey((p + "_dir").c_str()))
        prefs.putUChar((p + "_dir").c_str(), oldPrefs.getUChar((p + "_dir").c_str(), 0));
    }
    if (!prefs.isKey("w_ssid") && oldPrefs.isKey("w_ssid"))
      prefs.putString("w_ssid", oldPrefs.getString("w_ssid", ""));
    if (!prefs.isKey("w_pass") && oldPrefs.isKey("w_pass"))
      prefs.putString("w_pass", oldPrefs.getString("w_pass", ""));
    if (!prefs.isKey("h_feed") && oldPrefs.isKey("h_feed"))
      prefs.putFloat("h_feed", oldPrefs.getFloat("h_feed", 50.0f));
    if (!prefs.isKey("h_seek") && oldPrefs.isKey("h_seek"))
      prefs.putFloat("h_seek", oldPrefs.getFloat("h_seek", 500.0f));
    if (!prefs.isKey("h_pull") && oldPrefs.isKey("h_pull"))
      prefs.putFloat("h_pull", oldPrefs.getFloat("h_pull", 2.0f));
    prefs.end();
    oldPrefs.end();
  }
}

String generateRandomPassword() {
  const char charset[] = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz";
  String pass = "CNC_";
  for (int i = 0; i < 8; i++) {
    pass += charset[esp_random() % (sizeof(charset) - 1)];
  }
  return pass;
}

void loadNVS() {
  migrateOldNVS();
  if (!prefs.begin("cnc_grbl", false)) return;
  uint32_t schema = prefs.getUInt("nvs_schema", 0);
  if (schema < NVS_SCHEMA_VERSION) {
    prefs.putUInt("nvs_schema", NVS_SCHEMA_VERSION);
  }
  wifi_ssid = prefs.getString("w_ssid", "");
  wifi_pass = prefs.getString("w_pass", "");
  wifi_devname = prefs.getString("w_devname", "CNC-XYZW-GRBL");

  if (!prefs.isKey("ap_pass")) {
    String rnd = generateRandomPassword();
    prefs.putString("ap_pass", rnd);
    softap_pass = rnd;
  } else {
    softap_pass = prefs.getString("ap_pass", "CNC_998877");
  }

  ip_oct4 = prefs.getUChar("ip_oct4", 167);
  dualLimitsEnabled = (prefs.getUChar("dual_lim", 0) == 1);
  probeEnabled = (prefs.getUChar("probe_en", 1) == 1);
  softLimitsEnabled = (prefs.getUChar("soft_lim", 0) == 1);

  float h_seek = prefs.getFloat("h_seek", 500.0f);
  float h_feed = prefs.getFloat("h_feed", 50.0f);
  float h_pull = prefs.getFloat("h_pull", 2.0f);

  for (int i = 0; i < AXIS_COUNT; i++) {
    String p = String(AXIS_CHARS[i]);
    ax[i].stepsPerMm = prefs.getFloat((p + "_spm").c_str(), 568.0f);
    ax[i].maxRateMmMin = prefs.getFloat((p + "_rate").c_str(), 1320.0f);
    ax[i].accelMmSec2 = prefs.getFloat((p + "_acc").c_str(), 30.0f);
    ax[i].maxTravelMm = prefs.getFloat((p + "_max").c_str(), 110.0f);
    ax[i].backlashMm = prefs.getFloat((p + "_bl").c_str(), 0.0f);
    ax[i].dirInvert = prefs.getUChar((p + "_dir").c_str(), 0);
    ax[i].homingSeekRateMmMin = h_seek;
    ax[i].homingFeedRateMmMin = h_feed;
    ax[i].homingPulloffMm = h_pull;
  }
  prefs.end();
}

void initNetwork() {
  if (wifi_ssid.length() > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(wifi_devname.c_str());
    IPAddress localIp(192, 168, 1, ip_oct4);
    IPAddress gw(192, 168, 1, 1);
    IPAddress mask(255, 255, 255, 0);
    WiFi.config(localIp, gw, mask);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("CNC-XYZW-CONFIG", softap_pass.c_str());
    Serial.printf("[WIFI] SoftAP Seguro. SSID: CNC-XYZW-CONFIG | Clave NVS: %s | IP: %s\n",
                  softap_pass.c_str(), WiFi.softAPIP().toString().c_str());
  } else {
    Serial.printf("[WIFI] Conectado STA. IP: %s\n", WiFi.localIP().toString().c_str());
  }

  server.begin();
  server.setNoDelay(true);
}

void TaskMotors(void * pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();

    if (homingCycleRequested) {
      homingCycleRequested = false;
      doHomingCycleInternal();
    }

    if (actuatorsEnabled && !holdActive && (machineState == STATE_IDLE || machineState == STATE_RUN || machineState == STATE_JOG)) {
      MotionBlock blk;
      if (popBlock(blk)) {
        executeBlock(blk);
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.printf("\r\nGrbl %s ['$' for help]\r\n", FW_VERSION);

  stateMutex = xSemaphoreCreateMutex();
  initMotionHardware();
  loadNVS();
  initPlanner();
  initGCodeParser();
  initNetwork();

  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&wdt_cfg);
  esp_task_wdt_add(NULL);

  lastCommTimeMs = millis();
  xTaskCreatePinnedToCore(TaskMotors, "TaskMotors", 8192, NULL, 5, &TaskMotorsHandle, 1);
}

static void handleRealtimeChar(Stream& out, char c) {
  lastCommTimeMs = millis();
  if (c == '?') {
    refreshInputs();
    String pn = "";
    if (ax[0].limitMinTriggered) pn += "X";
    if (ax[1].limitMinTriggered) pn += "Y";
    if (ax[2].limitMinTriggered) pn += "Z";
    if (ax[3].limitMinTriggered) pn += "W";
    if (probeActive()) pn += "P";

    out.printf("<%s|MPos:%.3f,%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f,%.3f|FS:%.0f,%.0f%s%s>\r\n",
      grblStateStr(),
      ax[0].getMPosMm(), ax[1].getMPosMm(), ax[2].getMPosMm(), ax[3].getMPosMm(),
      ax[0].getWPosMm(), ax[1].getWPosMm(), ax[2].getWPosMm(), ax[3].getWPosMm(),
      currentFeedRateMmMin, currentSpindleRpm,
      (pn.length() > 0 ? "|Pn:" : ""), pn.c_str());
  } else if (c == '!') {
    if (machineState != STATE_ALARM && machineState != STATE_HOMING) {
      holdActive = true;
    }
  } else if (c == '~') {
    holdActive = false;
  } else if (c == 0x18) {
    stopAllMotion();
    clearAlarmState();
    out.printf("\r\nGrbl %s ['$' for help]\r\n", FW_VERSION);
  } else if (c == 0x85) {
    purgeJogBlocks();
  }
}

void loop() {
  esp_task_wdt_reset();

  if (estopTriggered) {
    estopTriggered = false;
    stopAllMotion();
    setActuatorsState(false);
    machineState = STATE_ALARM;
    Serial.print("\r\nALARM: E-Stop Fisico Disparado\r\n");
    if (client && client.connected()) client.print("\r\nALARM: E-Stop Fisico Disparado\r\n");
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '?' || c == '!' || c == '~' || c == 0x18 || c == 0x85) {
      handleRealtimeChar(Serial, c);
      continue;
    }
    lastCommTimeMs = millis();
    if (c == '\r' || c == '\n') {
      if (serialIdx > 0) {
        serialBuffer[serialIdx] = '\0';
        processGrblLine(Serial, serialBuffer);
        serialIdx = 0;
      }
    } else if (serialIdx < MAX_LINE_LEN - 1) {
      serialBuffer[serialIdx++] = c;
    }
  }

  if (!client || !client.connected()) {
    client = server.accept();
    if (client) {
      client.setNoDelay(true);
      lastCommTimeMs = millis();
      client.printf("\r\nGrbl %s ['$' for help]\r\n", FW_VERSION);
    }
  } else {
    while (client.available() > 0) {
      char c = (char)client.read();
      if (c == '?' || c == '!' || c == '~' || c == 0x18 || c == 0x85) {
        handleRealtimeChar(client, c);
        continue;
      }
      lastCommTimeMs = millis();
      if (c == '\r' || c == '\n') {
        if (tcpIdx > 0) {
          tcpBuffer[tcpIdx] = '\0';
          processGrblLine(client, tcpBuffer);
          tcpIdx = 0;
        }
      } else if (tcpIdx < MAX_LINE_LEN - 1) {
        tcpBuffer[tcpIdx++] = c;
      }
    }
  }
  delay(1);
}