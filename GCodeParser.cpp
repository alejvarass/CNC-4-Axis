#include "GCodeParser.h"
#include <Preferences.h>
#include <esp_task_wdt.h>

extern Preferences prefs;
static bool gcode_is_relative = false;
static bool gcode_is_inches = false;
static bool gcode_arc_relative = true;
static float gcode_feed_rate_mmpm = 600.0f;
static int gcode_motion_mode = 0;

void initGCodeParser() {
  gcode_is_relative = false;
  gcode_is_inches = false;
  gcode_arc_relative = true;
  gcode_feed_rate_mmpm = 600.0f;
  gcode_motion_mode = 0;
}

static bool nextWord(char*& p, char& letter, float& val) {
  while (*p == ' ' || *p == '\t') p++;
  if (!*p || !isalpha((unsigned char)*p)) return false;
  letter = *p++;
  char* endPtr = p;
  val = strtof(p, &endPtr);
  p = endPtr;
  return true;
}

void printAllGrblSettings(Stream& out) {
  for (int i = 0; i < AXIS_COUNT; i++) {
    out.printf("$%d=%.3f\r\n", 100 + i, ax[i].stepsPerMm);
    out.printf("$%d=%.1f\r\n", 110 + i, ax[i].maxRateMmMin);
    out.printf("$%d=%.1f\r\n", 120 + i, ax[i].accelMmSec2);
    out.printf("$%d=%.2f\r\n", 130 + i, ax[i].maxTravelMm);
    out.printf("$%d=%.3f\r\n", 40 + i, ax[i].backlashMm);
  }
  out.printf("$3=%d\r\n", (ax[0].dirInvert | (ax[1].dirInvert << 1) | (ax[2].dirInvert << 2) | (ax[3].dirInvert << 3)));
  out.printf("$20=%d\r\n", softLimitsEnabled ? 1 : 0);
  out.printf("$21=%d\r\n", dualLimitsEnabled ? 1 : 0);
  out.printf("$29=%d\r\n", probeEnabled ? 1 : 0);
  prefs.begin("cnc_grbl", true);
  out.printf("$30=%d\r\n", (int)prefs.getUChar("axis_mask", 15));
  prefs.end();
  out.printf("$24=%.1f\r\n", ax[0].homingFeedRateMmMin);
  out.printf("$25=%.1f\r\n", ax[0].homingSeekRateMmMin);
  out.printf("$27=%.2f\r\n", ax[0].homingPulloffMm);
  out.print("ok\r\n");
}

static bool writeGrblSetting(int param, float val) {
  if (param >= 100 && param <= 103) {
    int i = param - 100;
    if (val < 0.1f || val > 100000.0f) return false;
    ax[i].stepsPerMm = val;
    prefs.begin("cnc_grbl", false);
    prefs.putFloat((String(AXIS_CHARS[i]) + "_spm").c_str(), val);
    prefs.end();
    return true;
  }
  if (param >= 110 && param <= 113) {
    int i = param - 110;
    if (val < 1.0f || val > 50000.0f) return false;
    ax[i].maxRateMmMin = val;
    prefs.begin("cnc_grbl", false);
    prefs.putFloat((String(AXIS_CHARS[i]) + "_rate").c_str(), val);
    prefs.end();
    return true;
  }
  if (param >= 120 && param <= 123) {
    int i = param - 120;
    if (val < 1.0f || val > 5000.0f) return false;
    ax[i].accelMmSec2 = val;
    prefs.begin("cnc_grbl", false);
    prefs.putFloat((String(AXIS_CHARS[i]) + "_acc").c_str(), val);
    prefs.end();
    return true;
  }
  if (param >= 130 && param <= 133) {
    int i = param - 130;
    if (val < 1.0f || val > 2000.0f) return false;
    ax[i].maxTravelMm = val;
    prefs.begin("cnc_grbl", false);
    prefs.putFloat((String(AXIS_CHARS[i]) + "_max").c_str(), val);
    prefs.end();
    return true;
  }
  if (param >= 40 && param <= 43) {
    int i = param - 40;
    if (val < 0.0f || val > 10.0f) return false;
    ax[i].backlashMm = val;
    prefs.begin("cnc_grbl", false);
    prefs.putFloat((String(AXIS_CHARS[i]) + "_bl").c_str(), val);
    prefs.end();
    return true;
  }
  if (param == 3) {
    uint8_t mask = (uint8_t)val;
    for (int i = 0; i < AXIS_COUNT; i++) {
      ax[i].dirInvert = (mask >> i) & 0x01;
      prefs.begin("cnc_grbl", false);
      prefs.putUChar((String(AXIS_CHARS[i]) + "_dir").c_str(), ax[i].dirInvert);
      prefs.end();
    }
    return true;
  }
  if (param == 20) {
    softLimitsEnabled = ((int)val == 1);
    prefs.begin("cnc_grbl", false);
    prefs.putUChar("soft_lim", softLimitsEnabled ? 1 : 0);
    prefs.end();
    return true;
  }
  if (param == 21) {
    dualLimitsEnabled = ((int)val == 1);
    prefs.begin("cnc_grbl", false);
    prefs.putUChar("dual_lim", dualLimitsEnabled ? 1 : 0);
    prefs.end();
    return true;
  }
  if (param == 29) {
    probeEnabled = ((int)val == 1);
    prefs.begin("cnc_grbl", false);
    prefs.putUChar("probe_en", probeEnabled ? 1 : 0);
    prefs.end();
    return true;
  }
  if (param == 30) {
    uint8_t mask = (uint8_t)val;
    prefs.begin("cnc_grbl", false);
    prefs.putUChar("axis_mask", mask);
    prefs.end();
    return true;
  }
  if (param == 24) {
    for (int i = 0; i < AXIS_COUNT; i++) ax[i].homingFeedRateMmMin = val;
    prefs.begin("cnc_grbl", false);
    prefs.putFloat("h_feed", val);
    prefs.end();
    return true;
  }
  if (param == 25) {
    for (int i = 0; i < AXIS_COUNT; i++) ax[i].homingSeekRateMmMin = val;
    prefs.begin("cnc_grbl", false);
    prefs.putFloat("h_seek", val);
    prefs.end();
    return true;
  }
  if (param == 27) {
    for (int i = 0; i < AXIS_COUNT; i++) ax[i].homingPulloffMm = val;
    prefs.begin("cnc_grbl", false);
    prefs.putFloat("h_pull", val);
    prefs.end();
    return true;
  }
  return false;
}

static inline void printGrblPlanError(Stream& out, PlanResult pr) {
  switch (pr) {
    case PLAN_FULL: out.print("error:1\r\n"); break;
    case PLAN_SOFT_LIMIT: out.print("error:15\r\n"); break;
    case PLAN_NOT_HOMED: out.print("error:9\r\n"); break;
    default: out.print("error:2\r\n"); break;
  }
}

static bool isAngleBetween(float a, float a0, float a1, bool cw) {
  if (cw) {
    if (a0 >= a1) return (a <= a0 && a >= a1);
    else return (a <= a0 || a >= a1);
  } else {
    if (a1 >= a0) return (a >= a0 && a <= a1);
    else return (a >= a0 || a <= a1);
  }
}

static bool checkArcBoundingBox(float cx, float cy, float r, float a0, float a1, bool cw, float x0, float y0, float x1, float y1) {
  float minX = min(x0, x1);
  float maxX = max(x0, x1);
  float minY = min(y0, y1);
  float maxY = max(y0, y1);

  if (fabsf(a0 - a1) < 1e-4f) {
    minX = cx - r; maxX = cx + r;
    minY = cy - r; maxY = cy + r;
  } else {
    if (isAngleBetween(0.0f, a0, a1, cw)) maxX = max(maxX, cx + r);
    if (isAngleBetween((float)M_PI / 2.0f, a0, a1, cw)) maxY = max(maxY, cy + r);
    if (isAngleBetween((float)M_PI, a0, a1, cw)) minX = min(minX, cx - r);
    if (isAngleBetween(3.0f * (float)M_PI / 2.0f, a0, a1, cw)) minY = min(minY, cy - r);
  }

  if (minX < -0.05f || maxX > ax[0].maxTravelMm + 0.05f) return false;
  if (minY < -0.05f || maxY > ax[1].maxTravelMm + 0.05f) return false;
  return true;
}

void processGrblLine(Stream& out, char* rawLine) {
  while (*rawLine == ' ' || *rawLine == '\t') rawLine++;
  if (strncmp(rawLine, "$WIFI=", 6) == 0) {
    char* comma = strchr(rawLine + 6, ',');
    if (comma) {
      *comma = '\0';
      String ssid = String(rawLine + 6);
      String pass = String(comma + 1);
      pass.trim();
      prefs.begin("cnc_grbl", false);
      prefs.putString("w_ssid", ssid);
      prefs.putString("w_pass", pass);
      prefs.end();
      out.print("[MSG:WiFi Guardado. Reinicie para conectar]\r\nok\r\n");
      return;
    }
    out.print("error:3\r\n");
    return;
  }

  if (strncmp(rawLine, "$AP=", 4) == 0) {
    String ap_pass = String(rawLine + 4);
    ap_pass.trim();
    if (ap_pass.length() >= 8) {
      prefs.begin("cnc_grbl", false);
      prefs.putString("ap_pass", ap_pass);
      prefs.end();
      out.print("[MSG:Clave SoftAP Guardada]\r\nok\r\n");
      return;
    }
    out.print("error:3\r\n");
    return;
  }

  char line[MAX_LINE_LEN];
  size_t o = 0;
  bool inParen = false;

  for (size_t k = 0; rawLine[k] && o < MAX_LINE_LEN - 1; k++) {
    char c = rawLine[k];
    if (inParen) { if (c == ')') inParen = false; continue; }
    if (c == '(') { inParen = true; continue; }
    if (c == ';') break;
    line[o++] = (char)toupper((unsigned char)c);
  }
  while (o > 0 && (line[o - 1] == ' ' || line[o - 1] == '\t')) o--;
  line[o] = '\0';

  if (line[0] == '\0' || line[0] == '%') { out.print("ok\r\n"); return; }

  if (line[0] == '$') {
    if (line[1] == '\0' || line[1] == '$') {
      printAllGrblSettings(out);
      return;
    }
    if (line[1] == '#') {
      out.printf("[G54:0.000,0.000,0.000,0.000]\r\n[G92:%.3f,%.3f,%.3f,%.3f]\r\n[PRB:%.3f,%.3f,%.3f,%.3f:%d]\r\nok\r\n",
                 (float)ax[0].wcoSteps / safeSpm(AXIS_X), (float)ax[1].wcoSteps / safeSpm(AXIS_Y),
                 (float)ax[2].wcoSteps / safeSpm(AXIS_Z), (float)ax[3].wcoSteps / safeSpm(AXIS_W),
                 lastProbePosMm[0], lastProbePosMm[1], lastProbePosMm[2], lastProbePosMm[3], probeTriggered ? 1 : 0);
      return;
    }
    if (line[1] == 'G') {
      out.printf("[GC:G%d G54 %s %s %s M%d S%.0f F%.0f]\r\nok\r\n",
                 gcode_motion_mode,
                 gcode_is_inches ? "G20" : "G21",
                 gcode_is_relative ? "G91" : "G90",
                 gcode_arc_relative ? "G91.1" : "G90.1",
                 spindleRunning ? 3 : 5,
                 currentSpindleRpm,
                 gcode_feed_rate_mmpm);
      return;
    }
    if (isdigit((unsigned char)line[1])) {
      char* eq = strchr(line, '=');
      if (eq) {
        *eq = '\0';
        int p = atoi(line + 1);
        float val = strtof(eq + 1, NULL);
        if (writeGrblSetting(p, val)) out.print("ok\r\n");
        else out.print("error:3\r\n");
        return;
      }
    }
    if (line[1] == 'I') {
      out.printf("[VER:GRBL v1.1h XYZW / ESP32 v%s]\r\n[OPT:V,W,DUAL_LIMITS,PROBE,SPINDLE_PWM,NVS_SAVE]\r\nok\r\n", FW_VERSION);
      return;
    }
    if (line[1] == 'H') {
      if (!actuatorsEnabled || (machineState != STATE_IDLE && machineState != STATE_ALARM)) {
        out.print("error:8\r\n");
        return;
      }
      homingCycleRequested = true;
      out.print("ok\r\n");
      return;
    }
    if (line[1] == 'X') {
      if (estopActive()) { out.print("error:4\r\n"); return; }
      if (!clearAlarmState()) { out.print("error:9\r\n"); return; }
      setActuatorsState(true);
      out.print("[MSG:Caution: Unlocked]\r\nok\r\n");
      return;
    }
    if (strncmp(line, "$J=", 3) == 0) {
      if (machineState == STATE_ALARM) { out.print("error:9\r\n"); return; }
      float target[AXIS_COUNT];
      for (int i = 0; i < AXIS_COUNT; i++) target[i] = plannedPosMm[i];
      bool jog_rel = gcode_is_relative;
      bool jog_inches = gcode_is_inches;
      float feed = gcode_feed_rate_mmpm;

      char* p = line + 3;
      char w; float val;
      while (nextWord(p, w, val)) {
        if (w == 'G') {
          int g = (int)val;
          if (g == 90) jog_rel = false;
          else if (g == 91) jog_rel = true;
          else if (g == 20) jog_inches = true;
          else if (g == 21) jog_inches = false;
        } else if (w == 'F') feed = jog_inches ? (val * 25.4f) : val;
      }
      p = line + 3;
      while (nextWord(p, w, val)) {
        for (int i = 0; i < AXIS_COUNT; i++) {
          if (w == AXIS_CHARS[i]) {
            float mm = jog_inches ? (val * 25.4f) : val;
            target[i] = jog_rel ? (plannedPosMm[i] + mm) : (mm + ((float)ax[i].wcoSteps / safeSpm((AxisId)i)));
          }
        }
      }
      PlanResult pr = planAndEnqueueBlock(target, feed, true, false, 0);
      if (pr == PLAN_OK) out.print("ok\r\n");
      else printGrblPlanError(out, pr);
      return;
    }
    out.print("error:3\r\n");
    return;
  }

  int motionMode = -1;
  bool isProbeMode = false;
  bool probeErrorOnNoContact = true;
  bool wcoLine = false;
  bool dwellLine = false;
  bool g53Line = false;
  bool lWordSeen = false;
  int lWord = -1;
  float dwellSec = -1.0f;
  float sVal = -1.0f;
  int spindleCmd = 0;

  char* p = line;
  char w; float val;
  while (nextWord(p, w, val)) {
    if (w == 'G') {
      int g = (int)val;
      if (g >= 0 && g <= 3) { motionMode = g; gcode_motion_mode = g; }
      else if (g == 4) dwellLine = true;
      else if (g == 10 || g == 92) wcoLine = true;
      else if (g == 38) {
        isProbeMode = true;
        probeErrorOnNoContact = (fabsf(val - 38.2f) < 0.05f);
      }
      else if (g == 90) {
        if (fabsf(val - 90.1f) < 0.05f) gcode_arc_relative = false;
        else gcode_is_relative = false;
      }
      else if (g == 91) {
        if (fabsf(val - 91.1f) < 0.05f) gcode_arc_relative = true;
        else gcode_is_relative = true;
      }
      else if (g == 20) gcode_is_inches = true;
      else if (g == 21) gcode_is_inches = false;
      else if (g == 53) g53Line = true;
      else if (g == 18 || g == 19 || g == 28 || g == 30) { out.print("error:20\r\n"); return; }
    } else if (w == 'M') {
      int m = (int)val;
      if (m == 3 || m == 4) spindleCmd = m;
      else if (m == 5) spindleCmd = 5;
    } else if (w == 'L') { lWordSeen = true; lWord = (int)val; }
    else if (w == 'P') dwellSec = val;
    else if (w == 'S') sVal = val;
    else if (w == 'F') gcode_feed_rate_mmpm = gcode_is_inches ? (val * 25.4f) : val;
  }

  float target[AXIS_COUNT];
  for (int i = 0; i < AXIS_COUNT; i++) target[i] = plannedPosMm[i];
  bool axisSeen = false;
  float iVal = 0.0f, jVal = 0.0f;
  bool hasI = false, hasJ = false;

  p = line;
  while (nextWord(p, w, val)) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (w == AXIS_CHARS[i]) {
        float mm = gcode_is_inches ? (val * 25.4f) : val;
        if (wcoLine) {
          ax[i].wcoSteps = lroundf((plannedPosMm[i] - mm) * safeSpm((AxisId)i));
        } else {
          target[i] = gcode_is_relative ? (plannedPosMm[i] + mm) : (g53Line ? mm : (mm + ((float)ax[i].wcoSteps / safeSpm((AxisId)i))));
          axisSeen = true;
        }
      }
    }
    if (w == 'I') { iVal = gcode_is_inches ? (val * 25.4f) : val; hasI = true; }
    if (w == 'J') { jVal = gcode_is_inches ? (val * 25.4f) : val; hasJ = true; }
  }

  if (axisSeen && !wcoLine && softLimitsEnabled) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (target[i] < -0.05f || target[i] > (ax[i].maxTravelMm + 0.05f)) {
        out.print("error:15\r\n");
        return;
      }
    }
  }

  if (wcoLine) {
    if (lWordSeen && lWord != 20 && lWord != 0) { out.print("error:2\r\n"); return; }
    out.print("ok\r\n");
    return;
  }

  if (sVal >= 0.0f) setSpindleSpeed(sVal);

  if (spindleCmd > 0) {
    float dummyTarget[AXIS_COUNT] = {0};
    PlanResult pr = planAndEnqueueBlock(dummyTarget, 0, false, false, 0, true, (spindleCmd != 5), (sVal >= 0.0f ? sVal : currentSpindleRpm));
    while (pr == PLAN_FULL) {
      if (!waitQueueSpace(&out, 10000)) { out.print("error:1\r\n"); return; }
      pr = planAndEnqueueBlock(dummyTarget, 0, false, false, 0, true, (spindleCmd != 5), (sVal >= 0.0f ? sVal : currentSpindleRpm));
    }
  }

  if (dwellLine) {
    if (dwellSec < 0.0f) { out.print("error:2\r\n"); return; }
    PlanResult pr = planAndEnqueueBlock(target, 0.0f, false, true, (uint32_t)(dwellSec * 1000.0f));
    while (pr == PLAN_FULL) {
      if (!waitQueueSpace(&out, 10000)) { out.print("error:1\r\n"); return; }
      pr = planAndEnqueueBlock(target, 0.0f, false, true, (uint32_t)(dwellSec * 1000.0f));
    }
    if (pr == PLAN_OK) out.print("ok\r\n");
    else printGrblPlanError(out, pr);
    return;
  }

  if (isProbeMode) {
    if (!probeEnabled) {
      out.print("error:2\r\n");
      return;
    }
    if (probeActive()) { out.print("error:4\r\n"); return; }
    probeTriggered = false;
    PlanResult pr = planAndEnqueueBlock(target, gcode_feed_rate_mmpm, false, false, 0, false, false, 0.0f, true);
    while (pr == PLAN_FULL) {
      if (!waitQueueSpace(&out, 10000)) { out.print("error:1\r\n"); return; }
      pr = planAndEnqueueBlock(target, gcode_feed_rate_mmpm, false, false, 0, false, false, 0.0f, true);
    }
    if (pr != PLAN_OK) { printGrblPlanError(out, pr); return; }

    while (machineState == STATE_RUN) {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    out.printf("[PRB:%.3f,%.3f,%.3f,%.3f:%d]\r\n",
               lastProbePosMm[0], lastProbePosMm[1], lastProbePosMm[2], lastProbePosMm[3], probeTriggered ? 1 : 0);
    if (!probeTriggered && probeErrorOnNoContact) {
      out.print("error:9\r\n");
    } else {
      out.print("ok\r\n");
    }
    return;
  }

  if ((motionMode >= 0) || axisSeen) {
    if (motionMode < 0) motionMode = gcode_motion_mode;

    if (motionMode == 2 || motionMode == 3) {
      if (!hasI && !hasJ) { out.print("error:22\r\n"); return; }
      float snapX = plannedPosMm[0], snapY = plannedPosMm[1];
      float cx = gcode_arc_relative ? (snapX + iVal) : iVal;
      float cy = gcode_arc_relative ? (snapY + jVal) : jVal;
      float r0 = hypotf(snapX - cx, snapY - cy);
      float r1 = hypotf(target[0] - cx, target[1] - cy);
      if (r0 < 0.01f || fabsf(r0 - r1) > 0.1f) { out.print("error:22\r\n"); return; }

      float a0 = atan2f(snapY - cy, snapX - cx);
      float a1 = atan2f(target[1] - cy, target[0] - cx);
      if (a0 < 0) a0 += 2.0f * (float)M_PI;
      if (a1 < 0) a1 += 2.0f * (float)M_PI;

      bool cw = (motionMode == 2);
      if (softLimitsEnabled && !checkArcBoundingBox(cx, cy, r0, a0, a1, cw, snapX, snapY, target[0], target[1])) {
        out.print("error:15\r\n");
        return;
      }

      float sweep = a1 - a0;
      if (cw) {
        if (sweep >= 0.0f) sweep -= 2.0f * (float)M_PI;
      } else {
        if (sweep <= 0.0f) sweep += 2.0f * (float)M_PI;
      }

      int nSeg = (int)ceilf(r0 * fabsf(sweep) / 1.0f);
      if (nSeg < 2) nSeg = 2;
      for (int s = 1; s <= nSeg; s++) {
        float t = (float)s / (float)nSeg;
        float ang = a0 + sweep * t;
        float seg[AXIS_COUNT] = {
          cx + r0 * cosf(ang),
          cy + r0 * sinf(ang),
          plannedPosMm[2] + (target[2] - plannedPosMm[2]) * t,
          plannedPosMm[3] + (target[3] - plannedPosMm[3]) * t
        };
        if (s == nSeg) { seg[0] = target[0]; seg[1] = target[1]; }

        PlanResult pr = planAndEnqueueBlock(seg, gcode_feed_rate_mmpm, false, false, 0);
        while (pr == PLAN_FULL) {
          if (!waitQueueSpace(&out, 10000)) { out.print("error:1\r\n"); return; }
          pr = planAndEnqueueBlock(seg, gcode_feed_rate_mmpm, false, false, 0);
        }
        if (pr != PLAN_OK) { printGrblPlanError(out, pr); return; }
      }
      out.print("ok\r\n");
      return;
    }

    float feed = (motionMode == 0) ? 100000.0f : gcode_feed_rate_mmpm;
    PlanResult pr = planAndEnqueueBlock(target, feed, false, false, 0);
    while (pr == PLAN_FULL) {
      if (!waitQueueSpace(&out, 10000)) { out.print("error:1\r\n"); return; }
      pr = planAndEnqueueBlock(target, feed, false, false, 0);
    }
    if (pr == PLAN_OK) out.print("ok\r\n");
    else printGrblPlanError(out, pr);
    return;
  }

  out.print("ok\r\n");
}