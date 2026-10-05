#include "Planner.h"
#include <esp_task_wdt.h>

MotionBlock blockQueue[BLOCK_QUEUE_SIZE];
volatile int qHead = 0;
volatile int qTail = 0;
float plannedPosMm[AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

void initPlanner() {
  qHead = 0;
  qTail = 0;
  for (int i = 0; i < BLOCK_QUEUE_SIZE; i++) blockQueue[i].active = false;
  syncPlannedPosFromActual();
}

void syncPlannedPosFromActual() {
  for (int i = 0; i < AXIS_COUNT; i++) plannedPosMm[i] = ax[i].getMPosMm();
}

void purgeJogBlocks() {
  jogCancelEpoch = jogCancelEpoch + 1;
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    int idx = qTail;
    while (idx != qHead) {
      if (blockQueue[idx].active && blockQueue[idx].isJog) {
        blockQueue[idx].active = false;
      }
      idx = (idx + 1) % BLOCK_QUEUE_SIZE;
    }
    syncPlannedPosFromActual();
    xSemaphoreGive(stateMutex);
  }
}

static float junctionSpeed(const float u1[AXIS_COUNT], const float u2[AXIS_COUNT], float v1, float v2, float accelMmS2) {
  float dot = 0.0f;
  for (int i = 0; i < AXIS_COUNT; i++) dot += u1[i] * u2[i];
  dot = constrain(dot, -1.0f, 1.0f);

  if (dot >= 0.9999f) return min(v1, v2);
  if (dot <= -0.9999f) return 0.0f;

  float sinHalfTheta = sqrtf((1.0f + dot) * 0.5f);
  if (sinHalfTheta >= 0.9999f) return min(v1, v2);

  float rFactor = sinHalfTheta / (1.0f - sinHalfTheta);
  float vj = sqrtf(accelMmS2 * JUNCTION_DEVIATION_MM * rFactor);
  return min(min(v1, v2), vj);
}

static void plannerForwardPass() {
  int n = (qHead - qTail + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
  if (n == 0) return;
  int prev = qTail;
  for (int k = 1; k < n; k++) {
    int idx = (qTail + k) % BLOCK_QUEUE_SIZE;
    MotionBlock& prevB = blockQueue[prev];
    MotionBlock& b = blockQueue[idx];
    if (!b.active) break;
    if (prevB.active && !prevB.isDwell && !b.isDwell && !prevB.isSpindleCmd && !b.isSpindleCmd) {
      float maxEntry = sqrtf(prevB.exitSpeedMmS * prevB.exitSpeedMmS + 2.0f * b.accelMmS2 * b.distanceMm);
      if (b.entrySpeedMmS > maxEntry) b.entrySpeedMmS = maxEntry;
      if (b.entrySpeedMmS > b.cruiseSpeedMmS) b.entrySpeedMmS = b.cruiseSpeedMmS;
    }
    prev = idx;
  }
}

static void plannerBackwardPass() {
  int idx = (qHead - 1 + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
  int next = -1;
  while (idx != qTail) {
    MotionBlock& b = blockQueue[idx];
    if (!b.active) break;
    if (next >= 0) {
      MotionBlock& nb = blockQueue[next];
      if (!b.isDwell && !nb.isDwell && !b.isSpindleCmd && !nb.isSpindleCmd) {
        float maxEntry = sqrtf(nb.entrySpeedMmS * nb.entrySpeedMmS + 2.0f * b.accelMmS2 * b.distanceMm);
        if (b.exitSpeedMmS > nb.entrySpeedMmS) b.exitSpeedMmS = nb.entrySpeedMmS;
        if (b.entrySpeedMmS > maxEntry) b.entrySpeedMmS = maxEntry;
        if (b.entrySpeedMmS > b.cruiseSpeedMmS) b.entrySpeedMmS = b.cruiseSpeedMmS;
      }
    }
    next = idx;
    idx = (idx - 1 + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
  }
}

PlanResult planAndEnqueueBlock(float targetMm[AXIS_COUNT], float feedMmPm, bool isJog, bool isDwell, uint32_t dwellMs, bool isSpindle, bool spEnable, float spRpm, bool isProbe) {
  if (!isDwell && !isSpindle) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (!isfinite(targetMm[i])) return PLAN_INVALID;
      if (softLimitsEnabled) {
        if (targetMm[i] < -0.05f || targetMm[i] > (ax[i].maxTravelMm + 0.05f)) return PLAN_SOFT_LIMIT;
      }
    }
  }

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(25)) != pdTRUE) return PLAN_FULL;

  if (isJog) {
    int jogCount = 0;
    int cur = qTail;
    while (cur != qHead) {
      if (blockQueue[cur].active && blockQueue[cur].isJog) jogCount++;
      cur = (cur + 1) % BLOCK_QUEUE_SIZE;
    }
    if (jogCount >= 1) {
      xSemaphoreGive(stateMutex);
      return PLAN_FULL;
    }
  }

  int nextHead = (qHead + 1) % BLOCK_QUEUE_SIZE;
  if (nextHead == qTail) {
    xSemaphoreGive(stateMutex);
    return PLAN_FULL;
  }

  MotionBlock& blk = blockQueue[qHead];
  memset(&blk, 0, sizeof(MotionBlock));
  blk.isJog = isJog;
  blk.isDwell = isDwell;
  blk.dwellMs = dwellMs;
  blk.isSpindleCmd = isSpindle;
  blk.spindleEnable = spEnable;
  blk.spindleRpm = spRpm;
  blk.isProbe = isProbe;

  if (isDwell || isSpindle) {
    blk.active = true;
    qHead = nextHead;
    xSemaphoreGive(stateMutex);
    return PLAN_OK;
  }

  float feedMmPs = max(feedMmPm, 1.0f) / 60.0f;
  float sumSq = 0.0f;
  int dominantAxis = 0;

  for (int i = 0; i < AXIS_COUNT; i++) {
    float d = targetMm[i] - plannedPosMm[i];
    blk.dirPos[i] = (d >= 0.0f);
    blk.deltaSteps[i] = (uint32_t)llabs(lroundf(d * safeSpm((AxisId)i)));
    blk.targetSteps[i] = lroundf(targetMm[i] * safeSpm((AxisId)i));
    if (blk.deltaSteps[i] > blk.maxSteps) {
      blk.maxSteps = blk.deltaSteps[i];
      dominantAxis = i;
    }
    sumSq += d * d;
  }

  blk.distanceMm = sqrtf(sumSq);
  blk.accelMmS2 = ax[dominantAxis].accelMmSec2;

  if (blk.distanceMm < 0.001f || blk.maxSteps == 0) {
    for (int i = 0; i < AXIS_COUNT; i++) plannedPosMm[i] = targetMm[i];
    xSemaphoreGive(stateMutex);
    return PLAN_OK;
  }

  for (int i = 0; i < AXIS_COUNT; i++) blk.unitVec[i] = (targetMm[i] - plannedPosMm[i]) / blk.distanceMm;

  float maxVectorSpeed = feedMmPs;
  for (int i = 0; i < AXIS_COUNT; i++) {
    float share = fabsf(targetMm[i] - plannedPosMm[i]) / blk.distanceMm;
    if (share < 1e-6f) continue;
    float axisCap = (ax[i].maxRateMmMin / 60.0f) / share;
    if (axisCap < maxVectorSpeed) maxVectorSpeed = axisCap;
  }
  blk.cruiseSpeedMmS = maxVectorSpeed;

  int prevIdx = (qHead - 1 + BLOCK_QUEUE_SIZE) % BLOCK_QUEUE_SIZE;
  if (prevIdx != qTail && blockQueue[prevIdx].active && !blockQueue[prevIdx].isDwell && !blockQueue[prevIdx].isSpindleCmd) {
    MotionBlock& prev = blockQueue[prevIdx];
    float vj = junctionSpeed(prev.unitVec, blk.unitVec, prev.cruiseSpeedMmS, blk.cruiseSpeedMmS, blk.accelMmS2);
    blk.entrySpeedMmS = min(vj, blk.cruiseSpeedMmS);
    prev.exitSpeedMmS = blk.entrySpeedMmS;
  } else {
    blk.entrySpeedMmS = 0.0f;
  }
  blk.exitSpeedMmS = 0.0f;

  blk.active = true;
  for (int i = 0; i < AXIS_COUNT; i++) plannedPosMm[i] = targetMm[i];
  qHead = nextHead;

  plannerBackwardPass();
  plannerForwardPass();
  xSemaphoreGive(stateMutex);
  return PLAN_OK;
}

bool popBlock(MotionBlock& out) {
  bool found = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    while (qHead != qTail) {
      if (blockQueue[qTail].active) {
        out = blockQueue[qTail];
        blockQueue[qTail].active = false;
        qTail = (qTail + 1) % BLOCK_QUEUE_SIZE;
        found = true;
        break;
      }
      qTail = (qTail + 1) % BLOCK_QUEUE_SIZE;
    }
    xSemaphoreGive(stateMutex);
  }
  return found;
}

bool waitQueueSpace(Stream* client, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint32_t epoch = stopEpoch;

  while (millis() - t0 < timeoutMs) {
    esp_task_wdt_reset();
    if (stopEpoch != epoch || estopTriggered || machineState == STATE_ALARM) return false;

    bool space = false;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      space = ((qHead + 1) % BLOCK_QUEUE_SIZE) != qTail;
      xSemaphoreGive(stateMutex);
    }
    if (space) return true;

    if (client && client->available() > 0) {
      int avail = client->available();
      for (int b = 0; b < avail; b++) {
        char c = (char)client->peek();
        if (c == '?' || c == '!' || c == '~' || c == 0x18 || c == 0x85) {
          client->read();
          lastCommTimeMs = millis();
          if (c == '?') {
            refreshInputs();
            String pn = "";
            if (ax[0].limitMinTriggered) pn += "X";
            if (ax[1].limitMinTriggered) pn += "Y";
            if (ax[2].limitMinTriggered) pn += "Z";
            if (ax[3].limitMinTriggered) pn += "W";
            if (probeActive()) pn += "P";

            client->printf("<%s|MPos:%.3f,%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f,%.3f|FS:%.0f,%.0f%s%s>\r\n",
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
            client->printf("\r\nGrbl %s ['$' for help]\r\n", FW_VERSION);
            return false;
          } else if (c == 0x85) {
            purgeJogBlocks();
          }
        } else {
          break;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}