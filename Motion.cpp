#include "Motion.h"
#include "Planner.h"
#include <esp_task_wdt.h>
#include <soc/gpio_struct.h>
#include <rom/gpio.h>

AxisHW hw[AXIS_COUNT] = {
  {23, 22, 34, 35}, // X: STEP, DIR, LIMIT_MIN, LIMIT_MAX
  {21, 17, 25, 36}, // Y
  {16, 4,  14, 39}, // Z
  {13, 15, 18, 33}  // W
};

AxisSettings ax[AXIS_COUNT];
volatile MachineState machineState = STATE_ALARM;
volatile bool actuatorsEnabled = false;
volatile bool estopTriggered = false;
bool estopActiveHigh = true;
volatile bool dualLimitsEnabled = false;
volatile bool probeEnabled = true;
volatile bool softLimitsEnabled = false;
volatile uint32_t stopEpoch = 0;
volatile uint32_t jogCancelEpoch = 0;
volatile bool homingCycleRequested = false;
volatile float currentFeedRateMmMin = 0.0f;
volatile float currentSpindleRpm = 0.0f;
volatile bool spindleRunning = false;
volatile bool holdActive = false;
volatile uint32_t lastCommTimeMs = 0;
volatile bool probeTriggered = false;
float lastProbePosMm[AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

static bool lastDirKnown[AXIS_COUNT] = {false, false, false, false};
static bool lastDirPos[AXIS_COUNT] = {true, true, true, true};

SemaphoreHandle_t stateMutex = NULL;
hw_timer_t * stepTimer = NULL;
TaskHandle_t TaskMotorsHandle = NULL;

void IRAM_ATTR onStepTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (TaskMotorsHandle != NULL) {
    vTaskNotifyGiveFromISR(TaskMotorsHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR onEstopISR() {
  GPIO.out_w1ts = (1 << PIN_ENABLE_ACTUATORS);
  gpio_matrix_out(PIN_SPINDLE, 0x100, false, false);
  GPIO.out_w1tc = (1 << PIN_SPINDLE);
  spindleRunning = false;
  actuatorsEnabled = false;
  estopTriggered = true;
}

void waitStepHardwareTimer(uint32_t delayUs) {
  if (delayUs < MIN_STEP_US) delayUs = MIN_STEP_US;
  if (delayUs > 20000) delayUs = 20000;
  timerWrite(stepTimer, 0);
  timerAlarm(stepTimer, (uint64_t)delayUs, false, 0);
  uint32_t timeoutTicks = pdMS_TO_TICKS(delayUs / 500 + 10);
  ulTaskNotifyTake(pdTRUE, timeoutTicks);
}

void initMotionHardware() {
  pinMode(PIN_ENABLE_ACTUATORS, OUTPUT);
  digitalWrite(PIN_ENABLE_ACTUATORS, HIGH);

  ledcAttach(PIN_SPINDLE, SPINDLE_PWM_FREQ, SPINDLE_PWM_RES);
  ledcWrite(PIN_SPINDLE, 0);

  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(PIN_ESTOP, onEstopISR, estopActiveHigh ? RISING : FALLING);

  pinMode(PIN_PROBE, INPUT_PULLUP);

  for (int i = 0; i < AXIS_COUNT; i++) {
    pinMode(hw[i].pinStep, OUTPUT);
    pinMode(hw[i].pinDir, OUTPUT);
    pinMode(hw[i].pinLimitMin, INPUT_PULLUP);
    pinMode(hw[i].pinLimitMax, INPUT);
    digitalWrite(hw[i].pinStep, LOW);
  }

  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);
}

bool estopActive() {
  return digitalRead(PIN_ESTOP) == (estopActiveHigh ? HIGH : LOW);
}

bool probeActive() {
  if (!probeEnabled) return false;
  return digitalRead(PIN_PROBE) == LOW;
}

void setActuatorsState(bool enable) {
  if (enable && estopActive()) {
    enable = false;
    estopTriggered = true;
  }
  actuatorsEnabled = enable;
  digitalWrite(PIN_ENABLE_ACTUATORS, enable ? LOW : HIGH);
  if (enable) {
    ledcAttach(PIN_SPINDLE, SPINDLE_PWM_FREQ, SPINDLE_PWM_RES);
    setSpindleSpeed(currentSpindleRpm);
  }
}

float safeSpm(AxisId a) {
  return (ax[a].stepsPerMm > 0.1f) ? ax[a].stepsPerMm : 568.0f;
}

void setAxisDirection(AxisId a, bool isForward) {
  bool level = isForward ? HIGH : LOW;
  if (ax[a].dirInvert) level = !level;
  digitalWrite(hw[a].pinDir, level ? HIGH : LOW);
}

void pulseAxis(AxisId a, bool isForward) {
  if (!actuatorsEnabled) return;
  digitalWrite(hw[a].pinStep, HIGH);
  delayMicroseconds(4);
  digitalWrite(hw[a].pinStep, LOW);

  if (isForward) ax[a].stepCount++;
  else ax[a].stepCount--;
}

void compensateBacklash(AxisId a, bool newDirPos, uint32_t stepUs) {
  float bl = ax[a].backlashMm;
  if (bl <= 0.0f) { lastDirKnown[a] = true; lastDirPos[a] = newDirPos; return; }
  if (lastDirKnown[a] && lastDirPos[a] == newDirPos) return;
  lastDirKnown[a] = true;
  lastDirPos[a] = newDirPos;
  uint32_t extra = (uint32_t)lroundf(bl * safeSpm(a));
  if (extra == 0 || extra > 20000) return;
  setAxisDirection(a, newDirPos);
  uint32_t epoch = stopEpoch;
  for (uint32_t i = 0; i < extra; i++) {
    esp_task_wdt_reset();
    if (!actuatorsEnabled || estopTriggered || stopEpoch != epoch || holdActive) return;
    digitalWrite(hw[a].pinStep, HIGH);
    delayMicroseconds(4);
    digitalWrite(hw[a].pinStep, LOW);
    waitStepHardwareTimer(stepUs);
  }
}

void setSpindleSpeed(float rpm, float maxRpm) {
  if (rpm >= 0.0f) currentSpindleRpm = rpm;
  if (spindleRunning) {
    float duty = constrain(currentSpindleRpm / maxRpm, 0.0f, 1.0f);
    ledcWrite(PIN_SPINDLE, (uint32_t)(duty * 255.0f));
  } else {
    ledcWrite(PIN_SPINDLE, 0);
  }
}

void setSpindleState(bool enable, float rpm) {
  spindleRunning = enable;
  if (rpm >= 0.0f) currentSpindleRpm = rpm;
  setSpindleSpeed(currentSpindleRpm);
}

void stopAllMotion() {
  stopEpoch = stopEpoch + 1;
  jogCancelEpoch = jogCancelEpoch + 1;
  holdActive = false;
  setSpindleState(false);
  currentFeedRateMmMin = 0.0f;

  for (int retry = 0; retry < 5; retry++) {
    if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      qHead = 0;
      qTail = 0;
      for (int i = 0; i < BLOCK_QUEUE_SIZE; i++) blockQueue[i].active = false;
      syncPlannedPosFromActual();
      xSemaphoreGive(stateMutex);
      break;
    }
  }

  for (int i = 0; i < AXIS_COUNT; i++) ax[i].isHomed = false;
  if (machineState != STATE_ALARM) machineState = STATE_IDLE;
}

bool clearAlarmState() {
  stopEpoch = stopEpoch + 1;
  jogCancelEpoch = jogCancelEpoch + 1;
  holdActive = false;
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    qHead = 0;
    qTail = 0;
    for (int i = 0; i < BLOCK_QUEUE_SIZE; i++) blockQueue[i].active = false;
    syncPlannedPosFromActual();
    machineState = STATE_IDLE;
    xSemaphoreGive(stateMutex);
    return true;
  }
  return false;
}

void refreshInputs() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    ax[i].limitMinTriggered = (digitalRead(hw[i].pinLimitMin) == HIGH);
    if (dualLimitsEnabled) {
      ax[i].limitMaxTriggered = (digitalRead(hw[i].pinLimitMax) == HIGH);
    } else {
      ax[i].limitMaxTriggered = false;
    }
  }
}

const char* grblStateStr() {
  if (holdActive || machineState == STATE_HOLD) return "Hold";
  switch (machineState) {
    case STATE_IDLE: return "Idle";
    case STATE_RUN: return "Run";
    case STATE_JOG: return "Jog";
    case STATE_HOMING: return "Home";
    case STATE_ALARM: return "Alarm";
    case STATE_CHECK: return "Check";
    default: return "Idle";
  }
}

static bool singleAxisHome(AxisId a) {
  uint32_t epoch = stopEpoch;
  setAxisDirection(a, false);
  compensateBacklash(a, false, MIN_STEP_US * 4);

  float seekDelayUs = 1e6f / ((ax[a].homingSeekRateMmMin / 60.0f) * safeSpm(a));
  float feedDelayUs = 1e6f / ((ax[a].homingFeedRateMmMin / 60.0f) * safeSpm(a));
  uint32_t maxSteps = (uint32_t)(ax[a].maxTravelMm * safeSpm(a) * 1.5f);
  uint32_t count = 0;
  bool hit = false;

  while (count < maxSteps) {
    esp_task_wdt_reset();
    if (stopEpoch != epoch || !actuatorsEnabled) return false;
    if (digitalRead(hw[a].pinLimitMin) == HIGH) { hit = true; break; }
    pulseAxis(a, false);
    waitStepHardwareTimer((uint32_t)seekDelayUs);
    count++;
  }
  if (!hit) return false;

  setAxisDirection(a, true);
  compensateBacklash(a, true, (uint32_t)seekDelayUs);
  uint32_t pulloffSteps = (uint32_t)lroundf(ax[a].homingPulloffMm * safeSpm(a));
  for (uint32_t i = 0; i < pulloffSteps; i++) {
    esp_task_wdt_reset();
    if (stopEpoch != epoch || !actuatorsEnabled) return false;
    pulseAxis(a, true);
    waitStepHardwareTimer((uint32_t)seekDelayUs);
  }

  setAxisDirection(a, false);
  compensateBacklash(a, false, (uint32_t)feedDelayUs);
  hit = false;
  count = 0;
  while (count < pulloffSteps * 2) {
    esp_task_wdt_reset();
    if (stopEpoch != epoch || !actuatorsEnabled) return false;
    if (digitalRead(hw[a].pinLimitMin) == HIGH) { hit = true; break; }
    pulseAxis(a, false);
    waitStepHardwareTimer((uint32_t)feedDelayUs);
    count++;
  }
  if (!hit) return false;

  setAxisDirection(a, true);
  compensateBacklash(a, true, (uint32_t)feedDelayUs);
  for (uint32_t i = 0; i < pulloffSteps; i++) {
    esp_task_wdt_reset();
    if (stopEpoch != epoch || !actuatorsEnabled) return false;
    pulseAxis(a, true);
    waitStepHardwareTimer((uint32_t)feedDelayUs);
  }

  ax[a].stepCount = 0;
  ax[a].isHomed = true;
  return true;
}

void doHomingCycleInternal() {
  machineState = STATE_HOMING;
  AxisId seq[AXIS_COUNT] = {AXIS_Z, AXIS_W, AXIS_X, AXIS_Y};
  bool success = true;

  for (int i = 0; i < AXIS_COUNT; i++) {
    if (!singleAxisHome(seq[i])) {
      success = false;
      break;
    }
  }

  syncPlannedPosFromActual();
  if (success) {
    machineState = STATE_IDLE;
  } else {
    machineState = STATE_ALARM;
    stopAllMotion();
  }
}

static inline int speedToStepUs(float speedMmS, float mmPerDominantStep) {
  if (speedMmS < 0.2f) speedMmS = 0.2f;
  float us = (mmPerDominantStep / speedMmS) * 1e6f;
  return (int)constrain(us, (float)MIN_STEP_US, 20000.0f);
}

bool executeBlock(const MotionBlock& blk) {
  if (!actuatorsEnabled) return false;
  uint32_t epoch = stopEpoch;
  uint32_t jogEpoch = jogCancelEpoch;

  if (blk.isSpindleCmd) {
    setSpindleState(blk.spindleEnable, blk.spindleRpm);
    return true;
  }

  if (blk.isDwell) {
    machineState = STATE_RUN;
    currentFeedRateMmMin = 0.0f;
    uint32_t t0 = millis();
    while (millis() - t0 < blk.dwellMs) {
      esp_task_wdt_reset();
      if (!actuatorsEnabled || stopEpoch != epoch) break;
      while (holdActive) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!actuatorsEnabled || stopEpoch != epoch) break;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (machineState != STATE_ALARM && !holdActive) machineState = STATE_IDLE;
    return true;
  }

  if (blk.maxSteps == 0) return true;

  machineState = blk.isJog ? STATE_JOG : STATE_RUN;
  long errAccum[AXIS_COUNT] = {0, 0, 0, 0};

  for (int i = 0; i < AXIS_COUNT; i++) {
    setAxisDirection((AxisId)i, blk.dirPos[i]);
    if (blk.deltaSteps[i] > 0) compensateBacklash((AxisId)i, blk.dirPos[i], MIN_STEP_US * 4);
  }

  float mmPerStep = blk.distanceMm / (float)blk.maxSteps;
  float a = blk.accelMmS2;
  float v_entry = max(blk.entrySpeedMmS, 0.2f);
  float v_cruise = max(blk.cruiseSpeedMmS, 0.2f);
  float v_exit = max(blk.exitSpeedMmS, 0.2f);

  float d_accel = (v_cruise * v_cruise - v_entry * v_entry) / (2.0f * a);
  float d_decel = (v_cruise * v_cruise - v_exit * v_exit) / (2.0f * a);
  if (d_accel < 0.0f) d_accel = 0.0f;
  if (d_decel < 0.0f) d_decel = 0.0f;

  if (d_accel + d_decel > blk.distanceMm) {
    v_cruise = sqrtf(max(0.2f, (2.0f * a * blk.distanceMm + v_entry * v_entry + v_exit * v_exit) / 2.0f));
    d_accel = (v_cruise * v_cruise - v_entry * v_entry) / (2.0f * a);
    d_decel = blk.distanceMm - d_accel;
  }

  uint32_t step_accel_end = (uint32_t)lroundf((d_accel / blk.distanceMm) * blk.maxSteps);
  uint32_t step_decel_start = blk.maxSteps - (uint32_t)lroundf((d_decel / blk.distanceMm) * blk.maxSteps);
  if (step_decel_start < step_accel_end) step_decel_start = step_accel_end;

  uint32_t step_resume_start = 0;
  float v_target_current = v_cruise;
  bool aborted = false;

  for (uint32_t step = 0; step < blk.maxSteps; step++) {
    if ((step & 0x7F) == 0) esp_task_wdt_reset();

    if (!actuatorsEnabled || stopEpoch != epoch) { aborted = true; break; }
    if (blk.isJog && jogCancelEpoch != jogEpoch) { aborted = true; break; }

    if (millis() - lastCommTimeMs > 400) {
      if (blk.isJog) {
        aborted = true;
        purgeJogBlocks();
        break;
      }
    }

    if (blk.isProbe) {
      if (probeActive()) {
        probeTriggered = true;
        for (int p_i = 0; p_i < AXIS_COUNT; p_i++) {
          lastProbePosMm[p_i] = ax[p_i].getMPosMm();
        }
        aborted = true;
        break;
      }
    }

    if (holdActive) {
      float v_current = max(0.2f, currentFeedRateMmMin / 60.0f);
      while (v_current > 0.2f && step < blk.maxSteps) {
        esp_task_wdt_reset();
        for (int i = 0; i < AXIS_COUNT; i++) {
          if (blk.deltaSteps[i] > 0) {
            if (!blk.dirPos[i] && digitalRead(hw[i].pinLimitMin) == HIGH) {
              machineState = STATE_ALARM; stopAllMotion(); aborted = true; break;
            }
            if (dualLimitsEnabled && blk.dirPos[i] && digitalRead(hw[i].pinLimitMax) == HIGH) {
              machineState = STATE_ALARM; stopAllMotion(); aborted = true; break;
            }
          }
        }
        if (aborted) break;

        v_current = max(0.2f, v_current - a * (mmPerStep / v_current));
        int holdDelayUs = speedToStepUs(v_current, mmPerStep);
        currentFeedRateMmMin = v_current * 60.0f;
        waitStepHardwareTimer(holdDelayUs);

        for (int i = 0; i < AXIS_COUNT; i++) {
          if (blk.deltaSteps[i] > 0) {
            errAccum[i] += blk.deltaSteps[i];
            if (errAccum[i] >= (long)blk.maxSteps) {
              errAccum[i] -= blk.maxSteps;
              pulseAxis((AxisId)i, blk.dirPos[i]);
            }
          }
        }
        step++;
      }

      currentFeedRateMmMin = 0.0f;
      while (holdActive) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!actuatorsEnabled || stopEpoch != epoch) { aborted = true; break; }
      }

      step_resume_start = step;
      v_entry = 0.2f;
      float d_remain = (float)(blk.maxSteps - step) * mmPerStep;

      if (d_remain > 0.01f) {
        float v_peak = sqrtf(max(0.2f, (2.0f * a * d_remain + v_entry * v_entry + v_exit * v_exit) / 2.0f));
        v_target_current = min(v_cruise, v_peak);
        d_accel = (v_target_current * v_target_current - v_entry * v_entry) / (2.0f * a);
        if (d_accel < 0.0f) d_accel = 0.0f;
        step_accel_end = step + (uint32_t)lroundf((d_accel / d_remain) * (blk.maxSteps - step));
        d_decel = (v_target_current * v_target_current - v_exit * v_exit) / (2.0f * a);
        if (d_decel < 0.0f) d_decel = 0.0f;
        step_decel_start = blk.maxSteps - (uint32_t)lroundf((d_decel / d_remain) * (blk.maxSteps - step));
        if (step_decel_start < step_accel_end) step_decel_start = step_accel_end;
      } else {
        step_accel_end = blk.maxSteps;
        step_decel_start = blk.maxSteps;
      }
    }
    if (aborted) break;

    for (int i = 0; i < AXIS_COUNT; i++) {
      if (blk.deltaSteps[i] > 0) {
        if (!blk.dirPos[i] && digitalRead(hw[i].pinLimitMin) == HIGH) {
          machineState = STATE_ALARM; stopAllMotion(); aborted = true; break;
        }
        if (dualLimitsEnabled && blk.dirPos[i] && digitalRead(hw[i].pinLimitMax) == HIGH) {
          machineState = STATE_ALARM; stopAllMotion(); aborted = true; break;
        }
      }
    }
    if (aborted) break;

    for (int i = 0; i < AXIS_COUNT; i++) {
      if (blk.deltaSteps[i] > 0) {
        errAccum[i] += blk.deltaSteps[i];
        if (errAccum[i] >= (long)blk.maxSteps) {
          errAccum[i] -= blk.maxSteps;
          pulseAxis((AxisId)i, blk.dirPos[i]);
        }
      }
    }

    float v_inst;
    if (step < step_accel_end && step_accel_end > step_resume_start) {
      float frac = (float)(step - step_resume_start) / (float)(step_accel_end - step_resume_start);
      v_inst = sqrtf(v_entry * v_entry + frac * (v_target_current * v_target_current - v_entry * v_entry));
    } else if (step >= step_decel_start) {
      float steps_left = (float)(blk.maxSteps - step);
      float total_decel_steps = (float)(blk.maxSteps - step_decel_start);
      float frac = (total_decel_steps > 0) ? (steps_left / total_decel_steps) : 0.0f;
      v_inst = sqrtf(v_exit * v_exit + frac * (v_target_current * v_target_current - v_exit * v_exit));
    } else {
      v_inst = v_target_current;
    }

    int currentDelayUs = speedToStepUs(v_inst, mmPerStep);
    currentFeedRateMmMin = v_inst * 60.0f;
    waitStepHardwareTimer(currentDelayUs);
  }

  bool completed = !aborted && actuatorsEnabled && machineState != STATE_ALARM;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    for (int i = 0; i < AXIS_COUNT; i++) {
      if (completed && blk.deltaSteps[i] > 0) {
        ax[i].stepCount = blk.targetSteps[i];
      }
      plannedPosMm[i] = ax[i].getMPosMm();
    }
    xSemaphoreGive(stateMutex);
  }

  currentFeedRateMmMin = 0.0f;
  if (machineState != STATE_ALARM && !holdActive) machineState = STATE_IDLE;
  return true;
}