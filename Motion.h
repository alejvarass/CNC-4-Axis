#pragma once
#include <Arduino.h>
#include "Config.h"

extern AxisHW hw[AXIS_COUNT];
extern AxisSettings ax[AXIS_COUNT];
extern volatile MachineState machineState;
extern volatile bool actuatorsEnabled;
extern volatile bool estopTriggered;
extern bool estopActiveHigh;
extern volatile bool dualLimitsEnabled;
extern volatile bool probeEnabled;
extern volatile bool softLimitsEnabled;
extern volatile uint32_t stopEpoch;
extern volatile uint32_t jogCancelEpoch;
extern volatile bool homingCycleRequested;
extern volatile float currentFeedRateMmMin;
extern volatile float currentSpindleRpm;
extern volatile bool spindleRunning;
extern volatile bool holdActive;
extern volatile uint32_t lastCommTimeMs;
extern volatile bool probeTriggered;
extern float lastProbePosMm[AXIS_COUNT];
extern SemaphoreHandle_t stateMutex;
extern hw_timer_t * stepTimer;
extern TaskHandle_t TaskMotorsHandle;

void initMotionHardware();
void setActuatorsState(bool enable);
bool estopActive();
bool probeActive();
void stopAllMotion();
bool clearAlarmState();
void refreshInputs();
float safeSpm(AxisId a);
void pulseAxis(AxisId a, bool isForward);
void setAxisDirection(AxisId a, bool isForward);
void compensateBacklash(AxisId a, bool newDirPos, uint32_t stepUs);
void waitStepHardwareTimer(uint32_t delayUs);
void setSpindleSpeed(float rpm, float maxRpm = 1000.0f);
void setSpindleState(bool enable, float rpm = -1.0f);
const char* grblStateStr();
void doHomingCycleInternal();
void TaskMotors(void * pvParameters);