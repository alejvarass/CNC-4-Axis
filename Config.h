#pragma once
#include <Arduino.h>

#define FW_VERSION "6.5.1-GRBL"
#define NVS_SCHEMA_VERSION 8

const uint16_t SERVER_PORT = 5000;
const size_t MAX_LINE_LEN = 256;
const uint8_t PIN_ENABLE_ACTUATORS = 27; // LOW = Habilitado
const uint8_t PIN_ESTOP = 32;            // Pin E-stop (NC a GND con INPUT_PULLUP)
const uint8_t PIN_SPINDLE = 2;           // Pin Husillo (LEDC PWM)
const uint8_t PIN_PROBE = 26;            // Pin Probe G38
const uint32_t MIN_STEP_US = 50;         // Periodo minimo seguro para Core 3.x

const uint8_t SPINDLE_PWM_CH = 0;
const double SPINDLE_PWM_FREQ = 5000.0;
const uint8_t SPINDLE_PWM_RES = 8; // 0-255

enum AxisId { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2, AXIS_W = 3, AXIS_COUNT = 4 };
const char AXIS_CHARS[AXIS_COUNT] = {'X', 'Y', 'Z', 'W'};

enum MachineState { STATE_IDLE, STATE_RUN, STATE_HOLD, STATE_JOG, STATE_HOMING, STATE_ALARM, STATE_CHECK };

struct AxisHW {
  uint8_t pinStep;
  uint8_t pinDir;
  uint8_t pinLimitMin;
  uint8_t pinLimitMax;
};

struct AxisSettings {
  float stepsPerMm = 568.0f;
  float maxRateMmMin = 1320.0f;
  float accelMmSec2 = 30.0f;
  float maxTravelMm = 110.0f;
  float homingSeekRateMmMin = 500.0f;
  float homingFeedRateMmMin = 50.0f;
  float homingPulloffMm = 2.0f;
  uint8_t dirInvert = 0;
  float backlashMm = 0.0f;

  int64_t stepCount = 0;
  int64_t wcoSteps = 0;
  bool limitMinTriggered = false;
  bool limitMaxTriggered = false;
  bool isHomed = false;

  inline float getMPosMm() const {
    return (float)stepCount / ((stepsPerMm > 0.1f) ? stepsPerMm : 568.0f);
  }
  inline float getWPosMm() const {
    return (float)(stepCount - wcoSteps) / ((stepsPerMm > 0.1f) ? stepsPerMm : 568.0f);
  }
};