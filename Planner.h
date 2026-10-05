#pragma once
#include <Arduino.h>
#include <Stream.h>
#include "Config.h"
#include "Motion.h"

enum PlanResult { PLAN_OK = 0, PLAN_FULL = 1, PLAN_SOFT_LIMIT = 2, PLAN_NOT_HOMED = 3, PLAN_INVALID = 4 };

struct MotionBlock {
  int64_t targetSteps[AXIS_COUNT];
  float unitVec[AXIS_COUNT];
  float distanceMm;
  float cruiseSpeedMmS;
  float entrySpeedMmS;
  float exitSpeedMmS;
  float accelMmS2;
  uint32_t deltaSteps[AXIS_COUNT];
  bool dirPos[AXIS_COUNT];
  uint32_t maxSteps;
  bool active;
  bool isJog;
  bool isDwell;
  uint32_t dwellMs;
  bool isSpindleCmd;
  bool spindleEnable;
  float spindleRpm;
  bool isProbe;
};

const int BLOCK_QUEUE_SIZE = 16;
const float JUNCTION_DEVIATION_MM = 0.05f;

extern MotionBlock blockQueue[BLOCK_QUEUE_SIZE];
extern volatile int qHead;
extern volatile int qTail;
extern float plannedPosMm[AXIS_COUNT];

void initPlanner();
PlanResult planAndEnqueueBlock(float targetMm[AXIS_COUNT], float feedMmPm, bool isJog, bool isDwell, uint32_t dwellMs, bool isSpindle = false, bool spEnable = false, float spRpm = 0.0f, bool isProbe = false);
void purgeJogBlocks();
bool executeBlock(const MotionBlock& blk);
bool popBlock(MotionBlock& out);
bool waitQueueSpace(Stream* client, uint32_t timeoutMs);
void syncPlannedPosFromActual();