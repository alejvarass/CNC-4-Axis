#pragma once
#include <Arduino.h>
#include <Stream.h>
#include "Config.h"
#include "Planner.h"

void initGCodeParser();
void processGrblLine(Stream& output, char* rawLine);
void printAllGrblSettings(Stream& out);