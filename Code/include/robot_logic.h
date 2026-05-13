#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include "types.h"
#include <vector>

// Global route access voor main.cpp indien nodig
extern std::vector<EnhancedRoutePoint> route;

// Functie definities
void initLogic();
void updateLogic(RobotState &currentState);
void executePID(float targetSpeed);
void detectSharpTurnEntry(uint8_t mask, float error);
void handleSharpTurnInFlight();

#endif