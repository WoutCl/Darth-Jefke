#ifndef MAPPING_H
#define MAPPING_H

#include <Arduino.h>

enum BranchDirection {
    BRANCH_NONE = 0,
    BRANCH_LEFT = 1,
    BRANCH_STRAIGHT = 2,
    BRANCH_RIGHT = 3,
    BRANCH_BACK = 4
};

struct PathPoint {
    float distance;
    float curve;
    uint8_t type;
    uint8_t branchDirection;
    uint8_t availableBranches;
    bool isNode;
    bool isDeadEnd;
    bool isZigzag;
    int32_t leftTicks;  // NIEUW: voor je Python script
    int32_t rightTicks; // NIEUW: voor je Python script
};

const int MAX_POINTS = 500;
extern int mapIndex;

// In Mapping.h
extern PathPoint trackMap[MAX_POINTS];
extern PathPoint speedrunMap[MAX_POINTS];
extern int speedrunIndex;
extern unsigned long speedrunStartTime; // Voor de tijdmeting

void dumpMap(unsigned long travelTime);
void dumpSpeedrunMap(unsigned long travelTime); // Nu met tijd
// We voegen ticks toe aan de save functie
void savePoint(float dist, float curve, float obsDist, int32_t lTicks, int32_t rTicks,
               uint8_t branchDirection = BRANCH_NONE, bool isNode = false,
               bool isDeadEnd = false, bool isZigzag = false, uint8_t availableBranches = 0);
void resetMap(); 
int getMapIndexAtDistance(float currentDist);
int getLastNodeIndex(float currentDist);
void prepareSpeedrun();


#endif