#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// Centrale definities om circulaire afhankelijkheden te voorkomen
enum RobotState { 
    IDLE, 
    COUNTDOWN,
    MAPPING, 
    SPEEDRUN, 
    OBSTACLE_STOP, 
    DATA_DUMP,
    FINISHED 
};

enum TurnType {
    STRAIGHT,
    NORMAL_TURN,
    SHARP_LEFT,
    SHARP_RIGHT,
    INTERSECTION,
    DEAD_END
};

enum FinishState { 
    NOT_FINISH, 
    CHECKING_FINISH, 
    FINISH_CONFIRMED,
    INTERSECTION_DETECTED 
};

struct EnhancedRoutePoint {
    float distance;
    TurnType turnType;
    uint8_t recommendedSpeed;
    float linePosition;
    bool isSharpTurnEntry;
    bool isSharpTurnExit;
};

#endif