#include "robot_logic.h"
#include "types.h"
#include "Config.h" // Met hoofdletter C
#include "sensors.h"
#include "motors.h"
#include "StartStop.h"

static bool inSharpTurnMode = false;
static int sharpTurnDir = 0; 
static uint32_t sharpTurnEntryTime = 0;
static float smoothSpeed = BASE_SPEED_MAPPING;
static float lastSavedDist = 0;

std::vector<EnhancedRoutePoint> route;

void initLogic() {
    route.reserve(1000);
}

void detectSharpTurnEntry(uint8_t mask, float error) {
    bool leftSide = (mask & 0b11000000);
    bool rightSide = (mask & 0b00000011);
    bool midSensors = (mask & MID_SENSOR_MASK);

    if (!midSensors && (leftSide || rightSide) && !inSharpTurnMode) {
        inSharpTurnMode = true;
        sharpTurnDir = leftSide ? -1 : 1;
        sharpTurnEntryTime = millis();
        Serial.printf("Sharp turn detected, turning %s\n", sharpTurnDir == -1 ? "left" : "right");
        // EXPLICIET TYPE TOEVOEGEN:
        route.push_back(EnhancedRoutePoint{currentSensors.distanceDriven, (sharpTurnDir == -1 ? SHARP_LEFT : SHARP_RIGHT), (uint8_t)BASE_SPEED_TURNS, error, true, false});
    }
}

void handleSharpTurnInFlight() {
    if (millis() - sharpTurnEntryTime > 800) { inSharpTurnMode = false; return; }

    if (sharpTurnDir == -1) setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
    else setMotorSpeed(BASE_SPEED_TURNS, -BASE_SPEED_TURNS);

    if (currentSensors.sensorMask & MID_SENSOR_MASK) {
        inSharpTurnMode = false;
        // EXPLICIET TYPE TOEVOEGEN:
        route.push_back(EnhancedRoutePoint{currentSensors.distanceDriven, STRAIGHT, (uint8_t)BASE_SPEED_TURNS, 0, false, true});
    }
}

void executePID(float targetSpeed) {
    static float lastError = 0;
    float error = currentSensors.linePosition;
    float P = KP * error;
    float D = KD * (error - lastError);
    lastError = error;
    setMotorSpeed(targetSpeed + P + D, targetSpeed - (P + D));
}

void updateLogic(RobotState &currentState) {
    if (currentSensors.distance < DISTANCE_THRESHOLD) { stopMotors(); return; }

    FinishState fStatus = checkFinishStatus();
    
    if (fStatus == INTERSECTION_DETECTED) {
        sharpTurnDir = -1; 
        inSharpTurnMode = true;
        sharpTurnEntryTime = millis();
        return;
    }

    if (fStatus == FINISH_CONFIRMED) { currentState = FINISHED; return; }

    if (inSharpTurnMode) { handleSharpTurnInFlight(); return; }

    if (currentState == MAPPING) {
        if (currentSensors.lineDetected) {
            detectSharpTurnEntry(currentSensors.sensorMask, currentSensors.linePosition);
            executePID(BASE_SPEED_MAPPING);
            if (currentSensors.distanceDriven - lastSavedDist >= SAVE_POINT_INTERVAL) {
                // EXPLICIET TYPE TOEVOEGEN:
                route.push_back(EnhancedRoutePoint{currentSensors.distanceDriven, STRAIGHT, (uint8_t)BASE_SPEED_MAPPING, currentSensors.linePosition, false, false});
                lastSavedDist = currentSensors.distanceDriven;
            }
        } else { setMotorSpeed(-60, 60); }
    } 
    else if (currentState == SPEEDRUN) {
        executePID(MAX_SPEED);
    }
}