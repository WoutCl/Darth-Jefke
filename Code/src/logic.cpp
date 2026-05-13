#include "Logic.h"
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Mapping.h" // ZEER BELANGRIJK: voor trackMap en getMapIndexAtDistance

// --- 1. GLOBALE STATISCHE VARIABELEN (Mogen hier maar 1x staan!) ---
static float smoothSpeed = 70;
static float integral = 0;
static float lastError = 0;
static float lastLoggedDist = 0;


void calculatePID() {
    // 1. Haal de foutwaarde op (0 is perfect op de lijn)
    float error = currentSensors.linePosition;

    // Als we de lijn helemaal kwijt zijn, doen we niets (of we gebruiken de laatste waarde)
    if (!currentSensors.lineDetected) {
        // Optioneel: als error > 0 draai rechtsom, als error < 0 draai linksom om lijn te zoeken
        return; 
    }

    // 2. Bereken PID componenten
    float P = error * KP;
    integral += error;
    float I = integral * KI;
    float D = (error - lastError) * KD;

    float turnCorrection = P + I + D;
    lastError = error;

    // 3. Pas toe op motorsnelheid
    int speedL = BASE_SPEED_MAPPING + turnCorrection;
    int speedR = BASE_SPEED_MAPPING - turnCorrection;

    // Zorg dat we binnen de PWM grenzen (0-255) blijven
    speedL = constrain(speedL, 0, MAX_SPEED);
    speedR = constrain(speedR, 0, MAX_SPEED);

    setMotorSpeed(speedL, speedR);
}


void prepareSpeedrun() {
    smoothSpeed = 70;  // Begin altijd rustig
    integral = 0;
    lastError = 0;
    lastLoggedDist = 0;
}

static bool applyNodeBias(int mappingIndex, int &leftPWM, int &rightPWM, int targetSpeed) {
    if (mappingIndex < 0 || mappingIndex >= mapIndex) return false;
    const PathPoint &node = trackMap[mappingIndex];
    if (!node.isNode || node.branchDirection == BRANCH_NONE) return false;

    switch (node.branchDirection) {
        case BRANCH_RIGHT:
            leftPWM = targetSpeed + 30;
            rightPWM = targetSpeed - 30;
            return true;
        case BRANCH_LEFT:
            leftPWM = targetSpeed - 30;
            rightPWM = targetSpeed + 30;
            return true;
        case BRANCH_STRAIGHT:
            leftPWM = targetSpeed;
            rightPWM = targetSpeed;
            return true;
        default:
            return false;
    }
}

void calculateSpeedrun() {
    float currentDist = currentSensors.distanceDriven;
    int mappingIndex = getMapIndexAtDistance(currentDist);
    
    // --- 1. LOOK-AHEAD (40cm = 8 punten) ---
    uint8_t immediateHazard = 0;
    uint8_t farHazard = 0;
    uint8_t currentHazard = 0;

    if (mappingIndex < mapIndex) {
        currentHazard = trackMap[mappingIndex].type;
        if (currentHazard > immediateHazard) immediateHazard = currentHazard;
    }

    for (int i = 1; i <= 8; i++) {
        int checkIndex = mappingIndex + i;
        if (checkIndex < mapIndex) {
            uint8_t type = trackMap[checkIndex].type;
            if (i <= 2) { // Direct (10-20cm)
                if (type > immediateHazard) immediateHazard = type;
            } else { // Verderop (20-40cm)
                if (type > farHazard) farHazard = type;
            }
        }
    }

    // --- 2. SNELHEIDS-LOGICA ---
    int targetSpeed = MAX_SPEED;
    if (immediateHazard >= 4) {
        targetSpeed = 60; // Heel traag voor scherpe bochten
    } else if (immediateHazard >= 1) {
        targetSpeed = 90; // Directe bocht/zigzag
    } else if (farHazard >= 4) {
        targetSpeed = 110; // Scherp bocht in de verte
    } else if (farHazard >= 1) {
        targetSpeed = 120; // Flauwe bocht in de verte
    }

    // Verlaag snelheid extra als de huidige lijnpositie een scherpe bocht aangeeft
    float currentCurve = abs(currentSensors.linePosition);
    if (currentCurve > 35.0) {
        targetSpeed = min(targetSpeed, 50);
    } else if (currentCurve > 22.0) {
        targetSpeed = min(targetSpeed, 70);
    } else if (currentCurve > 15.0) {
        targetSpeed = min(targetSpeed, 90);
    }

    // --- 3. RAMPING ---
    float accelRate = 0.4; // Iets sneller optrekken (was 0.25) om tijd te winnen
    float decelRate = 3.5; 

    if (smoothSpeed < targetSpeed) smoothSpeed += accelRate;
    else if (smoothSpeed > targetSpeed) smoothSpeed -= decelRate;

    if (smoothSpeed < 0) smoothSpeed = 0;
    if (smoothSpeed > MAX_SPEED) smoothSpeed = MAX_SPEED;

    // --- 4. PURE PID (Geen factoren, geen extra berekeningen) ---
    if (!currentSensors.lineDetected) {
        // Lijn kwijt: draai veel harder in de laatst bekende richting
        float recovery = lastError * 3.0f;
        int leftPWM = (int)smoothSpeed + (int)recovery;
        int rightPWM = (int)smoothSpeed - (int)recovery;
        leftPWM = constrain(leftPWM, 0, MAX_SPEED);
        rightPWM = constrain(rightPWM, 0, MAX_SPEED);
        setMotorSpeed(leftPWM, rightPWM);
        return;
    }

    float error = currentSensors.linePosition;
    float derivative = error - lastError;
    
    // Jouw standaard PID berekening:
    float output = (KP * error) + (KD * derivative);
    lastError = error;

    // --- 5. MOTOR AANSTURING (Met ondergrens) ---
    const int minSpeed = (immediateHazard >= 4 || currentCurve > 35.0f) ? 0 : 50;
    int leftPWM = (int)smoothSpeed + (int)output;
    int rightPWM = (int)smoothSpeed - (int)output;

    leftPWM = constrain(leftPWM, minSpeed, MAX_SPEED);
    rightPWM = constrain(rightPWM, minSpeed, MAX_SPEED);

    // Gebruik kaartinformatie om de juiste branch te nemen bij een opgeslagen kruispunt
    if (applyNodeBias(mappingIndex, leftPWM, rightPWM, (int)smoothSpeed)) {
        Serial.printf("Speedrun branch override bij node %d: left=%d right=%d\n", mappingIndex, leftPWM, rightPWM);
    }

    setMotorSpeed(leftPWM, rightPWM);

    // --- 6. DEBUG LOGGING ---
    if (abs(currentDist - lastLoggedDist) >= 5.0 && speedrunIndex < MAX_POINTS) {
        speedrunMap[speedrunIndex].distance = currentDist;
        speedrunMap[speedrunIndex].curve = error;
        speedrunMap[speedrunIndex].type = (immediateHazard >= 4) ? immediateHazard : farHazard;
        speedrunMap[speedrunIndex].leftTicks = abs(currentSensors.leftTicks);
        speedrunMap[speedrunIndex].rightTicks = abs(currentSensors.rightTicks);
        speedrunIndex++;
        lastLoggedDist = currentDist;
    }
}

void logSpeedrunPoint(float currentDist, float lineError, uint8_t type) {
    if (abs(currentDist - lastLoggedDist) >= 5.0 && speedrunIndex < MAX_POINTS) {
        speedrunMap[speedrunIndex].distance = currentDist;
        speedrunMap[speedrunIndex].curve = lineError;
        speedrunMap[speedrunIndex].type = type;
        speedrunMap[speedrunIndex].leftTicks = abs(currentSensors.leftTicks);
        speedrunMap[speedrunIndex].rightTicks = abs(currentSensors.rightTicks);
        speedrunIndex++;
        lastLoggedDist = currentDist;
    }
}

void resetPID() {
    lastError = 0;
    integral = 0;
}