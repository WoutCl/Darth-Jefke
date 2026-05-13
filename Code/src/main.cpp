#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Logic.h"
#include "Mapping.h"
#include "StartStop.h"


//begin variabelen
RobotState currentState = IDLE;
RobotState previousState = IDLE;
enum ObstaclePhase
{
  OB_NONE,
  OB_DRIVE_RIGHT,
  OB_SEARCH_LINE
};

ObstaclePhase obstaclePhase = OB_NONE;
unsigned long obstacleStartTime = 0;
float obstacleStartDistance = 0;
int obstacleDetectCount = 0;
int obstacleClearCount = 0;
const int OBSTACLE_CONFIRM_COUNT = 3;
const int OBSTACLE_CLEAR_COUNT = 3;
const float OBSTACLE_PASS_DISTANCE = 27.0; // cm rechtdoor rijden voorbij het object
const int OBSTACLE_RIGHT_SPEED_HIGH = 95;
const int OBSTACLE_RIGHT_SPEED_LOW = 25;
const int OBSTACLE_RIGHT_SPEED_AFTER = 85;
const int OBSTACLE_RIGHT_SPEED_BIAS = 10;
const int OBSTACLE_LEFT_SEARCH_SPEED = 20;
const float OBSTACLE_LINE_STEER_FACTOR = 1.0f;
const int OBSTACLE_LINE_STEER_LIMIT = 70;
float lastSavedDistance = 0;
float lastSavedLinePosition = 0;
unsigned long mappingStartTime = 0;
unsigned long speedrunModeStartTime = 0;
unsigned long mappingElapsedTime = 0;
unsigned long speedrunElapsedTime = 0;
static uint8_t lastSensorMask = 0;
static unsigned long finishDetectStartTime = 0;
static bool finishPaused = false;

// Variabelen voor de knop
int buttonPressCount = 0;
bool lastButtonState = HIGH; // We gaan uit van INPUT_PULLUP

// Helper: Detect sharp turn direction based on sensor pattern
// Returns: -1 for LEFT turn, 1 for RIGHT turn, 0 for no turn
inline int detectSharpTurnDirection(uint8_t mask) {
    bool midSensors = (mask & MID_SENSOR_MASK) != 0;
    bool leftExtreme = (mask & 0b11000000) != 0;  // sensors 0-1
    bool rightExtreme = (mask & 0b00000011) != 0; // sensors 6-7
    
    if (!midSensors && leftExtreme && !rightExtreme) return -1;  // LEFT
    if (!midSensors && !leftExtreme && rightExtreme) return 1;   // RIGHT
    return 0; // No sharp turn
}

inline bool isFullBlack() {
    return currentSensors.sensorMask == 0xFF;
}

inline uint8_t getBranchMask(uint8_t mask) {
    uint8_t branches = 0;
    if (mask & LEFT_SIDE_MASK) branches |= BRANCH_MASK_LEFT;
    if (mask & MID_SENSOR_MASK) branches |= BRANCH_MASK_STRAIGHT;
    if (mask & RIGHT_SIDE_MASK) branches |= BRANCH_MASK_RIGHT;
    return branches;
}

inline uint8_t chooseLeftHandBranch(uint8_t branches) {
    if (branches & BRANCH_MASK_LEFT) return BRANCH_LEFT;
    if (branches & BRANCH_MASK_STRAIGHT) return BRANCH_STRAIGHT;
    if (branches & BRANCH_MASK_RIGHT) return BRANCH_RIGHT;
    return BRANCH_NONE;
}

inline uint8_t chooseFallbackBranch(uint8_t branches) {
    if (branches & BRANCH_MASK_STRAIGHT) return BRANCH_STRAIGHT;
    if (branches & BRANCH_MASK_RIGHT) return BRANCH_RIGHT;
    if (branches & BRANCH_MASK_LEFT) return BRANCH_LEFT;
    return BRANCH_NONE;
}

inline bool onlyMiddleLastSeen(uint8_t mask) {
    return (mask & MID_SENSOR_MASK) && !(mask & (LEFT_SIDE_MASK | RIGHT_SIDE_MASK));
}

void performDeadEndRecovery() {
    Serial.println("Dead-end gedetecteerd, 180 graden draaien.");
    stopMotors();
    setMotorSpeed(120, -120);
    delay(TURN_180_DURATION_MS);
    stopMotors();
    delay(120);
}

void saveIntersectionNode(float distance, uint8_t branchDirection, uint8_t branchMask) {
    savePoint(distance,
              currentSensors.linePosition,
              currentSensors.distance,
              currentSensors.leftTicks,
              currentSensors.rightTicks,
              branchDirection,
              true,
              false,
              false,
              branchMask);
}

void markLastNodeDeadEnd() {
    int idx = getLastNodeIndex(currentSensors.distanceDriven);
    if (idx >= 0) {
        trackMap[idx].isDeadEnd = true;
        if (trackMap[idx].branchDirection == BRANCH_LEFT) {
            trackMap[idx].branchDirection = chooseFallbackBranch(trackMap[idx].availableBranches & ~BRANCH_MASK_LEFT);
        }
        trackMap[idx].type = 6;
        Serial.printf("Node %d als doodlopend gemarkeerd, nieuwe richting %d\n", idx, trackMap[idx].branchDirection);
    }
}

uint8_t probeAvailableBranches() {
    setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
    delay(INTERSECTION_PROBE_MS);
    uint8_t branches = getBranchMask(currentSensors.sensorMask);
    stopMotors();
    return branches;
}

bool obstacleInFront() {
  return currentSensors.distance > 1.0 && currentSensors.distance < DISTANCE_THRESHOLD;
}

bool obstacleConfirmed() {
  if (obstacleInFront()) {
    obstacleDetectCount++;
    obstacleClearCount = 0;
  } else {
    obstacleDetectCount = 0;
  }
  return obstacleDetectCount >= OBSTACLE_CONFIRM_COUNT;
}

bool obstacleStillDetected() {
  if (obstacleInFront()) {
    obstacleClearCount = 0;
    return true;
  }
  obstacleClearCount++;
  return obstacleClearCount < OBSTACLE_CLEAR_COUNT;
}

void beginObstacleAvoidance(RobotState fromState) {
  previousState = fromState;
  currentState = OBSTACLE_STOP;
  obstaclePhase = OB_DRIVE_RIGHT;
  obstacleStartTime = millis();
  obstacleStartDistance = currentSensors.distanceDriven;
  Serial.println("Obstakel gedetecteerd! Obstakeldrijving start.");
}

void handleObstacleAvoidance() {
  bool obstacleStillSeen = obstacleStillDetected();
  bool lineVisible = currentSensors.lineDetected;
  int baseSpeed = 55;
  float passedDistance = currentSensors.distanceDriven - obstacleStartDistance;

  Serial.printf("[OBSTACLE] afstand=%.1f cm, line=%d, phase=%d, passed=%.1f\n", currentSensors.distance, lineVisible, obstaclePhase, passedDistance);

  if (obstacleStillSeen) {
    obstaclePhase = OB_DRIVE_RIGHT;
    if (passedDistance < OBSTACLE_PASS_DISTANCE) {
      setMotorSpeed(baseSpeed + OBSTACLE_RIGHT_SPEED_HIGH, baseSpeed - OBSTACLE_RIGHT_SPEED_LOW);
      Serial.println("Obstacle zichtbaar: agressief rechts om object te omzeilen.");
    } else {
      setMotorSpeed(baseSpeed + OBSTACLE_RIGHT_SPEED_BIAS, baseSpeed - OBSTACLE_RIGHT_SPEED_AFTER);
      Serial.println("Obstacle zichtbaar: scherp rechts verder rond object.");
    }
  } else {
    if (obstaclePhase == OB_DRIVE_RIGHT) {
      if (passedDistance < OBSTACLE_PASS_DISTANCE) {
        setMotorSpeed(baseSpeed + OBSTACLE_RIGHT_SPEED_BIAS, baseSpeed - OBSTACLE_LEFT_SEARCH_SPEED);
        Serial.println("Nog niet ver genoeg voorbij object, rechtdoor met rechtsbias.");
      } else {
        obstaclePhase = OB_SEARCH_LINE;
        Serial.println("Object weg en genoeg voorbij: lijn terugzoeken.");
      }
    } else {
      if (lineVisible) {
        float lineError = currentSensors.linePosition;
        int steer = constrain((int)(lineError * OBSTACLE_LINE_STEER_FACTOR), -OBSTACLE_LINE_STEER_LIMIT, OBSTACLE_LINE_STEER_LIMIT);
        int leftPWM = constrain(baseSpeed - 10 + steer, 0, MAX_SPEED);
        int rightPWM = constrain(baseSpeed - 10 - steer, 0, MAX_SPEED);
        setMotorSpeed(leftPWM, rightPWM);
        Serial.println("Lijn gedetecteerd, stuur terug naar de lijn.");

        if (abs(lineError) < 10.0 && passedDistance >= 30.0) {
          Serial.println("Lijn teruggevonden, terug naar vorige modus.");
          currentState = previousState;
          obstaclePhase = OB_NONE;
          return;
        }
      } else {
        setMotorSpeed(baseSpeed - 20, baseSpeed + 30);
        Serial.println("Geen lijn, zoek langzaam naar links.");
      }
    }
  }

  if (previousState == MAPPING) {
    float distSinceLastSave = abs(currentSensors.distanceDriven - lastSavedDistance);
    if (distSinceLastSave >= 5.0) {
      savePoint(currentSensors.distanceDriven,
                lineVisible ? currentSensors.linePosition : 0.0,
                currentSensors.distance,
                currentSensors.leftTicks,
                currentSensors.rightTicks);
      lastSavedDistance = currentSensors.distanceDriven;
      Serial.println("Obstacle punt opgeslagen in map.");
    }
  } else if (previousState == SPEEDRUN) {
    logSpeedrunPoint(currentSensors.distanceDriven,
                     lineVisible ? currentSensors.linePosition : 0.0,
                     6);
  }
}

void setup()
{
  Serial.begin(115200);
  setupMotors();
  startSensorTask(); // Start Core 0

  // Zorg dat de afstand hoog begint, niet op 0!
  currentSensors.distance = 999.0;

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP); // Zorg dat de switch pin goed staat

  Serial.println("--- Robot Ready ---");
  Serial.println("Druk 1x: Start");
  Serial.println("Druk 2x: Stop");
  Serial.println("Druk 3x: Data Dump");
}

void loop()
{
  // --- 1. KNOP LOGICA (Debounced) ---
  bool currentButtonState = digitalRead(START_BUTTON_PIN);

  // Kijk of de knop is ingedrukt (van HIGH naar LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW)
  {
    delay(50); // Korte pauze tegen "denderen" van de knop
    buttonPressCount++;

    Serial.print("Knop ingedrukt! Aantal keer: ");
    Serial.println(buttonPressCount);

    if (buttonPressCount == 1)
    {
      currentState = COUNTDOWN;
    }
    else if (buttonPressCount == 2)
    {
      if (mappingStartTime > 0) {
          mappingElapsedTime = millis() - mappingStartTime;
          mappingStartTime = 0;
      }
      if (speedrunModeStartTime > 0) {
          speedrunElapsedTime = millis() - speedrunModeStartTime;
          speedrunModeStartTime = 0;
      }
      currentState = IDLE;
      stopMotors();
      Serial.println("Actie: GESTOPT.");
    }
    else if (buttonPressCount == 3)
    {
      if (mappingStartTime > 0) {
          mappingElapsedTime = millis() - mappingStartTime;
          mappingStartTime = 0;
      }
      if (speedrunModeStartTime > 0) {
          speedrunElapsedTime = millis() - speedrunModeStartTime;
          speedrunModeStartTime = 0;
      }
      currentState = DATA_DUMP;
      stopMotors();
      Serial.println("Actie: DATA DUMP START.");
    }
    else
    {
      // Na 3x resetten we de boel weer
      buttonPressCount = 0;
      currentState = IDLE;
    }
  }

  // --- Dead-end detectie ---
  if (!currentSensors.lineDetected && onlyMiddleLastSeen(lastSensorMask) && currentSensors.distanceDriven > 5.0) {
      Serial.println("Dead-end detected - Turning 180");
      markLastNodeDeadEnd();
      performDeadEndRecovery();
  }

  lastSensorMask = currentSensors.sensorMask;
  lastButtonState = currentButtonState;

  // --- 2. STATE MACHINE ---
  switch (currentState)
  {
  case IDLE:
    stopMotors();
    break;

case COUNTDOWN:
    Serial.println("Starten in 3...");
    delay(3000);
    
    encL.setCount(0);
    encR.setCount(0);
    currentSensors.distanceDriven = 0; 
    lastSavedDistance = 0;
    lastSavedLinePosition = 0;
    
    if (digitalRead(MODE_SWITCH_PIN) == LOW) {
        resetMap();
        mappingStartTime = millis();
        mappingElapsedTime = 0;
        speedrunModeStartTime = 0;
        speedrunElapsedTime = 0;
        currentState = MAPPING;
    } else {
        prepareSpeedrun();    // <--- RESET DE LOGICA VARIABELEN
        speedrunIndex = 0;
        speedrunModeStartTime = millis(); // De timer start NU
        speedrunElapsedTime = 0;
        mappingStartTime = 0;
        mappingElapsedTime = 0;
        currentState = SPEEDRUN;
    }
    break;

case MAPPING: {
    static unsigned long whiteTime = 0; 
    static int lastSideSeen = 0; 
    static unsigned long actionTimer = 0; 
    static unsigned long lastPrint = 0;
    static unsigned long finishDetectedTime = 0;

    // --- 1. SENSOR DATA PRINTEN (Voor debugging) ---
    if (millis() - lastPrint > 150) {
        Serial.print("Sensoren: [");
        for (int i = 7; i >= 0; i--) Serial.print((currentSensors.sensorMask & (1 << i)) ? "X" : "_");
        int cleanDist = (currentSensors.distance > 0 && currentSensors.distance < 200) ? currentSensors.distance : 0;
        Serial.printf("] | Dist: %3d cm | Driven: %5.1f cm\n", cleanDist, currentSensors.distanceDriven);
        lastPrint = millis();
    }

    // --- 2. OBSTAKEL DETECTIE (WACHT-MODUS) ---
    if (currentSensors.distance > 1 && currentSensors.distance < 15) {
        stopMotors();
        Serial.println("!!! OBSTAKEL GEZIEN: Robot wacht... !!!");
        break; 
    }

    // --- 3. VERBETERDE FINISH DETECTIE (Debounce van 200ms) ---
    if (currentSensors.sensorMask == 0xFF) {
        if (finishDetectedTime == 0) finishDetectedTime = millis();
        
        if (millis() - finishDetectedTime > 200) { 
            stopMotors();
            Serial.println(">>> FINISH BEVESTIGD: Robot stopt. <<<");
            break;
        }
        break; 
    } else {
        finishDetectedTime = 0; 
    }

    // --- 4. BOCHT & KRUISPUNT LOGICA (Met Gap Detectie & Rechtdoor prioriteit) ---
    FinishState finishStatus = checkFinishStatus();
    bool midLine = (currentSensors.sensorMask & 0b00011000); // Lijn in het midden
    
    // Sharp Turn Gap Detectie: Zoekt naar zwart op uiterste sensor EN wit ertussen
    bool sharpLeftGap = (currentSensors.sensorMask & 0b11000000) && !(currentSensors.sensorMask & 0b00100000);
    bool sharpRightGap = (currentSensors.sensorMask & 0b00000011) && !(currentSensors.sensorMask & 0b00000100);

    if ((finishStatus == INTERSECTION_DETECTED || sharpLeftGap || sharpRightGap) && (millis() - actionTimer > 600)) {
        
        // Iets vooruit om as goed te zetten voor de draai
        setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
        delay(45); 
        
        uint8_t branches = probeAvailableBranches();
        uint8_t targetDir = chooseLeftHandBranch(branches);

        // BESLISSING 1: Linksaf (Afslag of Scherpe Bocht)
        if (targetDir == BRANCH_LEFT || sharpLeftGap) {
            Serial.println(">>> ACTIE: Draai naar LINKS (Prioriteit) <<<");
            setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
            delay(250); 
            unsigned long turnTimeout = millis();
            while (millis() - turnTimeout < 2500) {
                // readSensors(); // <--- JOUW SENSOR FUNCTIE HIER
                if (currentSensors.sensorMask & 0b00011000) break;
                delay(5);
            }
            actionTimer = millis();
        } 
        // BESLISSING 2: Rechtsaf (Alleen als er GEEN middenlijn is, volgens Linkerhandregel)
        else if ((targetDir == BRANCH_RIGHT || sharpRightGap) && !midLine) {
            Serial.println(">>> ACTIE: Draai naar RECHTS <<<");
            setMotorSpeed(BASE_SPEED_TURNS, -BASE_SPEED_TURNS);
            delay(250);
            unsigned long turnTimeout = millis();
            while (millis() - turnTimeout < 2500) {
                // readSensors(); // <--- JOUW SENSOR FUNCTIE HIER
                if (currentSensors.sensorMask & 0b00011000) break;
                delay(5);
            }
            actionTimer = millis();
        }
        // BESLISSING 3: Rechtdoor (Als er een middenlijn is, negeren we rechts)
        else {
            Serial.println(">>> INFO: Rechtdoor aanhouden volgens Linkerhandregel <<<");
            actionTimer = millis(); 
        }

        if (finishStatus == INTERSECTION_DETECTED) {
            saveIntersectionNode(currentSensors.distanceDriven, targetDir, branches);
        }
        whiteTime = 0;
        break;
    }

    // --- 5. LIJNVOLGEN OF DEAD-END ---
    if (currentSensors.lineDetected) {
        whiteTime = 0; 
        calculatePID();
    } else {
        if (whiteTime == 0) whiteTime = millis();
        
        // Dead-end herstel (180 graden draai)
        if (millis() - whiteTime > 600) { 
            Serial.println(">>> DEAD-END: Draaien tot we weer een lijn zien <<<");
            markLastNodeDeadEnd();
            setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS); 
            delay(500); 
            unsigned long deadEndTimeout = millis();
            while (millis() - deadEndTimeout < 3000) {
                // readSensors(); // <--- JOUW SENSOR FUNCTIE HIER
                if (currentSensors.sensorMask & 0b00011000) break;
                delay(5);
            }
            whiteTime = 0;
            actionTimer = millis();
            break;
        } else {
            // Zoeken op basis van laatst geziene kant
            if (lastSideSeen == -1) setMotorSpeed(-BASE_SPEED_TURNS, BASE_SPEED_TURNS);
            else if (lastSideSeen == 1) setMotorSpeed(BASE_SPEED_TURNS, -BASE_SPEED_TURNS);
            else setMotorSpeed(BASE_SPEED_MAPPING - 15, BASE_SPEED_MAPPING - 15);
        }
    }

    // --- 6. GEHEUGEN UPDATE & DATA OPSLAAN ---
    if (currentSensors.sensorMask & 0b11100000) lastSideSeen = -1;
    else if (currentSensors.sensorMask & 0b00000111) lastSideSeen = 1;

    float distSinceLastSave = abs(currentSensors.distanceDriven - lastSavedDistance);
    if (currentSensors.lineDetected && distSinceLastSave >= NODE_SAVE_DISTANCE) {
        savePoint(currentSensors.distanceDriven, currentSensors.linePosition, 
                  currentSensors.distance, currentSensors.leftTicks, currentSensors.rightTicks);
        lastSavedDistance = currentSensors.distanceDriven;
    }
}
break;








  
  
case SPEEDRUN:
    if (obstacleConfirmed()) {
      beginObstacleAvoidance(currentState);
      break;
    }

    {
        FinishState finishStatus = checkFinishStatus();
        if (finishStatus == CHECKING_FINISH) {
            break;
        }
        if (finishStatus == FINISH_CONFIRMED) {
            Serial.println("Finish bereikt tijdens speedrun! Stop.");
            currentState = IDLE;
            stopMotors();
            break;
        }
    }

    calculateSpeedrun();

    // Automatische stop: als we voorbij de gemapte afstand zijn en de lijn echt kwijt raken
    if (mapIndex > 0) {
        float finalMapDistance = trackMap[mapIndex - 1].distance;
        if (currentSensors.distanceDriven > finalMapDistance + 20.0 && !currentSensors.lineDetected) {
            Serial.println("Finish bereikt! Stop.");
            currentState = IDLE;
            stopMotors();
        }
    }
    break;

  case OBSTACLE_STOP:
    handleObstacleAvoidance();
    break;

 case DATA_DUMP:
    stopMotors();
    
    // Bereken de tijden voor de dumps
    unsigned long mappingTijd = (mappingElapsedTime > 0) ? mappingElapsedTime : (mappingStartTime > 0 ? millis() - mappingStartTime : 0);
    unsigned long speedrunTijd = (speedrunElapsedTime > 0) ? speedrunElapsedTime : (speedrunModeStartTime > 0 ? millis() - speedrunModeStartTime : 0);
    
    // 1. Dump de originele kaart (Mapping)
    dumpMap(mappingTijd); 
    
    delay(1000); // Korte pauze zodat de seriële buffer niet overloopt
    
    // 2. Dump de debug kaart (Speedrun)
    // Deze is leeg als je nog geen speedrun hebt gedaan
    dumpSpeedrunMap(speedrunTijd); 
    
    buttonPressCount = 0;
    mappingStartTime = 0;
    speedrunModeStartTime = 0;
    mappingElapsedTime = 0;
    speedrunElapsedTime = 0;
    currentState = IDLE;
    break;
  }
}