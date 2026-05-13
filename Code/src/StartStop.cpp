#include "StartStop.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "types.h"

static bool isCheckingFinish = false;
static float finishCheckStartDist = 0;
static bool finishPaused = false; // Nieuwe variabele voor de pauze-stand

FinishState checkFinishStatus() {
    if (!USE_START_FINISH) return NOT_FINISH;

    // We kijken of de sensoren grotendeels zwart zien (minstens 6 van de 8 sensoren)
    // Dit is robuuster tegen stofjes of kleine afwijkingen op het finishvlak.
    int blackCount = 0;
    for (int i = 0; i < 8; i++) {
        if (currentSensors.sensorMask & (1 << i)) blackCount++;
    }
    bool seesHeavyBlack = (blackCount >= 6); 
    bool seesNormalLine = currentSensors.lineDetected && !seesHeavyBlack;

    // --- PAUZE LOGICA ---
    if (finishPaused) {
        stopMotors(); // Houd motoren uit
        if (seesNormalLine) { 
            Serial.println(">>> LIJN TERUGGEVONDEN: Pauze opgeheven! <<<");
            finishPaused = false;
            return NOT_FINISH; // Robot gaat weer rijden in de loop
        }
        return CHECKING_FINISH; // Blijf in "pauze"
    }

    // --- DETECTIE LOGICA ---
    if (seesHeavyBlack) {
        if (!isCheckingFinish) {
            isCheckingFinish = true;
            finishCheckStartDist = currentSensors.distanceDriven;
        }

        float distDriven = currentSensors.distanceDriven - finishCheckStartDist;
        
        // Pas als hij 5 cm over zwart heeft gereden, is het de finish
        if (distDriven >= 5.0) { // Jouw gevraagde 5 cm
            Serial.println(">>> FINISH BEVESTIGD: Robot gaat in pauze-stand <<<");
            isCheckingFinish = false;
            finishPaused = true; 
            return FINISH_CONFIRMED; 
        }
        
        // Tijdens de 5cm check rijden we rustig door
        setMotorSpeed(BASE_SPEED_MAPPING, BASE_SPEED_MAPPING);
        return CHECKING_FINISH;
    } 
    else {
        // We zien GEEN zwart meer. 
        if (isCheckingFinish) {
            isCheckingFinish = false;
            // Als we weer een normale lijn zien, was de zwarte balk korter dan 5cm.
            // Dat betekent: het was een kruispunt of T-splitsing!
            if (seesNormalLine) {
                Serial.println(">>> KORTE BALK GEZIEN: Kruispunt gedetecteerd! <<<");
                return INTERSECTION_DETECTED;
            }
        }
        return NOT_FINISH;
    }
}

// Belangrijk om deze ook te updaten voor als je opnieuw start
void resetStartSequence() {
    isCheckingFinish = false;
    finishPaused = false;
}