#include "Mapping.h"
#include "Config.h"

// --- 1. GLOBALE VARIABELEN (Slechts één keer declareren) ---
PathPoint trackMap[MAX_POINTS];
int mapIndex = 0;
float lastCurve = 0; 

PathPoint speedrunMap[MAX_POINTS];
int speedrunIndex = 0;
unsigned long speedrunStartTime = 0;

// --- 2. FUNCTIES ---

int getMapIndexAtDistance(float currentDist) {
    // We zoeken in de lijst naar het punt waar we nu ongeveer zijn
    for (int i = 0; i < mapIndex; i++) {
        if (trackMap[i].distance >= currentDist) {
            return i;
        }
    }
    return (mapIndex > 0) ? mapIndex - 1 : 0; 
}

int getLastNodeIndex(float currentDist) {
    for (int i = mapIndex - 1; i >= 0; i--) {
        if (trackMap[i].isNode && trackMap[i].distance <= currentDist) {
            return i;
        }
    }
    return -1;
}

const char* branchDirectionName(uint8_t dir) {
    switch (dir) {
        case BRANCH_LEFT: return "LEFT";
        case BRANCH_STRAIGHT: return "STRAIGHT";
        case BRANCH_RIGHT: return "RIGHT";
        case BRANCH_BACK: return "BACK";
        default: return "NONE";
    }
}

void savePoint(float dist, float curve, float obsDist, int32_t lTicks, int32_t rTicks,
               uint8_t branchDirection, bool isNode,
               bool isDeadEnd, bool isZigzag, uint8_t availableBranches) {
    if (mapIndex < MAX_POINTS) {
        uint8_t detectedType = 0;

        // Detecteer echte zigzags: snel wisselende richting
        if ((curve < -ZIGZAG_THRESHOLD && lastCurve > ZIGZAG_THRESHOLD) ||
            (curve > ZIGZAG_THRESHOLD && lastCurve < -ZIGZAG_THRESHOLD)) {
            detectedType = 1; // Zigzag
            isZigzag = true;
        }
        else if (obsDist < 30.0 && obsDist > 1.0) {
            detectedType = 6; // Obstakel of smalle passage
        }
        else if (curve <= -SHARP_TURN_THRESHOLD) {
            detectedType = 4; // Scherp Links
        }
        else if (curve >= SHARP_TURN_THRESHOLD) {
            detectedType = 5; // Scherp Rechts
        }
        else if (curve <= -12.0) {
            detectedType = 2; // Flauw Links
        }
        else if (curve >= 12.0) {
            detectedType = 3; // Flauw Rechts
        }

        trackMap[mapIndex].distance = dist;
        trackMap[mapIndex].curve = curve;
        trackMap[mapIndex].type = detectedType;
        trackMap[mapIndex].branchDirection = branchDirection;
        trackMap[mapIndex].availableBranches = availableBranches;
        trackMap[mapIndex].isNode = isNode;
        trackMap[mapIndex].isDeadEnd = isDeadEnd;
        trackMap[mapIndex].isZigzag = isZigzag;
        trackMap[mapIndex].leftTicks = abs(lTicks);
        trackMap[mapIndex].rightTicks = abs(rTicks);

        lastCurve = curve;
        mapIndex++;
    }
}

void dumpMap(unsigned long travelTime) {
    Serial.println("\n======= VOLLEDIGE DATA DUMP =======");
    Serial.print("Totale rijtijd: "); Serial.print(travelTime / 1000.0); Serial.println(" sec");
    
    float totaleAfstand = (mapIndex > 0) ? trackMap[mapIndex-1].distance : 0;
    Serial.print("Totale afstand: "); Serial.print(totaleAfstand); Serial.println(" cm");
    
    Serial.println("-----------------------------------");
    Serial.println("Punt;Afstand_cm;Lijn_Positie;Type_ID;Node;Branch;DeadEnd;Zigzag;Branches;LeftTicks;RightTicks");
    
    for (int i = 0; i < mapIndex; i++) {
        Serial.printf("%d;%.1f;%.2f;%d;%d;%s;%d;%d;%d;%d;%d\n",
            i,
            trackMap[i].distance,
            trackMap[i].curve,
            trackMap[i].type,
            trackMap[i].isNode ? 1 : 0,
            branchDirectionName(trackMap[i].branchDirection),
            trackMap[i].isDeadEnd ? 1 : 0,
            trackMap[i].isZigzag ? 1 : 0,
            trackMap[i].availableBranches,
            trackMap[i].leftTicks,
            trackMap[i].rightTicks);
    }
    Serial.println("======= EINDE DATA DUMP =======\n");
}

void dumpSpeedrunMap(unsigned long travelTime) {
    Serial.println("\n======= SPEEDRUN DEBUG DUMP =======");
    Serial.print("Speedrun tijd: "); Serial.print(travelTime / 1000.0); Serial.println(" sec");
    Serial.println("Punt;Afstand_cm;Lijn_Positie;Type_ID;LeftTicks;RightTicks");
    for (int i = 0; i < speedrunIndex; i++) {
        Serial.printf("%d;%.1f;%.2f;%d;%d;%d\n", 
            i, speedrunMap[i].distance, speedrunMap[i].curve, 
            speedrunMap[i].type, speedrunMap[i].leftTicks, speedrunMap[i].rightTicks);
    }
    Serial.println("======= EINDE SPEEDRUN DUMP =======\n");
}

void resetMap() {
    mapIndex = 0;
    speedrunIndex = 0; // Ook de debug map resetten
    lastCurve = 0;
    Serial.println("Kaarten gewist voor nieuwe poging.");
}