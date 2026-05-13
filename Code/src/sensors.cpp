#include "Sensors.h"
#include "Config.h"
#include <ESP32Encoder.h>

// Global variabelen
volatile SensorData currentSensors;
ESP32Encoder encL;
ESP32Encoder encR;

// Interne functies
void sensorTaskLoop(void * pvParameters);
float readUltrasonic();

void startSensorTask() {
    // C++ Truc: We dwingen '1' (PULLUP) naar het type dat de library verwacht, wat de naam nu ook is.
    ESP32Encoder::useInternalWeakPullResistors = (decltype(ESP32Encoder::useInternalWeakPullResistors))1;
    
    encL.attachFullQuad(ENC_L_A, ENC_L_B);
    encR.attachFullQuad(ENC_R_A, ENC_R_B);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    xTaskCreatePinnedToCore(sensorTaskLoop, "SensorTask", 4000, NULL, 1, NULL, 0);
}



void sensorTaskLoop(void * pvParameters) {
    for(;;) {
        float teller = 0;
        float noemersum = 0;
        uint8_t mask = 0;
        bool seesLine = false;
        int weights[8] = {-40, -30, -20, -10, 10, 20, 30, 40};

        for (int i = 0; i < 8; i++) {
            int val = analogRead(sensorPinnen[i]);
            currentSensors.irValues[i] = val;
            if (val > IR_THRESHOLD) {
                seesLine = true;
                mask |= (1 << (7 - i));
                teller += (float)weights[i]; 
                noemersum++;
            }
        }
        
        currentSensors.lineDetected = seesLine;
        currentSensors.sensorMask = mask;
        currentSensors.linePosition = (noemersum > 0) ? (teller / noemersum) : 0;

        // AFSTAND BEREKENING (Slechts 1 keer!)
        currentSensors.leftTicks = encL.getCount();
        currentSensors.rightTicks = encR.getCount();
        
        // We gebruiken jouw 58 ticks per cm
        float avgTicks = (abs(currentSensors.leftTicks) + abs(currentSensors.rightTicks)) / 2.0;
        currentSensors.distanceDriven = avgTicks / 58.0; 

        currentSensors.distance = readUltrasonic();

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
float readUltrasonic() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    if (duration == 0) return 999.0; 
    return duration * 0.034 / 2.0;
}