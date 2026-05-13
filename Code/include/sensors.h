#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <ESP32Encoder.h> // Zorg dat deze hier ook staat voor het type

struct SensorData {
    uint8_t sensorMask;
    float linePosition;
    bool lineDetected;
    float distance;         // Ultrasoon
    float distanceDriven;   // Encoders in cm
    int32_t leftTicks;
    int32_t rightTicks;
    int irValues[8];
};

extern volatile SensorData currentSensors;
extern ESP32Encoder encL; // <--- VOEG DIT TOE
extern ESP32Encoder encR; // <--- VOEG DIT TOE

void startSensorTask();

#endif