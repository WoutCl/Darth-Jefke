// Config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>


// --- Start/Finish Instellingen ---
#define USE_START_FINISH true
#define FINISH_DRIVE_ON_CM 10.0 // Hoeveel cm doorrijden na detectie balk

// --- Logica Instellingen ---
#define MID_SENSOR_MASK 0b00011000
#define LEFT_SIDE_MASK 0b11110000
#define RIGHT_SIDE_MASK 0b00001111
#define SAVE_POINT_INTERVAL 5.0
#define NODE_SAVE_DISTANCE 5.0
#define INTERSECTION_PROBE_MS 120
#define TURN_180_DURATION_MS 850
#define SPEEDRUN_LOOKAHEAD_POINTS 8
#define ZIGZAG_THRESHOLD 12.0
#define SHARP_TURN_THRESHOLD 25.0
#define BRANCH_MASK_LEFT 0x01
#define BRANCH_MASK_STRAIGHT 0x02
#define BRANCH_MASK_RIGHT 0x04
#define MAX_NODES 100

#define BASE_SPEED_TURNS 100

// Voeg dit toe aan Config.h
#define TICKS_PER_CM 58.0 // Aantal encoder ticks per centimeter (afhankelijk van je wielen en encoders)

// --- Knoppen & Switches ---
#define START_BUTTON_PIN 23
#define MODE_SWITCH_PIN 19

// --- IR Sensor Array ---
const int SENSOR_COUNT = 8;
const int sensorPinnen[SENSOR_COUNT] = {32, 33, 34, 35, 36, 39, 25, 26}; 
#define IR_THRESHOLD 4050 

// In Config.h
// PID Waarden (Beginwaarden, moeten getuned worden)
#define KP 2.3 
#define KI 0.0
#define KD 16.0
//Kp = 2.3, Ki = 0.00, Kd = 16.0; 

#define BASE_SPEED_MAPPING 60 // Rustige snelheid voor het mappen
//#define BASE_SPEED_turns 100 // Verouderde macro, gebruik BASE_SPEED_TURNS
#define MAX_SPEED 180 //kan tot 255, maar 200 is vaak al snel genoeg

// --- Ultrasoon ---
#define TRIG_PIN 5
#define ECHO_PIN 18
#define DISTANCE_THRESHOLD 5.0 // cm

// --- Motoren (L298N / OT2065) ---
// Links
#define MOT_L_PWM 27
#define MOT_L_AIN1 14
#define MOT_L_AIN2 4
// Rechts
#define MOT_R_PWM 22
#define MOT_R_BIN1 21
#define MOT_R_BIN2 16

// PWM Kanalen (ESP32 specifiek)
#define PWM_FREQ 20000
#define PWM_RES 8
#define CH_LEFT 0
#define CH_RIGHT 1

// --- Motor Richting ---
#define LEFT_MOTOR_REVERSED true  // Deze motor zit fysiek 180 graden gedraaid
#define RIGHT_MOTOR_REVERSED false

// --- Encoders ---
#define ENC_L_A 12
#define ENC_L_B 13
#define ENC_R_A 15
#define ENC_R_B 2

#endif