#ifndef STARTSTOP_H
#define STARTSTOP_H

#include <Arduino.h>
#include "types.h" // Zorg dat types.h geladen wordt voor de enum

// VERWIJDER de enum FinishState hier, die staat al in types.h

FinishState checkFinishStatus();
bool handleStartLogic();
void resetStartSequence();

#endif