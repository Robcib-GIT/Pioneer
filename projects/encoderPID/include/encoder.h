#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "pinout.h"


void encoder_setup();
void IRAM_ATTR doEncode1A();
void IRAM_ATTR doEncode1B();
void IRAM_ATTR doEncode2A();
void IRAM_ATTR doEncode2B();

#endif