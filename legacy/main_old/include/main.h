#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#include "motorController.h"
#include "pinout.h"
#include "microROS.h"

extern motorController motor1;
extern motorController motor2;

// Cabeceras de funciones
void IRAM_ATTR doEncode1();
void IRAM_ATTR doEncode2();

 
#endif // MAIN_H