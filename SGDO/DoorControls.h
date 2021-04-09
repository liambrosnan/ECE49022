#include <Arduino.h>
#include "cc1101.h"

/*------------------------------ DEFS ------------------------------*/

//Radio and interrupt configuration
#define RADIO_CHANNEL             16
#define THIS_DEVICE_ADDRESS       22
#define DESINATION_DEVICE_ADDRESS BROADCAST_ADDRESS // Broadcast

// External intterupt pin for GDO0
#define GDO0_INTERRUPT_PIN 13 //SAM may cause issues, unused anyways

//Potentionally example code only
//These values are in the datasheet
#define RT0 100000   // Ω
#define B 3977      // K
#define VCC 3.3    //Supply voltage
#define R 160000  //R=10KΩ


/*------------------------------ Fn's ------------------------------*/

void setupDoorControl();
void changeDoorState(int fixedCode);
void pairDevice();
