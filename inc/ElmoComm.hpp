#ifndef ELMOCOMM_H
#define ELMOCOMM_H

// Standard headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <bitset>
#include <chrono>

// Ethercat headers
#include <ethercat.h>
#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatdc.h>
#include <ethercatcoe.h>
#include <ethercatfoe.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>

/* ELMO order of joints (physical default order)
  1. HFL  (Hip Frontal Left)
  2. HSL  (Hip Sagittal Left)
  3. HSR  (Hip Sagittal Right)
  4. KL   (Knee Left)
  5. HFR  (Hip Frontal Right)
  6. KR   (Knee Right)
*/ 

// struct for general ELMO data
struct ELMOData{
  char port[1028];
  int commStatus;   // communication status
  int16 torque[6];  // desried torque commands
  int32 pos[6];     // encoder joint position from ELMO
  int32 vel[6];     // encoder joint velocity from ELMO
  uint32 inputs[6]; // inputs
  uint16 controlword[6]; // status wordof each motor
  uint16 statusword[6]; // status wordof each motor
};

// struct to hold out-going data, Laptop --> ELMO
// Torque Control (x1602)
struct ELMOOut {
   int16_t torque;       // "Torque Command"
   uint16_t controlword; // "Control Word", converted to binary and use DS402 SM, 6040
};

// struct to hold in-coming data, ELMO --> Laptop
// Pos and Vel (0x1A03)
struct ELMOIn {
    int32_t position;  // "Position Actual Value"
    uint32_t inputs;   // "Digital Inputs"
    int32_t velocity;  // "Velocity Actual Value"
    uint16_t status;   // "Status Word", converted to binary and use DS402 SM, 6041
};

// struct for program state
enum programState {Initializing, 
                   Running, 
                   Exit};

// ELMO communication function
void *ELMOcommunication(void *data);

// callback function to check for ethercat communication errors
void *ecatcheck(void* why);

#endif
