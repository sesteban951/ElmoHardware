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
  int commStatus;
  int16 torque[6];  // desried torque commands
  int32 pos[6];     // encoder joint position from ELMO
  int32 vel[6];     // encoder joint velocity from ELMO
  uint32 inputs[6]; // encoder inputs
  uint16 status[6]; // status of each motor
};

// // struct to hold out-going data, Laptop --> ELMO
struct ELMOOut {
    int16 torque;       // "Torque Command"
    uint16 controlword; // "Control Word", converted to binary and use DS402 SM, 6040
};

// struct to hold out-going data, Laptop --> ELMO
// struct ELMOOut {
//     int32 position;       // "Position Command"
//     int32 inputs;       // "Digital Inputs"
//     uint16 controlword; // "Control Word", converted to binary and use DS402 SM, 6040
// };

// struct to hold in-coming data, ELMO --> Laptop
// PDO Index: 0x1A03
struct ELMOIn {
    int32 position;  // "Position Actual Value"
    uint32 inputs;   // "Digital Inputs"
    int32 velocity;  // "Velocity Actual Value"
    uint16 status;   // "Status Word", converted to binary and use DS402 SM, 6041
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