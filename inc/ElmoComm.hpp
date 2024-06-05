#ifndef ELMOCOMM_H
#define ELMOCOMM_H

// Standard headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <math.h>
#include <bitset>
#include <chrono>
#include <algorithm>

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

// struct for general ELMO data
struct ELMOData{
  uint8_t OpMode;              // operation mode
  char port[1028];             // ethernet port container
  bool motor_control_switch;   // desired motor state
  int commStatus;              // communication status
  double freq;                 // frequency of control loop
  int32_t position[6];         // desried position commands from Laptop
  int32_t pos[6];              // encoder joint position from ELMO
  int32_t vel[6];              // encoder joint velocity from ELMO
  uint32_t inputs[6];          // inputs
  uint16_t controlword[6];     // control word of each motor
  uint16_t statusword[6];      // status word of each motor
};

// struct to hold out-going data, Laptop --> ELMO
// Position Control (x1600)
struct ELMOOut {
   int32_t position;     // "Target Position"
   int32_t inputs;       // "Digital Inputs"
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

// ELMO communication function
void *ELMOcommunication(void *data);

// callback function to check for ethercat communication errors
void *ecatcheck(void* why);

#endif
