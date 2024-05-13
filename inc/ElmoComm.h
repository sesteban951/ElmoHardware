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

// struct for general ELMO data
struct ELMOData{
  char port[1028];
  int commStatus;
  int16 torque[6];  // number of torque commands
  int32 torso;      // current torso position
  int32 torso_d;    // desired torso position
  int32 pos[6];     // encoder joint position
  int32 vel[6];     // encoder joint velocity
  uint32 inputs[6]; // encoder inputs
  uint16 status[6]; // status of each motor
};

// struct to hold out-going data, Laptop --> ELMO
struct ELMOOut {
    int16 torque;       // torque command
    uint16 controlword; // control word
};

// struct to hold in-coming data, ELMO --> Laptop
// 0x1A03
struct ELMOIn {
    int32 position;  // joint position
    uint32 inputs;   // encoder inputs
    int32 velocity;  // joint velocity
    uint16 status;   // status of motor
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