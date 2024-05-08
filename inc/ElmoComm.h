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

// General ELMO header
struct Data{
  char port[1028];
  int commStatus;
  int16 torque[4];
  int32 torso;
  int32 torso_d;
  int32 pos[4];
  int32 vel[4];
  uint32 inputs[4];
  uint16 status[4];
};

// struct to hold out-going data
struct ELMOOut {
    int16 torque;
    uint16 controlword;
};

// struct to hold in-coming data
struct ELMOIn {
    int32 position;
    uint32 inputs;
    int32 velocity;
    uint16 status;
};

// function to initialize ELMO
void initELMO(char* port);

// ELMO communication function
void *ELMOcommunication(void *data);

// callback function to check for ethercat communication errors
void *ecatcheck(void* why);

#endif