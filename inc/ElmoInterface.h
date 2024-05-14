#ifndef ELMOINTERFACE_H
#define ELMOINTERFACE_H

// we will also use the ELMO communication header
#include "ElmoComm.h"

// standard headers
#include <Eigen/Dense>

/* ELMO order of joints (raw order)
  1. HFL  (Hip Frontal Left)
  2. HSL  (Hip Sagittal Left)
  3. HSR  (Hip Sagittal Right)
  4. KL   (Knee Left)
  5. HFR  (Hip Frontal Right)
  6. KR   (Knee Right)
*/ 

// converison factors for reading the encoder data
#define CPR 8192.0    // counts per revolution of the encoder (RLS RMB20)
#define HIP_GR 30.0   // gear ratio for the hip actuators
#define KNEE_GR 50.0  // gear ratio for the knee actuators

// actuator conversion factors
#define HIP_CONVERSION 2*M_PI/CPR/HIP_GR
#define KNEE_CONVERSION 2*M_PI/CPR/KNEE_GR

//  A class that enables communication between the computer and motor controllers
class ELMOInterface {
    
    public:

        // struct to hold ELMO data
        struct ELMOData *data;

        // use default constructor / desctructors
        ELMOInterface() {};
        ~ELMOInterface() {};

        // function to initialize ELMO
        void initELMO(char* port, pthread_t thread1, pthread_t thread2);

        // function to get encoder data
        Eigen::VectorXd getEncoderData();

};

#endif