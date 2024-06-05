#ifndef ELMOINTERFACE_H
#define ELMOINTERFACE_H

// we will also use the ELMO communication header
#include "ElmoComm.hpp"

// standard headers
#include <Eigen/Dense>

// converison factors for reading the encoder data
const double CPR = 8192.0;    // counts per revolution of the encoder (RLS RMB20)
const double HIP_GR = 30.0;   // gear ratio for the hip actuators
const double KNEE_GR = 50.0;  // gear ratio for the knee actuators

// actuator conversion factors
const double HIP_CONVERSION = 2*M_PI/CPR/HIP_GR;
const double KNEE_CONVERSION = 2*M_PI/CPR/KNEE_GR;

// current to torque factor (linear) [N-m/A]
const double CURRENT_TO_TORQUE = 0.03125;

// struct for joint gains
struct JointGains {

    // proportional gains
    double Kp_HFL;
    double Kp_HSL;
    double Kp_KL;
    double Kp_HFR;
    double Kp_HSR;
    double Kp_KR;

    // derivative gains
    double Kd_HFL;
    double Kd_HSL;
    double Kd_KL;
    double Kd_HFR;
    double Kd_HSR;
    double Kd_KR;

    // feedforward torque scaling
    double Kff_HFL;
    double Kff_HSL;
    double Kff_KL;
    double Kff_HFR;
    double Kff_HSR;
    double Kff_KR;
};

// struct for joint gains
struct JointLimits {

    // Hip Frontal Left (HFL)
    double q_max_HFL; 
    double q_min_HFL;

    // Hip Sagittal Left (HSL)
    double q_max_HSL; 
    double q_min_HSL;

    // Knee Left (KL)
    double q_max_KL;  
    double q_min_KL;

    // Hip Frontal Right (HFR)
    double q_max_HFR; 
    double q_min_HFR;

    // Hip Sagittal Right (HSR)
    double q_max_HSR; 
    double q_min_HSR;

    // Knee Right (KR)
    double q_max_KR;  
    double q_min_KR;
};

// variable for joint data
typedef Eigen::Matrix< double, 12, 1> JointVec;    // vector for joint state
typedef Eigen::Matrix< double, 6, 1> JointCommand;  // vector for feedforward torque
typedef Eigen::Matrix< double, 18, 1> ELMOStatus;   // status of each motor controller

//  A class that enables communication between the computer and motor controllers
class ELMOInterface {
    
    public:

        // constructor / desctructors
        ELMOInterface() {};
        ~ELMOInterface() {};

        // function to initialize/shutdown ELMO
        void initELMO(uint8 opmode, double freq, char* port, pthread_t thread1, pthread_t thread2);
        void shutdownELMO();

        // function to set the low level gains and limits
        void setLimits(JointLimits limits);

        // function to get the ELMO status
        ELMOStatus getELMOStatus();

        // function to get encoder data
        JointVec getEncoderData();

        // functions to compute and send target pos to the ELMO
        JointCommand checkPosition(JointCommand position_ref);
        void sendPosition(JointCommand position);

    private:

        // struct to hold ELMO data
        struct ELMOData *data;

        // struct to hold the joint gains
        JointGains gains;

        //struct to hold the joint limits
        JointLimits limits;
};

#endif