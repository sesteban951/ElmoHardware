#ifndef ELMOINTERFACE_H
#define ELMOINTERFACE_H

// we will also use the ELMO communication header
#include "ElmoComm.hpp"

// standard headers
#include <Eigen/Dense>
#include <chrono>

// maximum motor wait to initialize time
#define MAX_MOTOR_INIT_WAIT_TIME 10.0

// converison factors for reading the encoder data
const double CPR = 8192.0;    // counts per revolution of the encoder (RLS RMB20)
const double HIP_GR = 30.0;   // gear ratio for the hip actuators
const double KNEE_GR = 50.0;  // gear ratio for the knee actuators

// actuator conversion factors
const double HIP_CONVERSION = 2*M_PI/CPR/HIP_GR;
const double KNEE_CONVERSION = 2*M_PI/CPR/KNEE_GR;

// struct for joint limits
struct JointLimits {

    // Hip Frontal Left (HFL)
    double q_max_HFL; 
    double q_min_HFL;
    double qd_limit_HFL;

    // Hip Sagittal Left (HSL)
    double q_max_HSL; 
    double q_min_HSL;
    double qd_limit_HSL;

    // Knee Left (KL)
    double q_max_KL;  
    double q_min_KL;
    double qd_limit_KL;

    // Hip Frontal Right (HFR)
    double q_max_HFR; 
    double q_min_HFR;
    double qd_limit_HFR;

    // Hip Sagittal Right (HSR)
    double q_max_HSR; 
    double q_min_HSR;
    double qd_limit_HSR;

    // Knee Right (KR)
    double q_max_KR;  
    double q_min_KR;
    double qd_limit_KR;
};

// variable for joint data and elmo status
typedef Eigen::Matrix< double, 12, 1> JointEncoderVec;    // vector for joint state
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

        // function to set the low level limits
        void setLimits(JointLimits limits, double freq);

        // function to get the ELMO status
        ELMOStatus getELMOStatus();

        // function to get encoder data
        JointEncoderVec getEncoderData();

        // functions to compute and send target pos to the ELMO
        JointCommand checkPosition(JointCommand position_ref);
        void sendPosition(JointCommand position);

    private:

        // struct to hold ELMO data
        struct ELMOData *data;

        //struct to hold the joint limits
        JointLimits limits;

        // to rate limit the joints (previous applied position)
        JointCommand prev_pos_app;

        // maximum allowable increment in joint positions
        double max_delta_pos_HFL, max_delta_pos_HSL, max_delta_pos_KL, 
               max_delta_pos_HFR, max_delta_pos_HSR, max_delta_pos_KR;

        // function to wait for all ELMOs to be armed
        int waitForELMOarmed();
};

#endif