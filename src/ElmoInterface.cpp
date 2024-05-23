#include "../inc/ElmoInterface.h"

#include <thread>
#include <chrono>

// function to intialize the ELMO motor controllers
void ELMOInterface::initELMO(char* port, pthread_t thread1, pthread_t thread2) {

    // Initialize the ELMO data struct
    this->data = (struct ELMOData *)malloc(sizeof(struct ELMOData));

    // Inital values to populate the ELMO data struct
    int32 pos0[6] = {0,0,0,0,0,0};
    int32 vel0[6] = {0,0,0,0,0,0};
    int32 torque0[6] = {0,0,0,0,0,0};

    // Populate the ELMO data struct with the intial values
    memcpy(this->data->pos, pos0, sizeof(pos0));        // set the position to zero
    memcpy(this->data->pos, vel0, sizeof(vel0));        // set the velocity to zero
    memcpy(this->data->pos, torque0, sizeof(torque0));  // set the torque to zero
    strcpy(this->data->port, port);                     // attach the ethernet port

    printf("SOEM (Simple Open EtherCAT Master)\nSetting Up ELMO drivers...\n");

    // std::this_thread::sleep_for(std::chrono::seconds(1));
    
    /* Thread to catch ELMO errors and act appropriately */
    pthread_create( &thread1, NULL, &ecatcheck, (void (*)) &ctime);
    
    // std::this_thread::sleep_for(std::chrono::seconds(1)); 
    
    /* Thread to communicate with ELMO. Send and receive data */
    pthread_create( &thread2, NULL, &ELMOcommunication, (void (*)) &this->data);

    // std::this_thread::sleep_for(std::chrono::seconds(1)); 

    // Wait for communication to be set up
    while(this->data->commStatus != 1) {
    switch (this->data->commStatus) {
        case -1: // If comm setup fails,
        std::cout << "Communication setup failed. Exiting..." << std::endl;
        exit(2);
    }
    };

    printf("Ready.\n");
    usleep(1000);
}

// function to set the low level control gains
void ELMOInterface::setGains(JointGains gains) {

    // set the gains
    this->gains = gains;
}

// function to get the raw encoder data from ELMO (reordered)
JointVec ELMOInterface::getEncoderData() {

    // declare variables for the incoming data
    int32 pos[6];
    int32 vel[6];
    JointVec tmp(12);

    // copy the incoming data to the declared variables
    memcpy(pos, this->data->pos, sizeof(pos));
    memcpy(vel, this->data->vel, sizeof(vel));

    // poulate the Eigen vector with reordered data
    tmp.setZero();
    tmp << pos[0] * HIP_CONVERSION,  // (HFL) Hip Frontal Left
            pos[1] * HIP_CONVERSION,  // (HSL) Hip Sagittal Left
            pos[3] * KNEE_CONVERSION, // (KL) Knee Left
            pos[4] * HIP_CONVERSION,  // (HFR) Hip Frontal Right
            pos[2] * HIP_CONVERSION,  // (HSR) Hip Sagittal Right
            pos[5] * KNEE_CONVERSION, // (KR) Knee Right
            vel[0] * HIP_CONVERSION,  // (HFL) Hip Frontal Left
            vel[1] * HIP_CONVERSION,  // (HSL) Hip Sagittal Left
            vel[3] * KNEE_CONVERSION, // (KL) Knee Left
            vel[4] * HIP_CONVERSION,  // (HFR) Hip Frontal Right
            vel[2] * HIP_CONVERSION,  // (HSR) Hip Sagittal Right
            vel[5] * KNEE_CONVERSION; // (KR) Knee Right

    return tmp;
}

// function to compute the torque command (ordered the right way)
JointTorque ELMOInterface::computeTorque(JointVec joint_ref, JointTorque tau_ff) {

    // get the current joint state
    JointVec joint_data = this->getEncoderData();

    // intialize the torque vector
    JointTorque tau;
    double tau_HFL, tau_HSL, tau_KL, tau_HFR, tau_HSR, tau_KR;

    // compute the torque for each joint
    tau_HFL = this->gains.Kp_HFL * (joint_ref(0) - joint_data(0)) 
            + this->gains.Kd_HFL * (joint_ref(6) - joint_data(6)) 
            + this->gains.Kff_HFL * tau_ff(0);

    tau_HSL = this->gains.Kp_HSL * (joint_ref(1) - joint_data(1))
            + this->gains.Kd_HSL * (joint_ref(7) - joint_data(7))
            + this->gains.Kff_HSL * tau_ff(1);

    tau_KL = this->gains.Kp_KL * (joint_ref(2) - joint_data(2))
            + this->gains.Kd_KL * (joint_ref(8) - joint_data(8))
            + this->gains.Kff_KL * tau_ff(2);

    tau_HFR = this->gains.Kp_HFR * (joint_ref(3) - joint_data(3))
            + this->gains.Kd_HFR * (joint_ref(9) - joint_data(9))
            + this->gains.Kff_HFR * tau_ff(3);

    tau_HSR = this->gains.Kp_HSR * (joint_ref(4) - joint_data(4))
            + this->gains.Kd_HSR * (joint_ref(10) - joint_data(10))
            + this->gains.Kff_HSR * tau_ff(4);

    tau_KR = this->gains.Kp_KR * (joint_ref(5) - joint_data(5))
            + this->gains.Kd_KR * (joint_ref(11) - joint_data(11))
            + this->gains.Kff_KR * tau_ff(5); 

    // return the torque vector
    tau << tau_HFL, tau_HSL, tau_KL, tau_HFR, tau_HSR, tau_KR;

    // DEBUGING
    // if any joint deviates 20 degrees from zero, set all torques to zero
    for (int i = 0; i < 6; i++) {

        if (abs(joint_data(i)) > 0.35) {
            tau.setZero();
            std::cout << "Joint " << i << " is out of bounds! Setting all torques to zero." << std::endl;
        }
    }

    return tau;
}

// function to send target torque to the ELMO
void ELMOInterface::sendTorque(JointTorque torque) {
    
    // unpack the torque vector
    double tau_HFL, tau_HSL, tau_KL, tau_HFR, tau_HSR, tau_KR;
    tau_HFL = torque(0);
    tau_HSL = torque(1);
    tau_KL  = torque(2);
    tau_HFR = torque(3);
    tau_HSR = torque(4);
    tau_KR  = torque(5);

    // reorder the torques to match the ELMO daisy chain order
    JointTorque torque_applied;
    torque_applied << tau_HFL, tau_HSL, tau_HSR, tau_KL, tau_HFR, tau_KR;

    // populate the data pointer with the torque values (cast to int16)
    for (int i = 0; i < 6; i++) {
        this->data->torque[i] = (int16) torque_applied(i);
    }
}
