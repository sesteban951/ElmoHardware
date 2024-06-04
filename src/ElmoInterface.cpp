#include "../inc/ElmoInterface.hpp"

/* ELMO order of joints (physical daisy chain order)
  1. HFL  (Hip Frontal Left)
  2. HSL  (Hip Sagittal Left)
  3. HSR  (Hip Sagittal Right)
  4. KL   (Knee Left)
  5. HFR  (Hip Frontal Right)
  6. KR   (Knee Right)
*/ 

// function to intialize the ELMO motor controllers
void ELMOInterface::initELMO(uint8 opmode, double freq, char* port, pthread_t thread1, pthread_t thread2) {

    // Initialize the ELMO data struct
    this->data = (struct ELMOData *)malloc(sizeof(struct ELMOData));

    // set the operation mode
    this->data->OpMode = opmode;

    // set the frequency of the control loop
    this->data->freq = freq;

    // flip the motor switch to be on
    this->data->motor_control_switch = true;

    // Inital values to populate the ELMO data struct
    int32 pos0[6] = {0,0,0,0,0,0};
    int32 vel0[6] = {0,0,0,0,0,0};
    // int32 torque0[6] = {0,0,0,0,0,0};
    int32 position0[6] = {0,0,0,0,0,0};

    // Populate the ELMO data struct with the intial values
    memcpy(this->data->pos, pos0, sizeof(pos0));                // set the position to zero
    memcpy(this->data->vel, vel0, sizeof(vel0));                // set the velocity to zero
    memcpy(this->data->position, position0, sizeof(position0)); // set the torque to zero
    strcpy(this->data->port, port);                             // attach the ethernet port

    printf("SOEM (Simple Open EtherCAT Master)\nSetting Up ELMO drivers...\n");
    
    // Threading stuff to set each thread to the highest priority
    pthread_attr_t attr;
    struct sched_param param;
    
    // intialize the thread attribute
    pthread_attr_init(&attr);
    
    // set the thread to be FIFO
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    // set the thread priority to the maximum
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);

    // set the thread to be inheritable
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    /* Thread to catch ELMO errors and act appropriately */
    pthread_create(&thread1, &attr, &ecatcheck, (void (*)) &ctime);
    
    // /* Thread to communicate with ELMO. Send and receive data */
    pthread_create(&thread2, &attr, &ELMOcommunication, (void (*)) &this->data);

    std::cout << "Created threads for ELMO communication and error checking." << std::endl;

    // Wait for communication to be set up
    while(this->data->commStatus != 1) {
        switch (this->data->commStatus) {
            case -1: // If comm setup fails,
            std::cout << "Communication setup failed. Exiting..." << std::endl;
            exit(2);
        }
    };

    printf("Ready.\n");
    usleep(3000);
}

// function to that flips the motor control switch to off
void ELMOInterface::shutdownELMO() {

    // turn the desired motor switch to be off
    this->data->motor_control_switch = false;
}

// function to set the low level control gains
void ELMOInterface::setGains(JointGains gains) {

    // set the gains
    this->gains = gains;
}

// function to set the joint limits
void ELMOInterface::setLimits(JointLimits limits) {

    // set the limits
    this->limits = limits;
}

// function to get the ELMO status (reordered)
ELMOStatus ELMOInterface::getELMOStatus() {

    // declare variables for the incoming data
    uint16 joint_input[6];
    uint16 joint_control[6];
    uint16 joint_status[6];
    ELMOStatus tmp;

    // copy the incoming data to the declared variables
    memcpy(joint_input, this->data->inputs, sizeof(joint_input));
    memcpy(joint_control, this->data->controlword, sizeof(joint_control));
    memcpy(joint_status, this->data->statusword, sizeof(joint_status));

    // poulate the Eigen vector with reordered data
    tmp << joint_input[0],   // (HFL) Hip Frontal Left
           joint_input[1],   // (HSL) Hip Sagittal Left
           joint_input[3],   // (KL) Knee Left
           joint_input[4],   // (HFR) Hip Frontal Right
           joint_input[2],   // (HSR) Hip Sagittal Right
           joint_input[5],   // (KR) Knee Right
           joint_control[0], // (HFL) Hip Frontal Left
           joint_control[1], // (HSL) Hip Sagittal Left
           joint_control[3], // (KL) Knee Left
           joint_control[4], // (HFR) Hip Frontal Right
           joint_control[2], // (HSR) Hip Sagittal Right
           joint_control[5], // (KR) Knee Right
           joint_status[0],  // (HFL) Hip Frontal Left
           joint_status[1],  // (HSL) Hip Sagittal Left
           joint_status[3],  // (KL) Knee Left
           joint_status[4],  // (HFR) Hip Frontal Right
           joint_status[2],  // (HSR) Hip Sagittal Right
           joint_status[5];  // (KR) Knee Right

    return tmp;
}

// function to get the raw encoder data from ELMO
JointVec ELMOInterface::getEncoderData() {

    // declare variables for the incoming data
    int32 pos[6];
    int32 vel[6];
    JointVec tmp(12);

    // copy the incoming data to the declared variables
    memcpy(pos, this->data->pos, sizeof(pos));
    memcpy(vel, this->data->vel, sizeof(vel));

    // populate the Eigen vector with reordered data
    tmp.setZero();
    tmp <<  pos[0] * HIP_CONVERSION,  // (HFL) Hip Frontal Left
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

// function to compute the torque command
JointTorque ELMOInterface::computeTorque(JointVec joint_ref, JointTorque tau_ff) {

    // get the current joint state
    JointVec joint_data = this->getEncoderData();

    // intialize the torque vector
    JointTorque tau;
    double tau_HFL, tau_HSL, tau_KL, tau_HFR, tau_HSR, tau_KR;

    // saturate the reference joint angles
    if (joint_ref(0) < this->limits.q_min_HFL || joint_ref(0) > this->limits.q_max_HFL) {
        std::cout << "[WARNING] Joint HFL reference is out of bounds! Saturating." << std::endl;
        joint_ref(0) = std::min(std::max(joint_ref(0), this->limits.q_min_HFL), this->limits.q_max_HFL);
    }
    if (joint_ref(1) < this->limits.q_min_HSL || joint_ref(1) > this->limits.q_max_HSL) {
        std::cout << "[WARNING] Joint HSL reference is out of bounds! Saturating." << std::endl;
        joint_ref(1) = std::min(std::max(joint_ref(1), this->limits.q_min_HSL), this->limits.q_max_HSL);
    }
    if (joint_ref(2) < this->limits.q_min_KL || joint_ref(2) > this->limits.q_max_KL) {
        std::cout << "[WARNING] Joint KL reference is out of bounds! Saturating." << std::endl;
        joint_ref(2) = std::min(std::max(joint_ref(2), this->limits.q_min_KL), this->limits.q_max_KL);
    }
    if (joint_ref(3) < this->limits.q_min_HFR || joint_ref(3) > this->limits.q_max_HFR) {
        std::cout << "[WARNING] Joint HFR reference is out of bounds! Saturating." << std::endl;
        joint_ref(3) = std::min(std::max(joint_ref(3), this->limits.q_min_HFR), this->limits.q_max_HFR);
    }
    if (joint_ref(4) < this->limits.q_min_HSR || joint_ref(4) > this->limits.q_max_HSR) {
        std::cout << "[WARNING] Joint HSR reference is out of bounds! Saturating." << std::endl;
        joint_ref(4) = std::min(std::max(joint_ref(4), this->limits.q_min_HSR), this->limits.q_max_HSR);
    }
    if (joint_ref(5) < this->limits.q_min_KR || joint_ref(5) > this->limits.q_max_KR) {
        std::cout << "[WARNING] Joint KR reference is out of bounds! Saturating." << std::endl;
        joint_ref(5) = std::min(std::max(joint_ref(5), this->limits.q_min_KR), this->limits.q_max_KR);
    }

    // // saturate the reference joint velocities
    // if (joint_ref(6) < this->limits.qd_min_HFL || joint_ref(6) > this->limits.qd_max_HFL) {
    //     std::cout << "[WARNING] Joint HFL reference velocity is out of bounds! Saturating." << std::endl;
    //     joint_ref(6) = std::min(std::max(joint_ref(6), this->limits.qd_min_HFL), this->limits.qd_max_HFL);
    // }
    // if (joint_ref(7) < this->limits.qd_min_HSL || joint_ref(7) > this->limits.qd_max_HSL) {
    //     std::cout << "[WARNING] Joint HSL reference velocity is out of bounds! Saturating." << std::endl;
    //     joint_ref(7) = std::min(std::max(joint_ref(7), this->limits.qd_min_HSL), this->limits.qd_max_HSL);
    // }
    // if (joint_ref(8) < this->limits.qd_min_KL || joint_ref(8) > this->limits.qd_max_KL) {
    //     std::cout << "[WARNING] Joint KL reference velocity is out of bounds! Saturating." << std::endl;
    //     joint_ref(8) = std::min(std::max(joint_ref(8), this->limits.qd_min_KL), this->limits.qd_max_KL);
    // }
    // if (joint_ref(9) < this->limits.qd_min_HFR || joint_ref(9) > this->limits.qd_max_HFR) {
    //     std::cout << "[WARNING] Joint HFR reference velocity is out of bounds! Saturating." << std::endl;
    //     joint_ref(9) = std::min(std::max(joint_ref(9), this->limits.qd_min_HFR), this->limits.qd_max_HFR);
    // }
    // if (joint_ref(10) < this->limits.qd_min_HSR || joint_ref(10) > this->limits.qd_max_HSR) {
    //     std::cout << "[WARNING] Joint HSR reference velocity is out of bounds! Saturating." << std::endl;
    //     joint_ref(10) = std::min(std::max(joint_ref(10), this->limits.qd_min_HSR), this->limits.qd_max_HSR);
    // }
    // if (joint_ref(11) < this->limits.qd_min_KR || joint_ref(11) > this->limits.qd_max_KR) {
    //     std::cout << "[WARNING] Joint KR reference velocity is out of bounds! Saturating." << std::endl;
    //     joint_ref(11) = std::min(std::max(joint_ref(11), this->limits.qd_min_KR), this->limits.qd_max_KR);
    // }
    
    // compute the torque for each joint
    tau_HFL = this->gains.Kp_HFL * (joint_ref(0) - joint_data(0)) 
            + this->gains.Kd_HFL * (joint_ref(6) - joint_data(6)) 
            + this->gains.Kff_HFL * tau_ff(0);

    tau_HSL = this->gains.Kp_HSL * (joint_ref(1) - joint_data(1))
            + this->gains.Kd_HSL * (joint_ref(7) - joint_data(7))
            + this->gains.Kff_HSL * tau_ff(1);

    tau_KL =  this->gains.Kp_KL * (joint_ref(2) - joint_data(2))
            + this->gains.Kd_KL * (joint_ref(8) - joint_data(8))
            + this->gains.Kff_KL * tau_ff(2);

    tau_HFR = this->gains.Kp_HFR * (joint_ref(3) - joint_data(3))
            + this->gains.Kd_HFR * (joint_ref(9) - joint_data(9))
            + this->gains.Kff_HFR * tau_ff(3);

    tau_HSR = this->gains.Kp_HSR * (joint_ref(4) - joint_data(4))
            + this->gains.Kd_HSR * (joint_ref(10) - joint_data(10))
            + this->gains.Kff_HSR * tau_ff(4);

    tau_KR =  this->gains.Kp_KR * (joint_ref(5) - joint_data(5))
            + this->gains.Kd_KR * (joint_ref(11) - joint_data(11))
            + this->gains.Kff_KR * tau_ff(5); 

    // check that we have not exceeded the joint limits 
    if (joint_data(0) < this->limits.q_min_HFL || joint_data(0) > this->limits.q_max_HFL) {
        std::cout << "[WARNING] Joint HFL is out of bounds! Setting torque to zero." << std::endl;
        tau_HFL = 0.0;
    }
    if (joint_data(1) < this->limits.q_min_HSL || joint_data(1) > this->limits.q_max_HSL) {
        std::cout << "[WARNING] Joint HFL is out of bounds! Setting torque to zero." << std::endl;
        tau_HSL = 0.0;
    }
    if (joint_data(2) < this->limits.q_min_KL || joint_data(2) > this->limits.q_max_KL) {
        std::cout << "[WARNING] Joint KL is out of bounds! Setting torque to zero." << std::endl;
        tau_KL = 0.0;
    }
    if (joint_data(3) < this->limits.q_min_HFR || joint_data(3) > this->limits.q_max_HFR) {
        std::cout << "[WARNING] Joint HFR is out of bounds! Setting torque to zero." << std::endl;
        tau_HFR = 0.0;
    }
    if (joint_data(4) < this->limits.q_min_HSR || joint_data(4) > this->limits.q_max_HSR) {
        std::cout << "[WARNING] Joint HSR is out of bounds! Setting torque to zero." << std::endl;
        tau_HSR = 0.0;
    }
    if (joint_data(5) < this->limits.q_min_KR || joint_data(5) > this->limits.q_max_KR) {
        std::cout << "[WARNING] Joint KR is out of bounds! Setting torque to zero." << std::endl;
        tau_KR = 0.0;
    }

    // return the torque vector
    tau << tau_HFL, tau_HSL, tau_KL, tau_HFR, tau_HSR, tau_KR;

    return tau;
}

// function to send target torque to the ELMO
void ELMOInterface::sendPosition(JointTorque torque) {
    
    // unpack the torque vector
    double p_HFL, p_HSL, p_KL, p_HFR, p_HSR, p_KR;
    p_HFL = torque(0);
    p_HSL = torque(1);
    p_KL  = torque(2);
    p_HFR = torque(3);
    p_HSR = torque(4);
    p_KR  = torque(5);

    // reorder the torques to match the ELMO daisy chain order
    JointTorque position_applied;
    position_applied << p_HFL, p_HSL, p_HSR, p_KL, p_HFR, p_KR;

    // populate the data pointer with the torque values
    for (int i = 0; i < 6; i++) {
        // this->data->torque[i] = (int16) torque_applied(i);
        this->data->position[i] = (int16) position_applied(i);
    }
}
