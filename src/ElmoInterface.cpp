#include "../inc/ElmoInterface.hpp"
#include <iomanip>


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
    int32_t pos0[6] = {0,0,0,0,0,0};
    int32_t vel0[6] = {0,0,0,0,0,0};
    int32_t position0[6] = {0,0,0,0,0,0};

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

// function to check if the joint limits are violated
JointCommand ELMOInterface::checkPosition(JointCommand position_ref) {

    // unpack the torque vector
    double p_HFL, p_HSL, p_KL, p_HFR, p_HSR, p_KR;
    p_HFL = position_ref(0);
    p_HSL = position_ref(1);
    p_KL  = position_ref(2);
    p_HFR = position_ref(3);
    p_HSR = position_ref(4);
    p_KR  = position_ref(5);

    // saturate the position values if the values are exceeded
    if (p_HFL < this->limits.q_min_HFL || p_HFL > this->limits.q_max_HFL) {
        std::cout << "Joint limit violation: Hip Frontal Left (HFL). Saturating.\n" << std::endl;
        p_HFL = std::max(this->limits.q_min_HFL, std::min(this->limits.q_max_HFL, p_HFL));
    }
    if (p_HSL < this->limits.q_min_HSL || p_HSL > this->limits.q_max_HSL) {
        std::cout << "Joint limit violation: Hip Sagittal Left (HSL). Saturating.\n" << std::endl;
        p_HSL = std::max(this->limits.q_min_HSL, std::min(this->limits.q_max_HSL, p_HSL));
    }
    if (p_KL < this->limits.q_min_KL || p_KL > this->limits.q_max_KL) {
        std::cout << "Joint limit violation: Knee Left (KL). Saturating.\n" << std::endl;
        p_KL = std::max(this->limits.q_min_KL, std::min(this->limits.q_max_KL, p_KL));
    }
    if (p_HFR < this->limits.q_min_HFR || p_HFR > this->limits.q_max_HFR) {
        std::cout << "Joint limit violation: Hip Frontal Right (HFR). Saturating.\n" << std::endl;
        p_HFR = std::max(this->limits.q_min_HFR, std::min(this->limits.q_max_HFR, p_HFR));
    }
    if (p_HSR < this->limits.q_min_HSR || p_HSR > this->limits.q_max_HSR) {
        std::cout << "Joint limit violation: Hip Sagittal Right (HSR). Saturating.\n" << std::endl;
        p_HSR = std::max(this->limits.q_min_HSR, std::min(this->limits.q_max_HSR, p_HSR));
    }
    if (p_KR < this->limits.q_min_KR || p_KR > this->limits.q_max_KR) {
        std::cout << "Joint limit violation: Knee Right (KR). Saturating.\n" << std::endl;
        p_KR = std::max(this->limits.q_min_KR, std::min(this->limits.q_max_KR, p_KR));
    }

    JointCommand position_checked;
    position_checked << p_HFL, p_HSL, p_KL, p_HFR, p_HSR, p_KR;

    return position_checked;
}

// function to send target torque to the ELMO
void ELMOInterface::sendPosition(JointCommand position_ref) {

    // check for joint limit violations
    position_ref = checkPosition(position_ref);

    // unpack the torque vector
    double p_HFL, p_HSL, p_KL, p_HFR, p_HSR, p_KR;
    p_HFL = position_ref(0) / HIP_CONVERSION;
    p_HSL = position_ref(1) / HIP_CONVERSION;
    p_KL  = position_ref(2) / KNEE_CONVERSION;
    p_HFR = position_ref(3) / HIP_CONVERSION;
    p_HSR = position_ref(4) / HIP_CONVERSION;
    p_KR  = position_ref(5) / KNEE_CONVERSION;

    // reorder the commands to match the ELMO daisy chain order
    JointCommand position_applied;
    position_applied << p_HFL, p_HSL, p_HSR, p_KL, p_HFR, p_KR;

    // populate the data pointer with the torque values
    for (int i = 0; i < 6; i++) {
        this->data->position[i] = (int32_t) position_applied(i);
        if (i == 5) {
            std::cout << "Knee Right (KR) position applied: " << position_applied(i) << std::endl;
            std::cout << "Knee Right (KR) data_position: " << position_applied(i) << std::endl;
        }
    }
}
