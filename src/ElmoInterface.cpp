#include "../inc/ElmoInterface.hpp"

/* ELMO order of joints (physical daisy chain order)
  1. HFL  (Hip Frontal Left)
  2. HSL  (Hip Sagittal Left)
  3. HSR  (Hip Sagittal Right)
  4. KL   (Knee Left)
  5. HFR  (Hip Frontal Right)
  6. KR   (Knee Right)
*/ 

// function to wait for all ELMOs to be ARMED
int ELMOInterface::waitForELMOarmed() {

    // idicator, 1 = success, 0 = failure
    int success;

    // container for the ELMO status
    ELMOStatus diagnostics;
    double elmo_status[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // for excedding the maximum wait time
    auto t1 = std::chrono::high_resolution_clock::now();
    double wait_time = 0;

    // wait unitl the ELMOs are ready
    std::cout << "Waiting for ELMOs to be ready..." << std::endl;
    bool elmo1, elmo2, elmo3, elmo4, elmo5, elmo6;
    while(true) {

        // break if all ELMOs are ready
        if (elmo1 && elmo2 && elmo3 && elmo4 && elmo5 && elmo6) {
            success = 1;
            break;
        }

        // check how long we've been waiting
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        wait_time = duration.count() / 1'000'000.0;

        // if we've been waiting for too long, exit, something is wrong
        if (wait_time >= MAX_MOTOR_INIT_WAIT_TIME) {
            success = 0;
            break;
        }

        // get the newest diagnostic data and wait until all ELMOs ready
        diagnostics = getELMOStatus();
        for (int i = 0; i < 6; ++i) {
            elmo_status[i] = diagnostics(i + 12);
        }

        // check the statusof each ELMO
        elmo1 = (elmo_status[0] == 4663.0);
        elmo2 = (elmo_status[1] == 4663.0);
        elmo3 = (elmo_status[2] == 4663.0);
        elmo4 = (elmo_status[3] == 4663.0);
        elmo5 = (elmo_status[4] == 4663.0);
        elmo6 = (elmo_status[5] == 4663.0);

        // print hte current ELMO Status
        std::cout << "-----------------------------" << std::endl;
        std::cout << "ELMO Status: " << elmo1 << ", " << elmo2 << ", " << elmo3 << ", " << elmo4 << ", " << elmo5 << ", " << elmo6 << std::endl;

        usleep(500'000);
    }

    return success;
}

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

    // intialize the previous joint position for rate limiting
    this->prev_pos_app.setZero();

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
    
    usleep(3000);

    // Wait for all ELMOs to be ARMED
    int success = waitForELMOarmed();

    if (success == 1) {
        std::cout << "All ELMOs are ready!" << std::endl;
    } else {
        std::cout << "ELMOs are not ready after " << MAX_MOTOR_INIT_WAIT_TIME << " seconds." << std::endl; 
        std::cout << "Check ELMOs. Exiting ELMO Interface." << std::endl;
        exit(1);
    }

    usleep(3000);
}

// function to that flips the motor control switch to off
void ELMOInterface::shutdownELMO() {

    // turn the desired motor switch to be off
    this->data->motor_control_switch = false;
    
    usleep(5000);
}

// function to set the joint limits
void ELMOInterface::setLimits(JointLimits limits, double freq) {

    // set the limits
    this->limits = limits;

    // set the maximum allowable change in position
    this->max_delta_pos_HFL = limits.qd_limit_HFL / freq;
    this->max_delta_pos_HSL = limits.qd_limit_HSL / freq;
    this->max_delta_pos_KL  = limits.qd_limit_KL / freq;
    this->max_delta_pos_HFR = limits.qd_limit_HFR / freq;
    this->max_delta_pos_HSR = limits.qd_limit_HSR / freq;
    this->max_delta_pos_KR  = limits.qd_limit_KR / freq;
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
JointEncoderVec ELMOInterface::getEncoderData() {

    // declare variables for the incoming data
    int32 current_pos[6];
    int32 current_vel[6];
    JointEncoderVec tmp(12);

    // copy the incoming data to the declared variables
    memcpy(current_pos, this->data->pos, sizeof(current_pos));
    memcpy(current_vel, this->data->vel, sizeof(current_vel));

    // populate the Eigen vector with reordered data
    tmp <<  current_pos[0] * HIP_CONVERSION,  // (HFL) Hip Frontal Left
            current_pos[1] * HIP_CONVERSION,  // (HSL) Hip Sagittal Left
            current_pos[3] * KNEE_CONVERSION, // (KL) Knee Left
            current_pos[4] * HIP_CONVERSION,  // (HFR) Hip Frontal Right
            current_pos[2] * HIP_CONVERSION,  // (HSR) Hip Sagittal Right
            current_pos[5] * KNEE_CONVERSION, // (KR) Knee Right
            current_vel[0] * HIP_CONVERSION,  // (HFL) Hip Frontal Left
            current_vel[1] * HIP_CONVERSION,  // (HSL) Hip Sagittal Left
            current_vel[3] * KNEE_CONVERSION, // (KL) Knee Left
            current_vel[4] * HIP_CONVERSION,  // (HFR) Hip Frontal Right
            current_vel[2] * HIP_CONVERSION,  // (HSR) Hip Sagittal Right
            current_vel[5] * KNEE_CONVERSION; // (KR) Knee Right

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

    // compute the signed position increment from last applied position to new reference position
    double delta_p_HFL, delta_p_HSL, delta_p_KL, delta_p_HFR, delta_p_HSR, delta_p_KR;
    delta_p_HFL = p_HFL - this->prev_pos_app(0);
    delta_p_HSL = p_HSL - this->prev_pos_app(1);
    delta_p_KL  = p_KL  - this->prev_pos_app(2);
    delta_p_HFR = p_HFR - this->prev_pos_app(3);
    delta_p_HSR = p_HSR - this->prev_pos_app(4);
    delta_p_KR  = p_KR  - this->prev_pos_app(5);

    // saturate the posiiton values based on the rate limits
    if (abs(delta_p_HFL) > this->max_delta_pos_HFL) {
        std::cout << "Rate limit violation: Hip Frontal Left (HFL). Saturating.\n" << std::endl;
        (delta_p_HFL >= 0) ? (p_HFL = this->prev_pos_app(0) + this->max_delta_pos_HFL) : (p_HFL = this->prev_pos_app(0) - this->max_delta_pos_HFL);
    }
    if (abs(delta_p_HSL) > this->max_delta_pos_HSL) {
        std::cout << "Rate limit violation: Hip Sagittal Left (HSL). Saturating.\n" << std::endl;
        (delta_p_HSL >= 0) ? (p_HSL = this->prev_pos_app(1) + this->max_delta_pos_HSL) : (p_HSL = this->prev_pos_app(1) - this->max_delta_pos_HSL);
    }
    if (abs(delta_p_KL) > this->max_delta_pos_KL) {
        std::cout << "Rate limit violation: Knee Left (KL). Saturating.\n" << std::endl;
        (delta_p_KL >= 0) ? (p_KL = this->prev_pos_app(2) + this->max_delta_pos_KL) : (p_KL = this->prev_pos_app(2) - this->max_delta_pos_KL);
    }
    if (abs(delta_p_HFR) > this->max_delta_pos_HFR) {
        std::cout << "Rate limit violation: Hip Frontal Right (HFR). Saturating.\n" << std::endl;
        (delta_p_HFR >= 0) ? (p_HFR = this->prev_pos_app(3) + this->max_delta_pos_HFR) : (p_HFR = this->prev_pos_app(3) - this->max_delta_pos_HFR);
    }
    if (abs(delta_p_HSR) > this->max_delta_pos_HSR) {
        std::cout << "Rate limit violation: Hip Sagittal Right (HSR). Saturating.\n" << std::endl;
        (delta_p_HSR >= 0) ? (p_HSR = this->prev_pos_app(4) + this->max_delta_pos_HSR) : (p_HSR = this->prev_pos_app(4) - this->max_delta_pos_HSR);
    }
    if (abs(delta_p_KR) > this->max_delta_pos_KR) {
        std::cout << "Rate limit violation: Knee Right (KR). Saturating.\n" << std::endl;
        (delta_p_KR >= 0) ? (p_KR = this->prev_pos_app(5) + this->max_delta_pos_KR) : (p_KR = this->prev_pos_app(5) - this->max_delta_pos_KR);
    }

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

    // for computing rate limited positions
    this->prev_pos_app(0) = position_ref(0);
    this->prev_pos_app(1) = position_ref(1);
    this->prev_pos_app(2) = position_ref(2);
    this->prev_pos_app(3) = position_ref(3);
    this->prev_pos_app(4) = position_ref(4);
    this->prev_pos_app(5) = position_ref(5);

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
    }
}
