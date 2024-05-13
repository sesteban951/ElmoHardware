#include "../inc/ElmoInterface.h"


// function to intialize the ELMO motor controllers
void ELMOInterface::initELMO(char* port, pthread_t thread1, pthread_t thread2) {

    // Initialize ELMO data struct
    this->data = (struct ELMOData *)malloc(sizeof(struct ELMOData));

    // Inital values to populate the ELMO data struct
    int32 torso0 = 0;
    int32 torso_d0 = 0;
    int32 pos0[6] = {0,0,0,0,0,0};
    int32 vel0[6] = {0,0,0,0,0,0};
    int32 torque0[6] = {0,0,0,0,0,0};

    // Populate the ELMO data struct with the intial values
    this->data->torso=torso0;
    this->data->torso_d=torso_d0;
    memcpy(this->data->pos, pos0, sizeof(pos0));
    memcpy(this->data->pos, vel0, sizeof(vel0));
    memcpy(this->data->pos, torque0, sizeof(torque0)); 
    strcpy(this->data->port, port);                    // attach the ethernet port

    printf("SOEM (Simple Open EtherCAT Master)\nSetting Up ELMO drivers...\n");

    /* Thread to catch ELMO errors and act appropriately */
    pthread_create( &thread1, NULL, &ecatcheck, (void (*)) &ctime);

    /* Thread to communicate with ELMO. Send and receive data*/
    pthread_create( &thread2, NULL, &ELMOcommunication, (void (*)) &this->data);

    // Wait for communication to be set up
    while(this->data->commStatus != 1) {
    switch (this->data->commStatus) {
        case -1: // If comm setup fails,
        std::cout << "Communication setup failed. Exiting..." << std::endl;
        exit(2);
    }
    };

    printf("Ready.\n");
}

// function to get teh raw encoder data
Eigen::VectorXd ELMOInterface::getElmoData() {
    
    // declare variables for the incoming data
    int32 pos[6];
    int32 vel[6];

    // copy the incoming data to the declared variables
    memcpy(pos, this->data->pos, sizeof(pos));
    memcpy(vel, this->data->vel, sizeof(vel));

    Eigen::VectorXd data(12);
    data.setZero();
    data << pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], vel[0], vel[1], vel[2], vel[3], vel[4], vel[5];
    
    return data;
}