#include "../inc/ElmoInterface.h"

void ELMOInterface::initELMO(char* port, pthread_t thread1, pthread_t thread2) {

    // Initialize ELMO data struct
    this->data = (struct ELMOData *)malloc(sizeof(struct ELMOData));
    int32 torso0 = 0;
    int32 torso_d0 = 0;
    int32 pos0[4] = {0,0,0,0};
    int32 vel0[4] = {0,0,0,0};
    int32 torque0[4] = {0,0,0,0};
    this->data->torso=torso0;
    this->data->torso_d=torso_d0;
    memcpy(this->data->pos, pos0, sizeof(pos0));
    memcpy(this->data->pos, vel0, sizeof(vel0));
    memcpy(this->data->pos, torque0, sizeof(torque0));
    strcpy(this->data->port, port);

    printf("SOEM (Simple Open EtherCAT Master)\nSetting Up ELMO drivers...\n");

    /* Thread to catch ELMO errors and act appropriately */
    pthread_create( &thread1, NULL, &ecatcheck, (void (*)) &ctime);

    /* Thread to read ELMO data */
    pthread_create( &thread2, NULL, &ELMOcommunication, (void (*)) &this->data);

    // Wait for communication to be set up
    while(this->data->commStatus != 1) {
    switch (this->data->commStatus) {
        case -1: // If comm setup fails,
        // EXCEPT(BAD_SOCKET); -- to be implemented: exceptions
        exit(2);
    }
    };

    printf("Ready\n");
}