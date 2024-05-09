#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Dense>
#include "ElmoComm.h"

#define actuator_conversion_factor 2*M_PI/8192.0/91.4285714286 // 2*PI*ticks/res_of_encoder/GEAR_RATIO
#define gear_ratio 91.4285714286
#define torso_conversion_factor 2*M_PI/8192.0/3 // 2*PI*ticks/res_of_encoder/GEAR_RATIO

class ELMOInterface {
    
    public:

        // struct to hold ELMO data
        struct ELMOData *data;

        // function to initialize ELMO
        void initELMO(char* port, pthread_t thread1, pthread_t thread2);

};

#endif