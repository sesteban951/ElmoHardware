
// Other imports 
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

// Custom ELMO libraries
#include "../inc/ElmoComm.h"
#include "../inc/ElmoInterface.h"

// char array to hold the ethernet port name
char port[1028];

// main ELMO control loop
int main() {

    // load config file
    std::string config_file = "../config/config.yaml";

    // instantiate yaml config object
    YAML::Node config = YAML::LoadFile(config_file);

    // load in the config parameters
    std::string eth_port = config["ethernet"].as<std::string>();

    // setup ethercat
    eth_port.copy(port, sizeof(port));

    //***************************************************************
    // DO STUFF

    // initialize ELMO
    ELMOInterface elmo;

    // create two threads, one for ecat checking, and the other for ELMO communication
    pthread_t thread1, thread2;
    elmo.initELMO(port, thread1, thread2);

    //***************************************************************

    std::cout << "Everything looks good." << std::endl;

    return 0;
}