
// standard imports
#include <thread>
#include <chrono>

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

    // initialize ELMO interface
    ELMOInterface elmo;

    // create two threads, one for ELMO communication and the other for ecat checking
    pthread_t thread1, thread2;
    elmo.initELMO(port, thread1, thread2);

    // get encoder data
    // for (int i = 0; i < 100; ++i) { // Loop 50 times
    for (;;) { // Loop 50 times

        Eigen::VectorXd data = elmo.getEncoderData();
        std::cout << "\n-----------------------------------------\n" << std::endl;
        std::cout << "Data: " << data << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Sleep for 250 milliseconds
    }

    //***************************************************************

    std::cout << "Everything looks good." << std::endl;

    return 0;
}