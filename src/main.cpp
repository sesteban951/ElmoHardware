
// Other imports 
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

// Custom ELMO libraries
#include "../inc/ElmoComm.h"

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

    // initialize ELMO
    initELMO(port);

    std::cout << "Everything looks good." << std::endl;

    return 0;
}