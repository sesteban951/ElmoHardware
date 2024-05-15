
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

    // set up the joint gains
    JointGains gains;
    gains.Kp_HFL = config["gains"]["Kp_HFL"].as<double>();
    gains.Kp_HSL = config["gains"]["Kp_HSL"].as<double>();
    gains.Kp_KL = config["gains"]["Kp_KL"].as<double>();
    gains.Kp_HFR = config["gains"]["Kp_HFR"].as<double>();
    gains.Kp_HSR = config["gains"]["Kp_HSR"].as<double>();
    gains.Kp_KR = config["gains"]["Kp_KR"].as<double>();

    gains.Kd_HFL = config["gains"]["Kd_HFL"].as<double>();
    gains.Kd_HSL = config["gains"]["Kd_HSL"].as<double>();
    gains.Kd_KL = config["gains"]["Kd_KL"].as<double>();
    gains.Kd_HFR = config["gains"]["Kd_HFR"].as<double>();
    gains.Kd_HSR = config["gains"]["Kd_HSR"].as<double>();
    gains.Kd_KR = config["gains"]["Kd_KR"].as<double>();

    gains.Kff_HFL = config["gains"]["Kff_HFL"].as<double>();
    gains.Kff_HSL = config["gains"]["Kff_HSL"].as<double>();
    gains.Kff_KL = config["gains"]["Kff_KL"].as<double>();
    gains.Kff_HFR = config["gains"]["Kff_HFR"].as<double>();
    gains.Kff_HSR = config["gains"]["Kff_HSR"].as<double>();
    gains.Kff_KR = config["gains"]["Kff_KR"].as<double>();

    //***************************************************************
    // DO STUFF

    // initialize ELMO interface
    ELMOInterface elmo;

    // set the joint gains
    elmo.setGains(gains);

    // create two threads, one for ELMO communication and the other for ecat checking
    pthread_t thread1, thread2;
    elmo.initELMO(port, thread1, thread2);

    // get encoder data
    for (;;) {
        
        std::cout << "\n-----------------------------------------\n" << std::endl;

        // get the current encoder data
        Eigen::VectorXd data = elmo.getEncoderData();

        // get the calculated torque command
        Eigen::VectorXd joint_ref(12);
        joint_ref.setZero();
        Eigen::VectorXd tau_ff(6);
        tau_ff.setZero();
        Eigen::VectorXd tau = elmo.computeTorque(joint_ref, tau_ff);
        elmo.sendTorque(tau);

        std::cout << "Encoder Data: " << data(2) << std::endl;
        std::cout << "Torque Command: " << tau(2) << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(500)); // Sleep for nanoseconds
    }

    //***************************************************************

    std::cout << "Everything looks good." << std::endl;

    return 0;
}