
// standard imports
#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

// Other imports 
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

// Custom ELMO libraries
#include "../inc/ElmoComm.hpp"
#include "../inc/ElmoInterface.hpp"

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

    // for logging purposes
    std::string log_file_data = "../data/data.csv";
    std::string log_file_diagnostics = "../data/diagnostics.csv";
    std::ofstream file_data;
    std::ofstream file_diagnostics;
    file_data.open(log_file_data);
    file_diagnostics.open(log_file_diagnostics);

    //***************************************************************
    // DO STUFF

    // initialize ELMO interface
    ELMOInterface elmo;

    // set the joint gains
    elmo.setGains(gains);

    // create two threads, one for ELMO communication and the other for ecat checking
    pthread_t thread1, thread2;
    elmo.initELMO(port, thread1, thread2);

    // initialize the intial time
    auto start = std::chrono::high_resolution_clock::now();

    // get encoder data
    for (;;){

        // std::cout << "\n-----------------------------------------\n" << std::endl;

        // get the current time in seconds
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
        double time = duration.count() / 1'000'000.0; // convert to seconds

        // get the currebt ELMO status
        ELMOStatus status = elmo.getELMOStatus();

        // get the current encoder data
        JointVec data = elmo.getEncoderData();

        // specify some joint reference
        JointVec joint_ref;
        joint_ref.setZero();

        // specify some feedforward torque
        JointTorque tau_ff;
        tau_ff.setZero();

        // compute the net torque command
        JointTorque tau = elmo.computeTorque(joint_ref, tau_ff);
        elmo.sendTorque(tau);

        // log the encoder data
        file_data << time << ", "
                  << data(0) << ", " << data(1) << ", " << data(2) << ", " << data(3) << ", " << data(4) << ", " << data(5) << ", " 
                  << data(6) << ", " << data(7) << ", " << data(8) << ", " << data(9) << ", " << data(10) << ", " << data(11) << ", "
                  << tau(0) << ", " << tau(1) << ", " << tau(2) << ", " << tau(3) << ", " << tau(4) << ", " << tau(5) << std::endl;

        file_diagnostics << time << ", "
                         << status(0) << ", " << status(1) << ", " << status(2) << ", " << status(3) << ", " << status(4) << ", " << status(5) << std::endl;

        std::this_thread::sleep_for(std::chrono::microseconds(500)); // Sleep for micro seconds
    }

    //***************************************************************

    return 0;
}