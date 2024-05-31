
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

// sine wave siganal
double sin_wave(double t, double A, double f) {

    double w, x;
    w = 2 * M_PI * f;

    x = A * sin(w * t);

    return x;
}

// derivative of sine wave dt
double sin_wave_dt(double t, double A, double f) {

    double w, x;
    w = 2 * M_PI * f;

    x = A * w * cos(w * t);

    return x;
}

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
    double time = 0.0;

    double sine_sig, sine_sig_dt;
    double A_ff, f_ff, A_ref, f_ref;
    A_ref = 1.0;
    f_ref = 0.5;
    A_ff = 130.0;
    f_ff = 0.5;

    // get encoder data
    while (time <= 10.0) {

        // std::cout << "\n-----------------------------------------\n" << std::endl;

        // get the current time in seconds
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
        time = duration.count() / 1'000'000.0; // convert to seconds

        // get the currebt ELMO status
        ELMOStatus diagnostics = elmo.getELMOStatus();

        // get the current encoder data
        JointVec data = elmo.getEncoderData();

        // specify some joint reference
        sine_sig = sin_wave(time, A_ref, f_ref);
        sine_sig_dt = sin_wave_dt(time, A_ff, f_ff);
        JointVec joint_ref;
        joint_ref.setZero();
        // joint_ref(2) = sine_sig;
        // joint_ref(8) = sine_sig_dt;

        // specify some feedforward torque
        JointTorque tau_ff;
        tau_ff.setZero();
        // tau_ff(2) = 140;

        // compute the net torque command
        JointTorque tau = elmo.computeTorque(joint_ref, tau_ff);

        // DEBUG
        tau.setZero();
        // tau(2) = 140;

        elmo.sendTorque(tau);

        // log the encoder and torque sent data
        file_data << time;
        for (int i = 0; i < data.size(); i++) {
            file_data << ", " << data(i);
        }
        for (int i = 0; i < tau.size(); i++) {
            file_data << ", " << tau(i);
        }
        file_data << std::endl;

        // log the diagnostics data
        file_diagnostics << time;
        for (int i = 0; i < diagnostics.size(); i++) {
            file_diagnostics << ", " << diagnostics(i);
        }
        file_diagnostics << std::endl;

        std::this_thread::sleep_for(std::chrono::microseconds(500)); // Sleep for micro seconds
    }

    // shutdown the ELMOs gracefully
    elmo.shutdownELMO();

    //***************************************************************

    return 0;
}