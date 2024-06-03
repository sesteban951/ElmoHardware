
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

// sine wave signal
double sin_wave(double t, double A, double f) {

    double w, x;
    w = 2 * M_PI * f;

    x = A * sin(w * t);

    return x;
}

// derivative of sine wave 
double sin_wave_dt(double t, double A, double f) {

    double w, dx;
    w = 2 * M_PI * f;

    dx = A * w * cos(w * t);

    return dx;
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

    // operation mode (TODO: fix bug, need to start at 8 then go to 10)
    uint8 opmode = (uint8) config["OpMode"].as<int>();

    // max program time
    double max_time = config["max_prog_time"].as<double>();

    // set up the joint gains
    JointGains gains;
    gains.Kp_HFL = config["gains"]["HFL"]["Kp"].as<double>();  // Hip Frontal Left (HFL)
    gains.Kd_HFL = config["gains"]["HFL"]["Kd"].as<double>();
    gains.Kff_HFL = config["gains"]["HFL"]["Kff"].as<double>();
    
    gains.Kp_HSL = config["gains"]["HSL"]["Kp"].as<double>();  // Hip Sagittal Left (HSL)
    gains.Kd_HSL = config["gains"]["HSL"]["Kd"].as<double>();
    gains.Kff_HSL = config["gains"]["HSL"]["Kff"].as<double>();
    
    gains.Kp_KL = config["gains"]["KL"]["Kp"].as<double>();    // Knee Left (KL)
    gains.Kd_KL = config["gains"]["KL"]["Kd"].as<double>();
    gains.Kff_KL = config["gains"]["KL"]["Kff"].as<double>();
    
    gains.Kp_HFR = config["gains"]["HFR"]["Kp"].as<double>();  // Hip Frontal Right (HFR)
    gains.Kd_HFR = config["gains"]["HFR"]["Kd"].as<double>();
    gains.Kff_HFR = config["gains"]["HFR"]["Kff"].as<double>();
    
    gains.Kp_HSR = config["gains"]["HSR"]["Kp"].as<double>();  // Hip Sagittal Right (HSR)
    gains.Kd_HSR = config["gains"]["HSR"]["Kd"].as<double>();
    gains.Kff_HSR = config["gains"]["HSR"]["Kff"].as<double>();
    
    gains.Kp_KR = config["gains"]["KR"]["Kp"].as<double>();    // Knee Right (KR)
    gains.Kd_KR = config["gains"]["KR"]["Kd"].as<double>();
    gains.Kff_KR = config["gains"]["KR"]["Kff"].as<double>();

    // set up joint limits
    JointLimits limits;
    limits.q_min_HFL = config["limits"]["HFL"]["q_min"].as<double>(); // Hip Frontal Left (HFL)
    limits.q_max_HFL = config["limits"]["HFL"]["q_max"].as<double>(); 
    limits.qd_min_HFL = config["limits"]["HFL"]["qd_min"].as<double>();
    limits.qd_max_HFL = config["limits"]["HFL"]["qd_max"].as<double>();

    limits.q_min_HSL = config["limits"]["HSL"]["q_min"].as<double>(); // Hip Sagittal Left (HSL)
    limits.q_max_HSL = config["limits"]["HSL"]["q_max"].as<double>();
    limits.qd_min_HSL = config["limits"]["HSL"]["qd_min"].as<double>();
    limits.qd_max_HSL = config["limits"]["HSL"]["qd_max"].as<double>();

    limits.q_min_KL = config["limits"]["KL"]["q_min"].as<double>();   // Knee Left (KL)
    limits.q_max_KL = config["limits"]["KL"]["q_max"].as<double>();
    limits.qd_min_KL = config["limits"]["KL"]["qd_min"].as<double>();
    limits.qd_max_KL = config["limits"]["KL"]["qd_max"].as<double>();

    limits.q_min_HFR = config["limits"]["HFR"]["q_min"].as<double>();  // Hip Frontal Right (HFR)
    limits.q_max_HFR = config["limits"]["HFR"]["q_max"].as<double>();
    limits.qd_min_HFR = config["limits"]["HFR"]["qd_min"].as<double>();
    limits.qd_max_HFR = config["limits"]["HFR"]["qd_max"].as<double>();

    limits.q_min_HSR = config["limits"]["HSR"]["q_min"].as<double>();  // Hip Sagittal Right (HSR)
    limits.q_max_HSR = config["limits"]["HSR"]["q_max"].as<double>();
    limits.qd_min_HSR = config["limits"]["HSR"]["qd_min"].as<double>();
    limits.qd_max_HSR = config["limits"]["HSR"]["qd_max"].as<double>();

    limits.q_min_KR = config["limits"]["KR"]["q_min"].as<double>();    // Knee Right (KR)
    limits.q_max_KR = config["limits"]["KR"]["q_max"].as<double>();
    limits.qd_min_KR = config["limits"]["KR"]["qd_min"].as<double>();
    limits.qd_max_KR = config["limits"]["KR"]["qd_max"].as<double>();

    // for logging purposes
    std::string log_file_time = "../data/time.csv";
    std::string log_file_data = "../data/data.csv";
    std::string log_file_commands = "../data/commands.csv";
    std::string log_file_diagnostics = "../data/diagnostics.csv";
    std::ofstream file_time;
    std::ofstream file_data;
    std::ofstream file_commands;
    std::ofstream file_diagnostics;
    file_time.open(log_file_time);
    file_data.open(log_file_data);
    file_commands.open(log_file_commands);
    file_diagnostics.open(log_file_diagnostics);

    //***************************************************************
    // DO STUFF

    // initialize ELMO interface
    ELMOInterface elmo;

    // set the joint gains and limits
    elmo.setGains(gains);
    elmo.setLimits(limits);

    // create two threads, one for ELMO communication and the other for ecat checking
    pthread_t thread1, thread2;
    elmo.initELMO(opmode, port, thread1, thread2);

    // initialize the intial time
    auto start = std::chrono::high_resolution_clock::now();
    double time = 0.0;

    double sine_sig, sine_sig_dt, sin_tau;
    double A_ff, f_ff, A_ref, f_ref;
    A_ref = 0.2;
    f_ref = 0.25;
    A_ff = 0.0;
    f_ff = 0.5;

    // get encoder data
    while (time <= max_time) {

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
        sine_sig_dt = sin_wave_dt(time, A_ref, f_ref);
        JointVec joint_ref;
        joint_ref.setZero();

        // HSL
        // joint_ref(0) = -0.3;
        // joint_ref(6) = 0.0;
        joint_ref(0) = sine_sig;
        joint_ref(6) = sine_sig_dt;

        // HSL
        joint_ref(1) = 0.2;
        joint_ref(7) = 0.0;
        // joint_ref(1) = sine_sig;
        // joint_ref(7) = sine_sig_dt;

        // KL 
        joint_ref(2) = 0.0;
        joint_ref(8) = 0.0;
        // joint_ref(2) = sine_sig;
        // joint_ref(8) = sine_sig_dt;

        // HSR
        joint_ref(4) = -0.0;
        joint_ref(10) = 0.0;
        // joint_ref(4) = sine_sig;
        // joint_ref(10) = sine_sig_dt;
        
        // KR
        joint_ref(5) = 0.0;
        joint_ref(11) = 0.0;
        // joint_ref(5) = sine_sig;
        // joint_ref(11) = sine_sig_dt;

        // specify some feedforward torque
        sin_tau = sin_wave(time, A_ff, f_ff);
        JointTorque tau_ff;
        tau_ff.setZero();

        // compute the net torque command
        JointTorque tau = elmo.computeTorque(joint_ref, tau_ff);

        // DEBUG
        // tau.setZero();
        // tau(0) = 0.0;  // Hip Frontal Left (HFL)
        // tau(1) = 0.0;  // Hip Sagittal Left (HSL)
        // tau(2) = 0.0;  // Knee Left (KL)
        tau(3) = 0.0;  // Hip Frontal Right (HFR)
        // tau(4) = 0.0;  // Hip Sagittal Right (HSR)
        // tau(5) = 0.0;  // Knee Right (KR)

        // send the torque command to the ELMO
        elmo.sendTorque(tau);

        // log the time data
        file_time << time << std::endl;

        // log the encoder and torque sent data
        file_data << data(0);
        for (int i = 1; i < data.size(); i++) {
            file_data << ", " << data(i);
        }
        for (int i = 0; i < tau.size(); i++) {
            file_data << ", " << tau(i);
        }
        file_data << std::endl;

        // log the reference and feedforward torque data
        file_commands << joint_ref(0);
        for (int i = 1; i < joint_ref.size(); i++) {
            file_commands << ", " << joint_ref(i);
        }
        for (int i = 0; i < tau_ff.size(); i++) {
            file_commands << ", " << tau_ff(i);
        }
        file_commands << std::endl;

        // log the diagnostics data
        file_diagnostics << diagnostics(0);
        for (int i = 1; i < diagnostics.size(); i++) {
            file_diagnostics << ", " << diagnostics(i);
        }
        file_diagnostics << std::endl;

        std::this_thread::sleep_for(std::chrono::microseconds(250)); // Sleep for micro seconds
    }

    // shutdown the ELMOs gracefully
    elmo.shutdownELMO();

    //***************************************************************

    return 0;
}