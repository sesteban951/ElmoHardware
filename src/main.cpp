
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
    uint8_t opmode = (uint8_t) config["OpMode"].as<int>();

    // operating frequency
    double freq = config["frequency"].as<double>();

    // max program time
    double max_time = config["max_prog_time"].as<double>();

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

    // set the joint limits
    elmo.setLimits(limits);

    // create two threads, one for ELMO communication and the other for ecat checking
    pthread_t thread1, thread2;
    elmo.initELMO(opmode, freq, port, thread1, thread2);

    // for trajectory generation
    double sine_sig, sine_sig_dt;
    double A_ref, f_ref;
    A_ref = 0.1;
    f_ref = 1.0;

    // initialize the intial time
    bool first_time = true;
    auto start = std::chrono::high_resolution_clock::now();
    auto t1 = start;
    double dt = 0.0, time = 0.0;

    // get encoder data
    while (time <= max_time) {

        // get the current time in seconds
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration_dt = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        dt = duration_dt.count() / 1'000'000.0; 
        
        if (dt >= (1.0 / freq) || first_time == true) {

            // update first time
            first_time = false;

            // update time
            t1 = t2;

            // for logging time
            auto duration_tot = std::chrono::duration_cast<std::chrono::microseconds>(t2 - start);
            time = duration_tot.count() / 1'000'000.0; 

            // get the current ELMO status
            ELMOStatus diagnostics = elmo.getELMOStatus();

            // get the current encoder data
            JointVec data = elmo.getEncoderData();

            // specify some feedforward torque
            JointCommand pos_ref;

            // DEBUG
            sine_sig = sin_wave(time, A_ref, f_ref);
            
            pos_ref(0) = 0.0;  // Hip Frontal Left (HFL)
            // pos_ref(0) = sine_sig;  // Hip Frontal Left (HFL)   

            pos_ref(1) = 0.0;  // Hip Sagittal Left (HSL)
            // pos_ref(1) = sine_sig;  // Hip Sagittal Left (HSL)

            pos_ref(2) = 0.0;  // Knee Left (KL)
            // pos_ref(2) = sine_sig;  // Knee Left (KL)

            pos_ref(3) = 0.0;  // Hip Frontal Right (HFR)
            // pos_ref(3) = sine_sig;  // Hip Frontal Right (HFR)
            
            pos_ref(4) = 0.0;  // Hip Sagittal Right (HSR)
            // pos_ref(4) = sine_sig;  // Hip Sagittal Right (HSR)

            // pos_ref(5) = 0.1;  // Knee Right (KR)
            pos_ref(5) = sine_sig;  // Knee Right (KR)

            // send the torque command to the ELMO
            elmo.sendPosition(pos_ref);

            // log the time data
            file_time << time << std::endl;

            // log the encoder and torque sent data
            file_data << data(0);
            for (int i = 1; i < data.size(); i++) {
                file_data << ", " << data(i);
            }
            file_data << std::endl;

            // log the reference and feedforward torque data
            file_commands << pos_ref(0);
            for (int i = 1; i < pos_ref.size(); i++) {
                file_commands << ", " << pos_ref(i);
            }
            file_commands << std::endl;

            // log the diagnostics data
            file_diagnostics << diagnostics(0);
            for (int i = 1; i < diagnostics.size(); i++) {
                file_diagnostics << ", " << diagnostics(i);
            }
            file_diagnostics << std::endl;
        }
    }

    // shutdown the ELMOs gracefully
    elmo.shutdownELMO();

    return 0;
}
