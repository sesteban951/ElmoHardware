# specify the minimum version of CMake required to build the project
cmake_minimum_required(VERSION 3.10)

# project name
project(elmo_test)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# define package path
set(soem_DIR /home/sergio/repos/SOEM_install/share/soem/cmake)
set(EIGEN_DIR /home/sergio/repos/eigen-3.4.0)
set(YAML_DIR /home/sergio/repos/yaml-cpp_install)

# include the directories
include_directories(soem_DIR)
include_directories(EIGEN_DIR)
include_directories(YAML_DIR)

# find the packages
find_package(soem REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# add  libraries
add_library(ELMOCOMM src/ElmoComm.cpp inc/ElmoComm.hpp)
target_link_libraries(ELMOCOMM PUBLIC soem)
add_library(ELMOINTERFACE src/ElmoInterface.cpp inc/ElmoInterface.hpp)
target_link_libraries(ELMOINTERFACE PUBLIC ELMOCOMM Eigen3::Eigen)

# main executable
add_executable(s src/main.cpp)               
target_link_libraries(s PUBLIC 
                      ELMOCOMM
                      ELMOINTERFACE
                      Eigen3::Eigen
                      yaml-cpp)

# SOEM simple test executable
add_executable(simple_test src/simple_test.c)
target_link_libraries(simple_test soem)

# mgruhler torque example executable
add_executable(torque_examples_2drivers src/torque_examples_2drivers.cpp)
target_link_libraries(torque_examples_2drivers soem)