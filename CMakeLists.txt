# specify the minimum version of CMake required to build the project
cmake_minimum_required(VERSION 3.10)

# project name
project(elmo)

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

# fin the packages
find_package(soem REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# executables
add_executable(test src/ElmoComm.cpp)

target_link_libraries(test soem)
target_link_libraries(test Eigen3::Eigen)
target_link_libraries(test yaml-cpp)