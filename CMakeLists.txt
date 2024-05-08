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

# add the ELMO library
add_library(ELMOCOMM SHARED src/ElmoComm.cpp)
target_link_libraries(ELMOCOMM PUBLIC soem)

# executables
add_executable(s src/main.cpp)
target_link_libraries(s PUBLIC ELMOCOMM)
target_link_libraries(s PUBLIC Eigen3::Eigen)
target_link_libraries(s PUBLIC yaml-cpp)
