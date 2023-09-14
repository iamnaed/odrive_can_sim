cmake_minimum_required(VERSION 3.8)
project(odrive_can_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(
    odrive_can_sim_lib
    STATIC
    src/odrive_can_sim.cpp
)

target_compile_features(
    odrive_can_sim_lib 
    PUBLIC 
    cxx_std_20
)

target_include_directories(
    odrive_can_sim_lib 
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(
    odrive_can_sim_main 
    src/odrive_main.cpp
)

target_link_libraries(
    odrive_can_sim_main
    PRIVATE
    odrive_can_sim_lib
)