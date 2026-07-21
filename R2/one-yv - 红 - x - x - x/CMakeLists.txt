cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME R2_armx)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add user sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    user/l1_laser.c
    user/mytask.c
    user/arm.c
    user/CAN_receive.c
    user/contorl_two.c
    user/control_Cylinder.c
    user/ecpid.c
    user/pid.c
    user/fifo.c
    user/ros_uart.c
    user/modbus.c
    user/hi14_driver.c
    user/R2006.c
    unitree_actuator_sdk\ -\ keilc/user/unitree.c
    unitree_actuator_sdk\ -\ keilc/gom_protocol.c
    unitree_actuator_sdk\ -\ keilc/crc_ccitt.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    user
    unitree_actuator_sdk\ -\ keilc
    unitree_actuator_sdk\ -\ keilc/user
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
