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
set(CMAKE_PROJECT_NAME RM_ITL)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")


# 降低所有构建类型的优化等级到最低
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-O0 -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O0 -g3")

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

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    #bsp
    ${CMAKE_SOURCE_DIR}/bsp/led/bsp_led.c
    ${CMAKE_SOURCE_DIR}/bsp/buzzer/bsp_buzzer.c
    ${CMAKE_SOURCE_DIR}/bsp/can/bsp_can.c
    ${CMAKE_SOURCE_DIR}/bsp/dwt/bsp_dwt.c 
    ${CMAKE_SOURCE_DIR}/bsp/usart/bsp_usart.c
    ${CMAKE_SOURCE_DIR}/bsp/usb/bsp_usb.c   
    ${CMAKE_SOURCE_DIR}/bsp/pwm/bsp_pwm.c   
    #modules
    ${CMAKE_SOURCE_DIR}/modules/Motor/DJI_Motor/DJI_Motor.c
    ${CMAKE_SOURCE_DIR}/modules/Motor/DM_Motor/DM_Motor.c
    ${CMAKE_SOURCE_DIR}/modules/Motor/RS_Motor/RS_Motor.c
    ${CMAKE_SOURCE_DIR}/modules/Motor/GQ_Motor/GQ_Motor.c
    ${CMAKE_SOURCE_DIR}/modules/remote_control/remote_control.c
    ${CMAKE_SOURCE_DIR}/modules/Pid/Pid.c
    ${CMAKE_SOURCE_DIR}/modules/PID_Test/PID_Test.c
    ${CMAKE_SOURCE_DIR}/modules/Lowpass/Lowpass.c
    ${CMAKE_SOURCE_DIR}/modules/BMI088/BMI088driver.c
    ${CMAKE_SOURCE_DIR}/modules/BMI088/BMI088Middleware.c
    #app
    ${CMAKE_SOURCE_DIR}/app/Init/Init.c
    ${CMAKE_SOURCE_DIR}/app/chassis/mecanum_chassis.c
    ${CMAKE_SOURCE_DIR}/app/chassis/chassis_behaviour.c
    ${CMAKE_SOURCE_DIR}/app/gimbal/gimbal.c  
    ${CMAKE_SOURCE_DIR}/app/ins/ins.c 
    ${CMAKE_SOURCE_DIR}/app/detect/detect.c  
    ${CMAKE_SOURCE_DIR}/app/shoot/shoot.c
    ${CMAKE_SOURCE_DIR}/app/referee/referee.c
    ${CMAKE_SOURCE_DIR}/app/ui/ui_default.c
    ${CMAKE_SOURCE_DIR}/app/ui/ui_interface.c
    ${CMAKE_SOURCE_DIR}/app/ui/ui_static_ui.c    
    ${CMAKE_SOURCE_DIR}/app/ui/ui_mode_ui.c     
    #usb
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/App/usb_device.c
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/App/usbd_cdc_if.c
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/App/usbd_desc.c
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/Target/usbd_conf.c
    #device
    ${CMAKE_SOURCE_DIR}/device/Algorithm/controller.c
    ${CMAKE_SOURCE_DIR}/device/Algorithm/kalman_filter.c
    ${CMAKE_SOURCE_DIR}/device/Algorithm/QuaternionEKF.c
    ${CMAKE_SOURCE_DIR}/device/Algorithm/user_lib.c
    ${CMAKE_SOURCE_DIR}/device/CRC/CRC.c
    ${CMAKE_SOURCE_DIR}/device/control_power/control_power.c
    ${CMAKE_SOURCE_DIR}/device/Circular_buffer/Circular_buffer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    #bsp
    ${CMAKE_SOURCE_DIR}/bsp/led
    ${CMAKE_SOURCE_DIR}/bsp/buzzer
    ${CMAKE_SOURCE_DIR}/bsp/can
    ${CMAKE_SOURCE_DIR}/bsp/dwt
    ${CMAKE_SOURCE_DIR}/bsp/usart
    ${CMAKE_SOURCE_DIR}/bsp/usb
    ${CMAKE_SOURCE_DIR}/bsp/pwm
    #modules
    ${CMAKE_SOURCE_DIR}/modules/Motor
    ${CMAKE_SOURCE_DIR}/modules/Motor/DJI_Motor
    ${CMAKE_SOURCE_DIR}/modules/Motor/DM_Motor
    ${CMAKE_SOURCE_DIR}/modules/Motor/RS_Motor
    ${CMAKE_SOURCE_DIR}/modules/Motor/GQ_Motor
    ${CMAKE_SOURCE_DIR}/modules/remote_control
    ${CMAKE_SOURCE_DIR}/modules/Pid
    ${CMAKE_SOURCE_DIR}/modules/PID_Test
    ${CMAKE_SOURCE_DIR}/modules/Lowpass
    ${CMAKE_SOURCE_DIR}/modules/BMI088
    #app
    ${CMAKE_SOURCE_DIR}/app/Init
    ${CMAKE_SOURCE_DIR}/app/chassis
    ${CMAKE_SOURCE_DIR}/app/gimbal
    ${CMAKE_SOURCE_DIR}/app/ins
    ${CMAKE_SOURCE_DIR}/app/detect
    ${CMAKE_SOURCE_DIR}/app/shoot
    ${CMAKE_SOURCE_DIR}/app/referee
    ${CMAKE_SOURCE_DIR}/app/ui
    #usb
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/App
    ${CMAKE_SOURCE_DIR}/USB_DEVICE/Target
    #device
    ${CMAKE_SOURCE_DIR}/device/Algorithm
    ${CMAKE_SOURCE_DIR}/device/Mahony
    ${CMAKE_SOURCE_DIR}/device/CRC
    ${CMAKE_SOURCE_DIR}/device/control_power
    ${CMAKE_SOURCE_DIR}/device/Circular_buffer
    #DSP
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
    # Add user defined libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/libCMSISDSP.a
)
