cmake_minimum_required(VERSION 3.28)
set(CMAKE_VERBOSE_MAKEFILE TRUE)
project(universal_led_strip_driver_board C)
set(CMAKE_C_STANDARD 11)

SET(MCU "attiny2313")     #Microcontroller model
SET(F_CPU 1000000)       # = 1MHz Frequency

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_C_COMPILER "avr-gcc")
SET(CMAKE_C_FLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU} -Os -std=c11 -Wall -Wstrict-prototypes -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
SET(CMAKE_C_LINK_FLAGS "-mmcu=${MCU}")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

add_executable(universal_led_strip_driver_board main.c
        input.h
        input.c)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avr-objcopy -j .text -j .data -O ihex ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avr-size ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} --mcu=${MCU} --format=avr)

