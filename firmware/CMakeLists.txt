# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD adafruit_feather_rp2040 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Include Pico SDK
include(pico_sdk_import.cmake)

project(candelta_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable
add_executable(candelta
    src/main.c
    src/can/mcp2515.c
    src/usb/usb_comm.c
    src/protocol/commands.c
    src/led/led.c
)

# Generate PIO header for WS2812
pico_generate_pio_header(candelta ${CMAKE_CURRENT_SOURCE_DIR}/src/led/ws2812.pio)

# Include directories
target_include_directories(candelta PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Link libraries
target_link_libraries(candelta
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_gpio
    hardware_pio
    pico_unique_id
)

# Enable USB output, disable UART
pico_enable_stdio_usb(candelta 1)
pico_enable_stdio_uart(candelta 0)

# Create UF2 file for flashing
pico_add_extra_outputs(candelta)

# Compiler warnings
target_compile_options(candelta PRIVATE -Wall -Wextra)
