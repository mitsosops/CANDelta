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

# Common source files shared by both builds
set(COMMON_SOURCES
    src/main.c
    src/can/mcp2515_common.c
    src/usb/usb_comm.c
    src/protocol/commands.c
    src/led/led.c
)

# Common libraries
set(COMMON_LIBS
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_gpio
    hardware_pio
    pico_unique_id
)

# Helper function to configure a candelta executable
function(configure_candelta_target TARGET_NAME RX_SOURCE USE_IRQ)
    add_executable(${TARGET_NAME} ${COMMON_SOURCES} ${RX_SOURCE})

    # Generate PIO header for WS2812
    pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/led/ws2812.pio)

    # Include directories
    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_BINARY_DIR}
    )

    # Link libraries
    target_link_libraries(${TARGET_NAME} ${COMMON_LIBS})

    # Enable USB output, disable UART
    pico_enable_stdio_usb(${TARGET_NAME} 1)
    pico_enable_stdio_uart(${TARGET_NAME} 0)

    # USB descriptors - custom PID for unique device identification
    # VID 0x2E8A = Raspberry Pi Foundation (community use allowed)
    # PID 0xCA1D = Custom PID for CANDelta ("CA1D" ≈ "CANID")
    target_compile_definitions(${TARGET_NAME} PRIVATE
        USBD_MANUFACTURER="CANDelta"
        USBD_PRODUCT="CANDelta CAN Analyzer"
        USBD_PID=0xCA1D
    )

    # Add IRQ mode define if needed
    if(USE_IRQ)
        target_compile_definitions(${TARGET_NAME} PRIVATE MCP2515_USE_IRQ)
    endif()

    # Create UF2 file for flashing
    pico_add_extra_outputs(${TARGET_NAME})

    # Compiler warnings
    target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra)
endfunction()

# Reminder for commercial release
message(STATUS "NOTE: Before commercial release, update USBD_MANUFACTURER and USBD_PRODUCT in CMakeLists.txt")

# Build both polling and IRQ versions
message(STATUS "Building both polling and IRQ firmware variants")

configure_candelta_target(candelta_polling src/can/mcp2515_polling.c FALSE)
configure_candelta_target(candelta_irq src/can/mcp2515_irq.c TRUE)
