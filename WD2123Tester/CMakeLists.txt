# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")
set(PICO_SDK_PATH "${USERHOME}/.pico-sdk/sdk/2.1.1")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Declare the project
project(WD2123Tester C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(WD2123Tester WD2123Tester.c )

pico_set_program_name(WD2123Tester "WD2123Tester")
pico_set_program_version(WD2123Tester "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(WD2123Tester 1)
pico_enable_stdio_usb(WD2123Tester 0)

# Add the standard library to the build
target_link_libraries(WD2123Tester
        pico_stdlib
        #pico_lwip
        #pico_cyw43_arch_lwip_poll
        hardware_pwm
)

# Add the standard include files to the build
target_include_directories(WD2123Tester PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(WD2123Tester)

