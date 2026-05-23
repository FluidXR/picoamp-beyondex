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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Use our vendored, patched TinyUSB instead of the SDK's bundled copy.
#
# Background: SDK 2.1.1 ships TinyUSB 0.18.0, which breaks USB Audio on Windows
# (see commit 4b6f971). We pinned to 0.17.0. But 0.17.0 has a separate RP2040
# DCD bug — _hw_endpoint_alloc is a bump allocator that doesn't reclaim DPRAM
# per-EP, only when ALL non-control EPs are closed. With our always-open WinUSB
# vendor interface plus iso audio EPs that cycle on every audio session start/stop,
# DPRAM exhausts after a few cycles and hard_assert panics the firmware. Upstream
# fix is hathach/tinyusb PR #1802; we backport it into our vendored 0.17.0 here.
#
# PICO_TINYUSB_PATH is the documented override point in pico-sdk's
# src/rp2_common/tinyusb/CMakeLists.txt — set before pico_sdk_init() so the SDK
# uses our tree, not its own.
set(PICO_TINYUSB_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/tinyusb" CACHE PATH "TinyUSB source tree")

set(BEYONDEX_REQUIRED_TUSB_VERSION "0.17.0")
if(NOT EXISTS "${PICO_TINYUSB_PATH}/src/tusb_option.h")
    message(FATAL_ERROR "Vendored TinyUSB missing at ${PICO_TINYUSB_PATH}")
endif()
file(STRINGS "${PICO_TINYUSB_PATH}/src/tusb_option.h" _tusb_ver_lines
     REGEX "^#define TUSB_VERSION_(MAJOR|MINOR|REVISION)")
foreach(_line ${_tusb_ver_lines})
    if(_line MATCHES "TUSB_VERSION_MAJOR[ \t]+([0-9]+)")
        set(_tusb_major ${CMAKE_MATCH_1})
    elseif(_line MATCHES "TUSB_VERSION_MINOR[ \t]+([0-9]+)")
        set(_tusb_minor ${CMAKE_MATCH_1})
    elseif(_line MATCHES "TUSB_VERSION_REVISION[ \t]+([0-9]+)")
        set(_tusb_rev ${CMAKE_MATCH_1})
    endif()
endforeach()
set(_tusb_ver "${_tusb_major}.${_tusb_minor}.${_tusb_rev}")
if(NOT _tusb_ver VERSION_EQUAL ${BEYONDEX_REQUIRED_TUSB_VERSION})
    message(FATAL_ERROR
        "Vendored TinyUSB at ${PICO_TINYUSB_PATH} is ${_tusb_ver}, "
        "expected ${BEYONDEX_REQUIRED_TUSB_VERSION}.")
endif()

# Also verify the PR #1802 patch is present in the vendored copy — without it
# audio start/stop cycling will wedge the firmware after a few cycles.
file(STRINGS "${PICO_TINYUSB_PATH}/src/portable/raspberrypi/rp2040/dcd_rp2040.c"
     _patch_marker REGEX "PR #1802 backport" LIMIT_COUNT 1)
if(NOT _patch_marker)
    message(FATAL_ERROR
        "Vendored TinyUSB is missing the PR #1802 backport "
        "(dcd_rp2040.c marker not found). The build would compile but produce "
        "firmware that wedges on Windows audio session churn.")
endif()
message(STATUS "TinyUSB version: ${_tusb_ver} (vendored, PR #1802 patched)")

project(beyondex_usb_speaker C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(beyondex_usb_speaker
        main.c
        usb_descriptors.c
        beyondex_diag.c
        )

pico_set_program_name(beyondex_usb_speaker "beyondex_usb_speaker")
pico_set_program_version(beyondex_usb_speaker "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(beyondex_usb_speaker 0)
pico_enable_stdio_usb(beyondex_usb_speaker 0)

# Add the standard library to the build
target_link_libraries(beyondex_usb_speaker
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(beyondex_usb_speaker PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

add_subdirectory(${CMAKE_SOURCE_DIR}/lib/pico-i2s-pio)

# Add any user requested libraries
target_link_libraries(beyondex_usb_speaker
        hardware_clocks
        hardware_dma
        hardware_flash
        hardware_sync
        tinyusb_device
        tinyusb_board
        pico-i2s-pio
        )

# Hardware workarounds
pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(beyondex_usb_speaker PRIVATE
        # Startup delay to avoid issue with QSPI flash
        PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64

        # Performance, avoid calls to ____wrap___aeabi_lmul_veneer when doing 64bit multiplies
        PICO_INT64_OPS_IN_RAM=1
)
target_compile_definitions(slower_boot2 PRIVATE
        #MG - 7/30/2024
        #System clock set to 295MHz by i2s lib: PICO_FLASH_SPI_CLKDIV=2 by default -> 147.5MHz clk is above 133MHz flash memory spec
        #Drop QSPI clock to avoid errors when loading program from flash (295MHz / 4 = 73.75MHz)
        # PICO_FLASH_SPI_CLKDIV=2 can still be used if voltage is raised to >=1.25V
        PICO_FLASH_SPI_CLKDIV=4
)
pico_set_boot_stage2(beyondex_usb_speaker slower_boot2)

pico_add_extra_outputs(beyondex_usb_speaker)

