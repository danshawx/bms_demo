# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(BOARD bms_demo_nrf52840)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bms_demo)

target_sources(app PRIVATE src/main.c)

# zephyr_library_include_directories(.)
zephyr_include_directories(
  src/include/drivers
#   configuration/${BOARD}
)

# Application sources
add_subdirectory(src/app)
add_subdirectory(src/drivers)



include_directories(src/app)
include_directories(src/drivers)
include_directories(src/app/st_mems1_datalogextended)

# Drivers directories
# add_subdirectory_ifdef(CONFIG_WATCHDOG_APPLICATION src/drivers/watchdog)
# add_subdirectory_ifdef(CONFIG_N58 src/drivers/nw_n58)