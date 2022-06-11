# ST Microelectronics LIS2DW12 3-axis accelerometer driver
#
# Copyright (c) 2019 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
# zephyr_library()

# target_sources(lis2dw12xd.c)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lis2dw12xd.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lis2dw12_i2c.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lis2dw12_reg.c)

zephyr_library_sources_ifdef(CONFIG_LIS2DW12XD_TRIGGER    lis2dw12xd_trigger.c)

# zephyr_library_include_directories(../stmemsc)
