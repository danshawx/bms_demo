
target_sources(app PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/comm.c
${CMAKE_CURRENT_SOURCE_DIR}/bat_led.c
# ${CMAKE_CURRENT_SOURCE_DIR}/bat_ble_uart.c
# ${CMAKE_CURRENT_SOURCE_DIR}/bat_ble.c
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/st_mems1_datalogextended)

zephyr_library_sources_ifdef(CONFIG_BQ769x2    monitor.c)
zephyr_library_sources_ifdef(CONFIG_LIS2DW12XD    gsensor.c)