
target_sources(app PRIVATE
# ${CMAKE_CURRENT_SOURCE_DIR}/monitor.c
${CMAKE_CURRENT_SOURCE_DIR}/bat_led.c
# ${CMAKE_CURRENT_SOURCE_DIR}/bat_ble_uart.c
# ${CMAKE_CURRENT_SOURCE_DIR}/bat_ble.c
)

zephyr_library_sources_ifdef(CONFIG_BQ769x2    monitor.c)
zephyr_library_sources_ifdef(CONFIG_LIS2DW12XD    gsensor.c)