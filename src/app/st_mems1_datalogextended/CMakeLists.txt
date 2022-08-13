
target_sources(app PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/nucleo_main.c
${CMAKE_CURRENT_SOURCE_DIR}/com.c
${CMAKE_CURRENT_SOURCE_DIR}/demo_serial.c
# ${CMAKE_CURRENT_SOURCE_DIR}/bat_ble.c
${CMAKE_CURRENT_SOURCE_DIR}/serial_protocol.c
${CMAKE_CURRENT_SOURCE_DIR}/sensor_commands.c
)
