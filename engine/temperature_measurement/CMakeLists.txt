cmake_minimum_required (VERSION 3.16)
set(CMAKE_CXX_STANDARD 11)

# rest of your project
add_executable(pico_lorawan_temperature_measurement
    main.cpp
)

SET(CMAKE_C_LINK_EXECUTABLE ${CMAKE_CXX_LINK_EXECUTABLE})
target_link_libraries(pico_lorawan_temperature_measurement pico_lorawan hardware_spi rapidjson hardware_sleep hardware_rosc hardware_pll hardware_adc hardware_i2c pac-adafruit-sht4x-library pac-adafruit-sensor)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_lorawan_temperature_measurement 1)
pico_enable_stdio_uart(pico_lorawan_temperature_measurement 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_lorawan_temperature_measurement)
