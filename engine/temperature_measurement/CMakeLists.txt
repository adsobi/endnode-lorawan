cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(pico_lorawan_temperature_measurement
    main.c
)

target_link_libraries(pico_lorawan_temperature_measurement pico_lorawan hardware_adc hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_lorawan_temperature_measurement 1)
pico_enable_stdio_uart(pico_lorawan_temperature_measurement 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_lorawan_temperature_measurement)
