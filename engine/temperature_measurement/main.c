/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * This example uses OTAA to join the LoRaWAN network and then sends the
 * internal temperature sensors value up as an uplink message periodically
 * and the first byte of any uplink messages received controls the boards
 * built-in LED.
 */

#include <stdio.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "tusb.h"

// edit with LoRaWAN Node Region and OTAA settings
#include "config.h"

// pin configuration for SX1276 radio module
const struct lorawan_sx1276_settings sx1276_settings = {
    .spi = {
        .inst = PICO_DEFAULT_SPI_INSTANCE,
        .mosi = PICO_DEFAULT_SPI_TX_PIN,
        .miso = PICO_DEFAULT_SPI_RX_PIN,
        .sck  = PICO_DEFAULT_SPI_SCK_PIN,
        .nss  = 8
    },
    .reset = 9,
    .dio0  = 7,
    .dio1  = 10
};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui   = LORAWAN_DEVICE_EUI,
    .app_eui      = LORAWAN_APP_EUI,
    .app_key      = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

// functions used in main
void internal_temperature_init();
float internal_temperature_get();

int main( void )
{
    char devEui[17];

    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    printf("Pico LoRaWAN - OTAA - Temperature + LED\n\n");
    printf("Dev EUI = %s\n", lorawan_default_dev_eui(devEui));

    ////////////////////////////////////////////////
    #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    return 0;
#endif
/////////////////////////////

    // initialize the LED pin and internal temperature ADC
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    internal_temperature_init();

    // uncomment next line to enable debug
    lorawan_debug(true);

    // initialize the LoRaWAN stack
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_otaa(&sx1276_settings, LORAWAN_REGION, &otaa_settings) < 0) {
        printf("failed!!!\n");
        while (1) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    // Start the join process and wait
    printf("Joining LoRaWAN network ...");
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process_timeout_ms(1000);
        printf(".");
    }
    printf(" joined successfully!\n");

    // loop forever
    while (1) {
        // get the internal temperature
        int8_t adc_temperature_byte = internal_temperature_get();

        // send the internal temperature as a (signed) byte in an unconfirmed uplink message
        printf("sending internal temperature: %d °C (0x%02x)... ", adc_temperature_byte, adc_temperature_byte);
        if (lorawan_send_unconfirmed(&adc_temperature_byte, sizeof(adc_temperature_byte), 2) < 0) {
            printf("failed!!!\n");
        } else {
            printf("success!\n");
        }

        // wait for up to 30 seconds for an event
        if (lorawan_process_timeout_ms(30000) == 0) {
            // check if a downlink message was received
            receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
            if (receive_length > -1) {
                printf("received a %d byte message on port %d: ", receive_length, receive_port);

                for (int i = 0; i < receive_length; i++) {
                    printf("%02x", receive_buffer[i]);
                }
                printf("\n");

                // the first byte of the received message controls the on board LED
                gpio_put(PICO_DEFAULT_LED_PIN, receive_buffer[0]);
            }
        }
    }

    return 0;
}

void internal_temperature_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

float internal_temperature_get()
{
    const float v_ref = 3.3;

    // select and read the ADC
    adc_select_input(4);
    uint16_t adc_raw = adc_read();

    // convert the raw ADC value to a voltage
    float adc_voltage = adc_raw * v_ref / 4095.0f;

    // convert voltage to temperature, using the formula from
    // section 4.9.4 in the RP2040 datasheet
    //   https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
    float adc_temperature = 27.0 - ((adc_voltage - 0.706) / 0.001721);

    return adc_temperature;
}
