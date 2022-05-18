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

extern "C"
{
#include "pico/lorawan.h"
}

#include "Adafruit_SHT4x.h"
#include "tusb.h"

// edit with LoRaWAN Node Region and OTAA settings
#include "config.h"
#define I2C_PORT i2c0
// pin configuration for SX1276 radio module
const struct lorawan_sx1276_settings sx1276_settings = {
    .spi = {
        .inst = PICO_DEFAULT_SPI_INSTANCE,
        .mosi = PICO_DEFAULT_SPI_TX_PIN,
        .miso = PICO_DEFAULT_SPI_RX_PIN,
        .sck = PICO_DEFAULT_SPI_SCK_PIN,
        .nss = 8},
    .reset = 9,
    .dio0 = 7,
    .dio1 = 10};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui = LORAWAN_DEVICE_EUI,
    .app_eui = LORAWAN_APP_EUI,
    .app_key = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK};

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;
static int addr = 0x44;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// functions used in main
void internal_temperature_init();
float internal_temperature_get();

// void temp_sensor_init(void) {
//     sleep_ms(1000);
//     uint8_t reg = 0x00;
//     uint8_t chipID[1];
//     i2c_write_blocking(I2C_PORT, addr, $reg, 1, true);
//     i2c_read_blocking(I2C_PORT, addr, chipID, 1, true);

//     if (chipID[0] != 0xA0) {
//         while(1) {
//             printf('ChipID not connected');
//             sleep_ms(5000);
//         }
//     }
// }

void init_sht4x_sensor()
{
    printf("Adafruit SHT4x test");
    if (!sht4.begin())
    {
        printf("Couldn't find SHT4x");
        while (1)
            delay(1);
    }
    printf("Found SHT4x sensor");

    // You can have 3 different precisions, higher precision takes longer
    sht4.setPrecision(SHT4X_MED_PRECISION);
    switch (sht4.getPrecision())
    {
    case SHT4X_HIGH_PRECISION:
        printf("High precision");
        break;
    case SHT4X_MED_PRECISION:
        printf("Med precision");
        break;
    case SHT4X_LOW_PRECISION:
        printf("Low precision");
        break;
    }

    // You can have 6 different heater settings
    // higher heat and longer times uses more power
    // and reads will take longer too!
    sht4.setHeater(SHT4X_NO_HEATER);
    switch (sht4.getHeater())
    {
    case SHT4X_NO_HEATER:
        printf("No heater");
        break;
    case SHT4X_HIGH_HEATER_1S:
        printf("High heat for 1 second");
        break;
    case SHT4X_HIGH_HEATER_100MS:
        printf("High heat for 0.1 second");
        break;
    case SHT4X_MED_HEATER_1S:
        printf("Medium heat for 1 second");
        break;
    case SHT4X_MED_HEATER_100MS:
        printf("Medium heat for 0.1 second");
        break;
    case SHT4X_LOW_HEATER_1S:
        printf("Low heat for 1 second");
        break;
    case SHT4X_LOW_HEATER_100MS:
        printf("Low heat for 0.1 second");
        break;
    }
}

int main(void)
{
    char devEui[17];

    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    while (!tud_cdc_connected())
    {
        tight_loop_contents();
    }

    printf("Pico LoRaWAN - OTAA - Temperature + LED\n\n");

    init_sht4x_sensor();
    sensors_event_t humidity, temp;
    // initialize the LED pin and internal temperature ADC
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    //internal_temperature_init();

    // uncomment next line to enable debug
    lorawan_debug(true);

    // initialize the LoRaWAN stack
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_otaa(&sx1276_settings, LORAWAN_REGION, &otaa_settings) < 0)
    {
        printf("failed!!!\n");
        while (1)
        {
            tight_loop_contents();
        }
    }
    else
    {
        printf("success!\n");
    }

    // Start the join process and wait
    printf("Joining LoRaWAN network ...");
    lorawan_join();

    while (!lorawan_is_joined())
    {
        lorawan_process_timeout_ms(1000);
        printf(".");

        sht4.getEvent(&humidity, &temp);
        printf("Temperature: %f degrees C\n", temp.temperature);
        printf("Humidity: %f% rH", humidity.relative_humidity);
    }
    printf(" joined successfully!\n");

    // loop forever
    while (1)
    {
        // get the internal temperature
        int8_t adc_temperature_byte = internal_temperature_get();

        sht4.getEvent(&humidity, &temp);
        printf("Temperature: %f degrees C\n", temp.temperature);
        printf("Humidity: %f% rH", humidity.relative_humidity);
        // send the internal temperature as a (signed) byte in an unconfirmed uplink message
        if (lorawan_send_unconfirmed(&temp.temperature, sizeof(temp.temperature), 2) < 0)
        {
            printf("failed!!!\n");
        }
        else
        {
            printf("success!\n");
        }

        // wait for up to 30 seconds for an event
        if (lorawan_process_timeout_ms(30000) == 0)
        {
            // check if a downlink message was received
            receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
            if (receive_length > -1)
            {
                printf("received a %d byte message on port %d: ", receive_length, receive_port);

                for (int i = 0; i < receive_length; i++)
                {
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
