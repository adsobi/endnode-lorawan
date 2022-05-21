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

#include "pico/sleep.h"
#include <stdint.h>
#include <stdlib.h>
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"

extern "C"
{
    #include "pico/lorawan.h"
}
//#include "deep_sleep.h"

#include "Adafruit_SHT4x.h"
#include "tusb.h"
#include "config.h"


#define RECEIVE_DELAY 2000
#define SEND_DATA_DELAY 5000
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

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

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

static void sleep_callback(void) {
    printf("RTC woke us up\n");
    uart_default_tx_wait_blocking();
    return;
}

/*
*   Maximumum value of 'second_to_sleep_to' is 86399 what is equals 23h 59m 59s.
*/
static void rtc_sleep(uint32_t second_to_sleep_to)
{
    if (second_to_sleep_to >= 86400) second_to_sleep_to = 86399;

    datetime_t t = {
        .year  = 2022,
        .month = 06,
        .day   = 01,
        .dotw  = 3,
        .hour  = 00,
        .min   = 00,
        .sec   = 00
    };

    datetime_t t_alarm = {
            .year  = 2022,
            .month = 06,
            .day   = 01,
            .dotw  = 3,
            .hour  = int(second_to_sleep_to / 3600),
            .min   = int((second_to_sleep_to / 60) % 60),
            .sec   = int(second_to_sleep_to % 60)
    };

    rtc_init();
    rtc_set_datetime(&t);
    sleep_goto_sleep_until(&t_alarm, &sleep_callback);
}


void recover_from_sleep(uint scb_orig, uint clock0_orig, uint clock1_orig){

    //Re-enable ring Oscillator control
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

    //reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = clock0_orig;
    clocks_hw->sleep_en1 = clock1_orig;

    //reset clocks
    clocks_init();
    stdio_init_all();

    return;
}

void measure_freqs(void) {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
    printf("clk_rtc  = %dkHz\n", f_clk_rtc);

    uart_default_tx_wait_blocking();
    // Can't measure clk_ref / xosc as it is the ref
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
    printf("Dev EUI: %c", lorawan_default_dev_eui(devEui));

    init_sht4x_sensor();
    sensors_event_t humidity, temp;
    // initialize the LED pin and internal temperature ADC
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

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
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    //save values for later
    uint scb_orig = scb_hw->scr;
    uint clock0_orig = clocks_hw->sleep_en0;
    uint clock1_orig = clocks_hw->sleep_en1;

    while (!lorawan_is_joined())
    {
        lorawan_process_timeout_ms(1000);
        printf(".");

        sht4.getEvent(&humidity, &temp);
        printf("Temperature: %f degrees C\n", temp.temperature);
        printf("Humidity: %f% rH\n", humidity.relative_humidity);;

        printf("Go to sleep...\n");
        uart_default_tx_wait_blocking();

        sleep_run_from_xosc();
        rtc_sleep(11);
        measure_freqs();
        printf("\n");
        //reset processor and clocks back to defaults
        recover_from_sleep(scb_orig, clock0_orig, clock1_orig);
        //clocks should be restored
        measure_freqs();

        printf("Sleep from sleep_ms\n");
        uart_default_tx_wait_blocking();
        sleep_ms(2000);

    }
    printf(" joined successfully!\n");

    // loop forever
    while (true)
    {
        sht4.getEvent(&humidity, &temp);
        printf("Temperature: %f degrees C\n", temp.temperature);
        printf("Humidity: %f% rH", humidity.relative_humidity);

        // while(!best_effort_wfe_or_timeout(SEND_DATA_DELAY))
        // {
        //     //pass
        // }

        // send the internal temperature as a (signed) byte in an unconfirmed uplink message
        if (lorawan_send_unconfirmed(&temp.temperature, sizeof(temp.temperature), 2) < 0)
        {
            printf("failed!!!\n");
        }
        else
        {
            printf("success!\n");
        }

        // wait for up to 2 seconds for an event
        if (lorawan_process_timeout_ms(RECEIVE_DELAY) == 0)
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
            }
        }

        //TODO: add rtc sleep
    }

    return 0;
}
