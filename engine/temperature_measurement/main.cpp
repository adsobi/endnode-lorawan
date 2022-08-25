#include <stdio.h>
#include <string.h>
#include <bits/stdc++.h>
#include <stdint.h>
#include <stdlib.h>
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/sleep.h"
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "Adafruit_SHT4x.h"
//#include "tusb.h"

extern "C"
{
    #include "pico/lorawan.h"
    #include "config.h"
}

#define RECEIVE_DELAY                           2000
#define SEND_DATA_DELAY                         8000
#define SIZE_OF_MEASUREMENTS_BUFFER             10
#define MEASUREMENT_CORRECTNESS                 0.02
#define NUMBER_OF_PREDICTED_MEASUREMENTS        1
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE        242
#define ANTENNA_ITER_THRESHOLD                  20

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

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
int iter_for_prediction = 0;

    //save values for later
    uint scb_orig = scb_hw->scr;
    uint clock0_orig = clocks_hw->sleep_en0;
    uint clock1_orig = clocks_hw->sleep_en1;

void init_sht4x_sensor()
{
    printf("Adafruit SHT4x test\n");
    if (!sht4.begin())
    {
        printf("Couldn't find SHT4x");
        while (true)
            delay(true);
    }
    printf("Found SHT4x sensor: ");

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
// ----------------------------- SLEEP - START ---------------------------------
/*
*   Maximumum value of 'second_to_sleep_to' is 86399 what is equals 23h 59m 59s.
*/
void recover_from_sleep(uint scb_orig, uint clock0_orig, uint clock1_orig){

    //Re-enable ring Oscillator control
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

    //reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = clock0_orig;
    clocks_hw->sleep_en1 = clock1_orig;

    //reset clock
    clocks_init();
    stdio_init_all();
    return;
}

static void sleep_callback() {
    recover_from_sleep(scb_orig, clock0_orig, clock1_orig);
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    printf("RTC woke up\n");
    uart_default_tx_wait_blocking();
    return;
}

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

    gpio_put(PICO_DEFAULT_LED_PIN, false);
    rtc_init();
    rtc_set_datetime(&t);
    sleep_run_from_xosc();
    sleep_goto_sleep_until(&t_alarm, &sleep_callback);
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
// ------------------------------ SLEEP - END ----------------------------------

// ---------------------------- PREDICTION - START -----------------------------
bool custom_sort(double a, double b)
{
    double a1=abs(a-0);
    double b1=abs(b-0);
    return a1<b1;
}

static void forupdate_params_for_prediction(
    std::vector<double>& buffer,
    std::pair<double, double>& params
) {
   /*Intialization Phase*/
    double err;
    double alpha = 0.01;
    params.first = 0.00;
    params.second = 0.00;

    /*Training Phase*/
    for (int epoch = 0; epoch < 2000; epoch ++)
    {
        int idx = epoch % SIZE_OF_MEASUREMENTS_BUFFER;
        double p = params.first + params.second * idx;
        err = p - buffer[idx];
        params.first = params.first - alpha * err;
        params.second = params.second - alpha * err * idx;
    }
    printf("Final Values are: B0=%f B1=%f\n", params.first, params.second);
    iter_for_prediction = 0;
}

double predict_temp_value(float& actual_temp, std::pair<double, double>& params)
{
    double predicted_value = params.first +
        (params.second * (SIZE_OF_MEASUREMENTS_BUFFER + iter_for_prediction));
    return predicted_value;
}

// ---------------------------- PREDICTION - END -------------------------------

int set_antenna_power(int rssi)
{
    if (rssi <= -100) return TX_POWER_0;
    else if (rssi <= -85 && rssi > -100) return TX_POWER_1;
    else if (rssi <= -70 && rssi > -85) return TX_POWER_2;
    else if (rssi <= -55 && rssi > -70) return TX_POWER_3;
    else if (rssi <= -40 && rssi > -55) return TX_POWER_4;
    else if (rssi <= -20 && rssi > -40) return TX_POWER_5;
    else if (rssi > -20) return TX_POWER_6;
}

int increase_tx_power(int tx_power)
{
    if (tx_power > TX_POWER_0) return tx_power--;
    return tx_power;
}

int decrease_tx_power(int tx_power)
{
    if (tx_power < TX_POWER_6) return tx_power++;
    return tx_power;
}

int adjust_antenna_power(int rssi, int tx_power)
{
    if (rssi < -130) return increase_tx_power(tx_power);
    else if (rssi > -100) return decrease_tx_power(tx_power);

    return tx_power;
}

double round_with_2_precision(double value)
{
    return round((value * 100)) / 100;}

int main(void)
{
    std::vector<double> temp_measurements;
    // SHT40 sensor provides measurements between -40 C to +125 C
    float predicted_temp = -150.00;
    float previous_predicted_temp = -150.00;
    std::pair<double, double> params(0,0);
    int antenna_gain = TX_POWER_0;
    int iter_for_antenna_gain = 0;

    // variables for receiving data
    int receive_length = 0;
    uint8_t receive_buffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];
    uint8_t receive_port = 0;

    // Initialize stdio and wait for USB CDC connect
    stdio_init_all();
    // while (!tud_cdc_connected())
    // {
    //     tight_loop_contents();
    // }

    // Initialize SHT40 sensor
    init_sht4x_sensor();
    sensors_event_t humidity, temp;

    // Initialize the LED pin and internal temperature ADC
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    //uncomment next line to enable debug
    lorawan_debug(true);

    // Initialize the LoRaWAN stack
     printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_otaa(&sx1276_settings, LORAWAN_REGION, &otaa_settings) < 0) {
        printf("failed!!!\n");
        while (true) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    // Start the join process and wait
    printf("Joining LoRaWAN network ... ");
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process();
    }
    bool antenna_gain_setted = false;
    printf("joined successfully!\n");
    uart_default_tx_wait_blocking();
    while (true)
    {
        rapidjson::StringBuffer message;
        rapidjson::Writer<rapidjson::StringBuffer> writer(message);
        writer.SetMaxDecimalPlaces(2);

        // Read values from sensor
        sht4.getEvent(&humidity, &temp);
        printf("Temperature: %f degrees C\n", temp.temperature);
        printf("Humidity: %f% rH\n", humidity.relative_humidity);;
        uart_default_tx_wait_blocking();

        // Update b0 and b1 params if buffer is filled and prediction is not correct
        if (
            temp_measurements.size() > SIZE_OF_MEASUREMENTS_BUFFER &&
            abs(temp.temperature - previous_predicted_temp) > (temp.temperature * MEASUREMENT_CORRECTNESS)
        ){
            update_params_for_prediction(temp_measurements, params);
            iter_for_prediction = 0;
        }

        writer.StartObject();
        writer.Key("delay");
        writer.Int64(SEND_DATA_DELAY);
        writer.Key("0");
        writer.Double(round_with_2_precision(temp.temperature));

        // If 10 values are in buffer start prediction of temperature
        if (params.first != 0 && params.second != 0) {
            //TODO: make prediction of many values
            previous_predicted_temp = predicted_temp;
            predicted_temp = predict_temp_value(temp.temperature, params);
            writer.Key("1");
            writer.Double(round_with_2_precision(predicted_temp));
            temp_measurements.erase(temp_measurements.begin());
        }

        writer.EndObject();
        printf("%s\n", message.GetString());
        printf("BUFFER SIZE: %i\n", temp_measurements.size());

        if (
            temp_measurements.size() < SIZE_OF_MEASUREMENTS_BUFFER ||
            !(bool)(iter_for_prediction % NUMBER_OF_PREDICTED_MEASUREMENTS)
        )
        {
            if (lorawan_send_unconfirmed(message.GetString(),
                    strlen(message.GetString()), 2, antenna_gain
                ) < 0)
            {
                printf("failed!!!\n");
            }
            else
            {
                if (iter_for_antenna_gain == ANTENNA_ITER_THRESHOLD)
                {
                    antenna_gain = TX_POWER_0;
                    iter_for_antenna_gain = 0;
                } else {
                    iter_for_antenna_gain++;
                }
                gpio_put(PICO_DEFAULT_LED_PIN, true);
                printf("success!\n");
            }

            if (lorawan_process_timeout_ms(RECEIVE_DELAY) == 0)
            {
                // check if a downlink message was received
                receive_length = lorawan_receive(
                    receive_buffer, sizeof(receive_buffer), &receive_port
                );
                if (receive_length > -1 && lorawan_rx_rssi() != NULL)
                {
                    iter_for_antenna_gain = 0;

                    if (!antenna_gain_setted) {
                        antenna_gain = set_antenna_power(lorawan_rx_rssi());
                        antenna_gain_setted = true;
                    } else {
                        antenna_gain = adjust_antenna_power(lorawan_rx_rssi(), antenna_gain);
                    }
                }
            }
        }

        iter_for_prediction++;
        temp_measurements.push_back(temp.temperature);

        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(SEND_DATA_DELAY);
    }

    return 0;
}
