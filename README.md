# end-node-lorawan
Enable LoRaWAN communications on your [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/) or any RP2040 based board using a [Semtech SX1276 radio module](https://www.semtech.com/apps/product.php?pn=SX1276).

Based on the Semtech's [LoRaWAN end-device stack implementation and example projects](https://github.com/Lora-net/LoRaMac-node).

## Hardware

 * RP2040 board
   * [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/)
   * [Adafruit Feather RP2040](https://www.adafruit.com/product/4884)
 * Semtech SX1276 board
   * [Adafruit RFM95W LoRa Radio Transceiver Breakout - 868 or 915 MHz - RadioFruit](https://www.adafruit.com/product/3072)
   * [Adafruit LoRa Radio FeatherWing - RFM95W 900 MHz - RadioFruit](https://www.adafruit.com/product/3231)

### Pinout

| Raspberry Pi Pico / RP2040 | Semtech SX1276 |
| ----------------- | -------------- |
| 3.3V | VCC |
| GND | GND |
| GPIO 18 | SCK |
| GPIO 19 | MOSI |
| GPIO 16 | MISO |
| GPIO 7 | DIO0 / G0 |
| GPIO 8 | NSS / CS |
| GPIO 9 | RESET |
| GPIO 10 | DIO1 / G1 |

## Building
In main directory run command:
  mkdir build && cd build
  cmake ..

In the next step go to directory engine\temperature_measurement and run command:
  make -j

