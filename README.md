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

| Pico | RP20401 |SX1276 Module	| RFM95W Breakout
| ------------------------------------------------
| 3V3  |  —	 | VCC	| VIN
| GND	 | GND | GND | GND
| Pin 10 | GP7	| DIO0 | G0
| Pin 11	| GP8	| NSS	| CS
| Pin 12	| GP9 |	RESET	| RST
| Pin 14	| GP10 | DIO1 | G1
| Pin 21	| GP16 | (SPI0 RX)	| MISO | MISO
| Pin 24	| GP18(SPI0 SCK)	| SCK	| SCK
| Pin 25	| GP19 (SPI0 TX)	| MOSI	| MOSI

| Pico | SHT40
| ------------
| 3V3 | VIN
| GND | GND
| SCL | Pin 6
| SCL | Pin 7

## Building
In main directory run command:
  mkdir build && cd build
  cmake ..

In the next step go to directory engine\temperature_measurement and run command:
  make -j

