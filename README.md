# Steps
### 1. connect your pico while holding the bootsel button
### 2. run the following:
    ```
    mkdir build
    && cd build
    && cmake ..
    && make
    && cp ./gGlove.uf2 /media/"$USER"/RPI-RP2

    ```
# Connections
* PICO 3v3  -> MPU6050 VCC
* PICO GND  -> MPU6050 GND
* PICO GP4  -> MPU6050 SDA
* PICO GP5  -> MPU6050 SCL
* PICO GP0  -> HC-05 TX
* PICO GP1  -> HC-05 RX
* PICO VBUS -> HC-05 VCC
* PICO GND  -> HC-05 GND

> NOTE: THIS BRANCH USES BLUETOOTH NOT USB