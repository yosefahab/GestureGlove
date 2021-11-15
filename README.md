# Steps to build project and upload project
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
* PICO 3v3 -> MPU6050 VCC
* PICO GND -> MPU6050 GND
* PICO GP4 -> MPU6050 SDA
* PICO GP5 -> MPU6050 SCL

> **NOTE: THIS BRANCH USES USB NOT BLUETOOTH**