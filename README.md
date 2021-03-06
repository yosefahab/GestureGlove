# Gesture Glove
ML Gesture recognition using edge impulse & RPI pico.
Detects gestures and performs shortcuts accordingly using a python script.
###### Fun fact: EdgeImpulse mentioned GestureGlove on [linkedin](https://www.linkedin.com/feed/update/urn:li:activity:6909140611157606400/) on #PiDay

# Requirements
* Cmake
* Pico SDK
* python
* pyserial ``` pip3 install pyserial ```
* pyautogui ``` pip3 install pyautogui ```

# Steps
### 1. connect your pico while holding the bootsel button
### 2. run the following:
    ```
    mkdir build
    && cd build
    && cmake ..
    && make
    ```
### 3. copy build/gGlove.uf2 to RPI-RP2 when its connected

# Connections
* PICO 3v3  -> MPU6050 VCC
* PICO GND  -> MPU6050 GND
* PICO GP4  -> MPU6050 SDA
* PICO GP5  -> MPU6050 SCL
* PICO GP0  -> HC-05 TX
* PICO GP1  -> HC-05 RX
* PICO VBUS -> HC-05 VCC
* PICO GND  -> HC-05 GND

# Bluetooth_Reciever
Python script to recieve data from the Pico and perform shortcuts saved in **config.json** file.  
If no custom configuration is passed in command arguments, the script uses the default configuration.  
##### To run :     
    python gGloveReciever.py <COM_NUMBER> <CONFIG_OBJECT(optional)>

> NOTE:
>
> * THIS BRANCH USES **BLUETOOTH** NOT USB
> * THIS PYTHON SCRIPT IS INTENDED FOR **WINDOWS**, IT HAS NOT BEEN TESTED WITH OTHER PLATFORMS.
