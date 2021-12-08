import serial
import pyautogui
import time

gestures = { "idle": None,
             "up-swipe": "volumeup",
             "down-swipe": "volumedown",
             "right-tilt": "nexttrack",
             "left-tilt": "prevtrack",
             "circular": "playpause",}

def retrieveData(ser):
    return ser.readline().decode("ascii")

prevKey = "idle"

def performAction(gest):
    global prevKey
    if prevKey != gest and gest !="idle":
        pyautogui.press(gestures.get(gest))
        time.sleep(.75)
    prevKey = gest
    return

def main():
    # change COM accordingly
    ser = serial.Serial("COM5",115200,Timeout =1)
    while True:
        gest = retrieveData(ser).strip()
        print("Read:",gest,"Action:",gestures.get(gest))
        performAction(gest)

if __name__ == "__main__":
    main()
