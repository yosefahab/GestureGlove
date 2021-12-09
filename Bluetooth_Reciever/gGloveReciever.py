import serial
import pyautogui
import time
import sys

gestures = { "idle": None,
             "up-swipe": "volumeup",
             "down-swipe": "volumedown",
             "right-tilt": "nexttrack",
             "left-tilt": "prevtrack",
             "circular": "playpause"}

def retrieveData(ser):
    return ser.readline().decode("ascii")

prevKey = "idle"

def performAction(gest):
    global prevKey
    # prevents double actions (essentially canceling or doubling previous action)
    if prevKey != gest and gest !="idle":
        pyautogui.press(gestures.get(gest))
    prevKey = gest
    return

def main(argv):
    try:
        # COM passed in program argv
        com = argv[1]
        print("Searching for Device on port: %s..." %com)
        ser = serial.Serial(com,115200,timeout=7)
    except:
        print("Device not Connected! Terminating...")
        exit(1)
    else:
        print("Listening to device on port: %s!" %com)
        while True:
            time.sleep(.75)
            try:                    
                gest = retrieveData(ser).strip()
                # remove this print if uninterested in output
                print("Read:",gest,"Action:",gestures.get(gest))
                performAction(gest)
            except:
                print("Device Disconnected! Terminating...")
                ser.close()
                exit(1)

if __name__ == "__main__":
    main(sys.argv)
