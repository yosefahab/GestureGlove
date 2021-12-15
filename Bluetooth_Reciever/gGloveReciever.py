import serial
import pyautogui
import time
import sys
import json

gestures = dict()
prevKey = "idle"


def retrieveData(ser):
    return ser.readline().decode("ascii")


def performAction(gest):
    global prevKey
    # prevents double actions (essentially canceling or doubling previous action)
    if prevKey != gest and gest != "idle":
        g = gestures.get(gest)
        # if shortcut is a combination of keys seperated by +
        if '+' in g:
            pyautogui.hotkey(*g.split('+'))
        else:
            pyautogui.press(g)

    prevKey = gest
    return


def retrieveConfig(obj):
    global gestures
    with open('config.json') as json_file:
        gestures = dict(*json.loads(json_file.read())[obj])


def main(argv):

    try:
        if len(argv) < 2:
            raise Exception("Please specify port!")
        elif (len(argv) > 3):
            raise Exception("Incorrect number of arguments!")
        # custom configuration object passed in program argv
        obj = argv[2] if len(argv) == 3 else "Default"
        retrieveConfig(obj)
        # COM passed in program argv
        com = argv[1]
        print("Searching for Device on port: %s..." % com)
        ser = serial.Serial(com, 115200, timeout=10)
    except OSError:
        print("Device not Connected! Terminating...")
    except Exception as e:
        print(e)
    else:
        print("Listening to device on port: %s!" % com)
        while True:
            time.sleep(.75)
            try:
                gest = retrieveData(ser).strip()
                # remove this print if uninterested in output
                print("Read:", gest, "Action:", gestures.get(gest))
                performAction(gest)
            except:
                print("Device Disconnected! Terminating...")
                ser.close()
                exit(1)


if __name__ == "__main__":
    main(sys.argv)


# The following are all available keys in PYAUTOGUI
# You may use a combination of them seperated by '+'
# ['\t', '\n', '\r', ' ', '!', '"', '#', '$', '%', '&', "'", '(', ')',
# '*', '+', ',', '-', '.', '/', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
# ':', ';', '<', '=', '>', '?', '@', '[', '\\', ']', '^', '_', '`',
# 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
# '{', '|', '}', '~', 'accept', 'add', 'alt', 'altleft', 'altright', 'apps', 'backspace',
# 'browserback', 'browserfavorites', 'browserforward', 'browserhome', 'browserrefresh', 'browsersearch', 'browserstop',
# 'capslock', 'clear', 'convert','ctrl', 'ctrlleft', 'ctrlright', 'decimal',
#  'del', 'delete', 'divide', 'down', 'end', 'enter', 'esc', 'escape', 'execute',
# 'f1', 'f2', 'f3', 'f4', 'f5', 'f6', 'f7', 'f8', 'f9', 'f10',
# 'f11', 'f12', 'f13', 'f14', 'f15', 'f16', 'f17', 'f18', 'f19', 'f20', 'f21', 'f22', 'f23', 'f24',
# 'final', 'fn', 'hanguel', 'hangul', 'hanja', 'help', 'home', 'insert', 'junja', 'kana', 'kanji',
# 'launchapp1', 'launchapp2', 'launchmail', 'launchmediaselect', 'left', 'modechange', 'multiply', 'nexttrack', 'nonconvert',
# 'num0', 'num1', 'num2', 'num3', 'num4', 'num5', 'num6', 'num7', 'num8', 'num9','numlock',
# 'pagedown', 'pageup', 'pause', 'pgdn', 'pgup', 'playpause', 'prevtrack', 'print', 'printscreen', 'prntscrn', 'prtsc', 'prtscr',
# 'return', 'right', 'scrolllock', 'select', 'separator', 'shift', 'shiftleft', 'shiftright', 'sleep', 'space', 'stop', 'subtract',
# 'tab', 'up', 'volumedown', 'volumemute', 'volumeup', 'win', 'winleft', 'winright', 'yen', 'command', 'option', 'optionleft', 'optionright']