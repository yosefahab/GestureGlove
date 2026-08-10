"""Command line entry point for the gesture glove receiver."""

import sys
import time
from pathlib import Path

import pyautogui
import serial

from .actions import IDLE_GESTURE, Keyboard, perform, resolve_binding
from .config import DEFAULT_PROFILE, GestureProfile, read_config, select_profile
from .protocol import read_protocol
from .receiver import open_port, read_gesture

READ_TIMEOUT_S = 10.0
# the firmware emits one classification per inference window; polling faster
# only re-reads labels that resolve_binding then discards as repeats
POLL_INTERVAL_S = 0.75
# config.json sits at the project root rather than inside the package, so it
# stays editable without touching installed code
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config.json"
# protocol.json lives at the repo root because the firmware build reads it too
DEFAULT_PROTOCOL_PATH = Path(__file__).resolve().parents[3] / "protocol.json"

USAGE = "usage: glove-receiver <COM_PORT> [PROFILE]"


def listen(
    port: serial.Serial,
    profile: GestureProfile,
    keyboard: Keyboard,
    poll_interval: float,
) -> None:
    """Read gestures until the device disconnects, firing each bound shortcut."""
    previous = IDLE_GESTURE
    while True:
        time.sleep(poll_interval)
        gesture = read_gesture(port)
        if gesture is None:
            continue
        binding = resolve_binding(profile, gesture, previous)
        print("Read:", gesture, "Action:", binding)
        if binding is not None:
            perform(binding, keyboard)
        previous = gesture


def main(argv: list[str]) -> int:
    """Parse arguments, open the device, and run the listen loop."""
    if not 2 <= len(argv) <= 3:
        print(USAGE)
        return 2

    port_name = argv[1]
    profile_name = argv[2] if len(argv) == 3 else DEFAULT_PROFILE

    config = read_config(DEFAULT_CONFIG_PATH)
    if config is None:
        print(f"Could not read config: {DEFAULT_CONFIG_PATH}")
        return 1

    profile = select_profile(config, profile_name)
    if profile is None:
        print(f"No such profile: {profile_name}. Available: {', '.join(config)}")
        return 1

    protocol = read_protocol(DEFAULT_PROTOCOL_PATH)
    if protocol is None:
        print(f"Could not read protocol: {DEFAULT_PROTOCOL_PATH}")
        return 1

    print(f"Searching for device on port: {port_name} at {protocol.baud_rate} baud...")
    port = open_port(port_name, protocol.baud_rate, READ_TIMEOUT_S)
    if port is None:
        print("Device not connected! Terminating...")
        return 1

    print(f"Listening to device on port: {port_name}!")
    try:
        listen(port, profile, pyautogui, POLL_INTERVAL_S)
    except serial.SerialException:
        print("Device disconnected! Terminating...")
        return 1
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        port.close()
    return 0


def run() -> None:
    """Console script wrapper that turns the exit code into a process status."""
    sys.exit(main(sys.argv))


if __name__ == "__main__":
    run()
