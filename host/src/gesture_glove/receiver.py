"""Serial transport: turning bytes from the glove into gesture labels."""

from typing import Protocol

import serial


class SerialPort(Protocol):
    """Structural view of the port, so reading can be tested without hardware."""

    def readline(self) -> bytes: ...


def open_port(name: str, baud: int, timeout: float) -> serial.Serial | None:
    """Open the glove's serial port, or None if no device is present there."""
    try:
        return serial.Serial(name, baud, timeout=timeout)
    except serial.SerialException:
        return None


def read_gesture(port: SerialPort) -> str | None:
    """Read one gesture label, or None if the line was not decodable as ascii.

    A garbled line means a dropped byte over the radio link, which is worth
    skipping rather than crashing on.
    """
    try:
        return port.readline().decode("ascii").strip()
    except UnicodeDecodeError:
        return None
