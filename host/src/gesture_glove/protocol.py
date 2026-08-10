"""Constants shared with the firmware, read from the repo's protocol.json.

The firmware gets the same values through a generated protocol.h, so the link
settings cannot drift between the two sides.
"""

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Protocol:
    """Link settings that both sides of the wire must agree on."""

    baud_rate: int


def read_protocol(path: Path) -> Protocol | None:
    """Read protocol.json, or None if it is missing or lacks a valid baud rate."""
    try:
        raw = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    baud_rate = raw.get("baud_rate")
    if not isinstance(baud_rate, int):
        return None
    return Protocol(baud_rate=baud_rate)
