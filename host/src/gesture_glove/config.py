"""Loading of gesture to shortcut profiles."""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

DEFAULT_PROFILE = "Default"

# raw shape on disk: {profile_name: [{gesture: shortcut, ...}]}
RawConfig = dict[str, list[dict[str, str]]]


@dataclass(frozen=True)
class GestureProfile:
    """A named set of gesture to keyboard shortcut bindings."""

    name: str
    bindings: Mapping[str, str]


def read_config(path: Path) -> RawConfig | None:
    """Read the raw profile table, or None if the file is missing or malformed.

    A bad config is user error rather than programmer error, so the caller gets
    None to report on instead of a traceback.
    """
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def select_profile(config: RawConfig, name: str) -> GestureProfile | None:
    """Extract one profile from the raw table, or None if absent or malformed.

    Each profile is stored as a single element list wrapping the bindings
    object; that shape predates this package and is kept so existing
    config.json files still load unchanged.
    """
    entries = config.get(name)
    if not entries:
        return None
    bindings = entries[0]
    if not isinstance(bindings, dict):
        return None
    return GestureProfile(name=name, bindings=dict(bindings))
