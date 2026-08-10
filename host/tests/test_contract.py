"""Checks that the host and the firmware still agree on the wire contract.

These are the drift failures that hardware testing catches late and painfully:
a baud rate that no longer matches, or a config profile naming a gesture the
model cannot produce.
"""

from pathlib import Path

import pytest

from gesture_glove.config import read_config, select_profile
from gesture_glove.labels import read_model_labels
from gesture_glove.protocol import read_protocol

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = REPO_ROOT / "host" / "config.json"
PROTOCOL_PATH = REPO_ROOT / "protocol.json"
MODEL_HEADER = REPO_ROOT / "model-parameters" / "model_variables.h"


def test_protocol_is_readable() -> None:
    protocol = read_protocol(PROTOCOL_PATH)
    assert protocol is not None
    assert protocol.baud_rate > 0


def test_generated_header_matches_protocol() -> None:
    """The firmware's generated header must carry the same baud rate.

    Skipped when the firmware has not been built, since the header only exists
    after CMake has run.
    """
    header = REPO_ROOT / "build" / "generated" / "protocol.h"
    if not header.exists():
        pytest.skip("firmware not built; no generated protocol.h")
    protocol = read_protocol(PROTOCOL_PATH)
    assert protocol is not None
    assert f"#define GLOVE_BAUD_RATE {protocol.baud_rate}" in header.read_text()


def test_model_labels_are_readable() -> None:
    labels = read_model_labels(MODEL_HEADER)
    assert labels is not None
    assert labels


@pytest.mark.parametrize(
    "profile_name", ["Default", "Music", "Games", "General"]
)
def test_profile_gestures_exist_in_model(profile_name: str) -> None:
    """Every gesture bound in a profile must be one the model can emit."""
    config = read_config(CONFIG_PATH)
    assert config is not None
    profile = select_profile(config, profile_name)
    assert profile is not None
    labels = read_model_labels(MODEL_HEADER)
    assert labels is not None

    unknown = set(profile.bindings) - set(labels)
    assert not unknown, f"{profile_name} binds gestures the model cannot emit: {unknown}"


@pytest.mark.parametrize(
    "profile_name", ["Default", "Music", "Games", "General"]
)
def test_profile_covers_every_model_label(profile_name: str) -> None:
    """Every gesture the model emits should have a binding, so none is silently dead."""
    config = read_config(CONFIG_PATH)
    assert config is not None
    profile = select_profile(config, profile_name)
    assert profile is not None
    labels = read_model_labels(MODEL_HEADER)
    assert labels is not None

    missing = set(labels) - set(profile.bindings)
    assert not missing, f"{profile_name} has no binding for: {missing}"
