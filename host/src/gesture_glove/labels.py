"""Extraction of the model's gesture labels from the Edge Impulse export.

The firmware prints these strings verbatim, so config.json has to use exactly
the same set. Parsing the generated header is ugly, but it keeps the model as
the single source of truth: re-exporting the model updates the labels here too.
"""

import re
from pathlib import Path

_CATEGORIES = re.compile(
    r"ei_classifier_inferencing_categories\[\]\s*=\s*\{(?P<body>[^}]*)\}"
)


def read_model_labels(header_path: Path) -> tuple[str, ...] | None:
    """Read the model's class labels, or None if the header cannot be parsed."""
    try:
        source = header_path.read_text()
    except OSError:
        return None
    match = _CATEGORIES.search(source)
    if match is None:
        return None
    return tuple(re.findall(r'"([^"]*)"', match.group("body")))
