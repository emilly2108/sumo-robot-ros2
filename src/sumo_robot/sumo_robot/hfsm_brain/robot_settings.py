"""Read one authoritative YAML file for all runtime tuning."""

from pathlib import Path
import math

try:
    import yaml
except ImportError as exc:
    raise RuntimeError("PyYAML is required: install python3-yaml") from exc


SETTINGS_PATH = Path(__file__).with_name("robot_settings.yaml")


def load_section(name: str, expected_keys=None) -> dict:
    try:
        with SETTINGS_PATH.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Cannot load {SETTINGS_PATH}: {exc}") from exc

    if not isinstance(document, dict):
        raise ValueError(f"{SETTINGS_PATH} must contain a YAML mapping")
    section = document.get(name)
    if not isinstance(section, dict):
        raise ValueError(f"Missing or invalid '{name}' section in {SETTINGS_PATH}")

    if expected_keys is not None:
        required = set(expected_keys)
        missing = required - section.keys()
        extra = section.keys() - required
        if missing or extra:
            raise ValueError(
                f"Invalid {name} settings: missing={sorted(missing)}, extra={sorted(extra)}"
            )

    for key, value in section.items():
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"{name}.{key} must be finite")
    return section
