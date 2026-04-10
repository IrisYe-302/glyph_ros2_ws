from __future__ import annotations

import os
from typing import Any, Optional, Tuple


def _infer_jetson_model_name() -> Optional[str]:
    if os.environ.get("JETSON_MODEL_NAME"):
        return None

    try:
        with open("/proc/device-tree/compatible", "rb") as compatible_file:
            compatible = compatible_file.read().decode("utf-8", errors="ignore")
    except OSError:
        return None

    entries = [entry for entry in compatible.split("\x00") if entry]
    normalized_entries = {entry.removesuffix("-super") for entry in entries}

    orin_nano_compatibles = {
        "nvidia,p3509-0000+p3767-0003",
        "nvidia,p3768-0000+p3767-0003",
        "nvidia,p3509-0000+p3767-0004",
        "nvidia,p3768-0000+p3767-0004",
        "nvidia,p3509-0000+p3767-0005",
        "nvidia,p3768-0000+p3767-0005",
    }
    if normalized_entries & orin_nano_compatibles:
        return "JETSON_ORIN_NANO"
    return None


def import_jetson_gpio() -> Tuple[Any, Optional[Exception], Optional[str]]:
    inferred_jetson_model = _infer_jetson_model_name()
    if inferred_jetson_model is not None:
        os.environ["JETSON_MODEL_NAME"] = inferred_jetson_model

    try:
        import Jetson.GPIO as gpio

        return gpio, None, inferred_jetson_model
    except Exception as exc:  # pragma: no cover - hardware-specific dependency
        return None, exc, inferred_jetson_model
