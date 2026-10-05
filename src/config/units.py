"""Explicit unit definitions and small, deterministic SI conversions."""

from __future__ import annotations

import math


class UnitError(ValueError):
    """Raised when a unit or conversion is not part of the AV1 registry."""


CANONICAL_UNITS = {
    "dimensionless": "1",
    "length": "m",
    "mass": "kg",
    "inertia": "kg*m^2",
    "velocity": "m/s",
    "acceleration": "m/s^2",
    "force": "N",
    "moment": "N*m",
    "angle": "rad",
    "angular_rate": "rad/s",
    "time": "s",
    "roughness": "m",
}

RECOGNIZED_UNITS = frozenset(
    {
        None,
        "1",
        "m",
        "mm",
        "um",
        "kg",
        "kg*m^2",
        "m/s",
        "m/s^2",
        "g0",
        "N",
        "N*m",
        "m/N",
        "rad",
        "deg",
        "rad/s",
        "deg/s",
        "s",
        "Hz",
    }
)

_SCALE = {
    ("m", "mm"): 1000.0,
    ("mm", "m"): 0.001,
    ("m", "um"): 1_000_000.0,
    ("um", "m"): 0.000001,
    ("rad", "deg"): 180.0 / math.pi,
    ("deg", "rad"): math.pi / 180.0,
    ("rad/s", "deg/s"): 180.0 / math.pi,
    ("deg/s", "rad/s"): math.pi / 180.0,
    ("g0", "m/s^2"): 9.80665,
    ("m/s^2", "g0"): 1.0 / 9.80665,
}


def validate_unit(unit: str | None) -> None:
    if unit not in RECOGNIZED_UNITS:
        raise UnitError(f"Unrecognized unit: {unit!r}")


def convert_value(value: float, from_unit: str | None, to_unit: str | None) -> float:
    """Convert a finite scalar between registered compatible units."""
    validate_unit(from_unit)
    validate_unit(to_unit)
    if not math.isfinite(value):
        raise UnitError(f"Cannot convert non-finite value: {value!r}")
    if from_unit == to_unit:
        return float(value)
    try:
        return float(value) * _SCALE[(from_unit, to_unit)]
    except KeyError as exc:
        raise UnitError(f"Unsupported conversion: {from_unit!r} -> {to_unit!r}") from exc
