"""Coordinate-frame metadata for the existing AeroVECTOR conventions."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CoordinateFrameDefinition:
    name: str
    axes: tuple[str, ...]
    description: str


GLOBAL_FRAME = CoordinateFrameDefinition(
    name="aerovector_global_2d",
    axes=("X_global_up", "Z_global_downrange"),
    description=(
        "Right-handed simulation plane used by the legacy model. Array index 0 "
        "is altitude/up; index 1 is horizontal/downrange."
    ),
)

BODY_FRAME = CoordinateFrameDefinition(
    name="aerovector_body_2d",
    axes=("X_body_forward", "Z_body_transverse"),
    description=(
        "X_body points from tail toward nose along the longitudinal axis. "
        "Z_body is the transverse in-plane axis used for normal force."
    ),
)

FRAME_CONVENTIONS = {
    "global_frame": GLOBAL_FRAME.name,
    "body_frame": BODY_FRAME.name,
    "zero_pitch": "body X axis aligned with global up",
    "positive_pitch": (
        "positive theta uses the legacy body-to-global matrix "
        "[[cos(theta), sin(theta)], [-sin(theta), cos(theta)]]"
    ),
    "angle_unit": "rad",
    "axial_datum": "nose tip, positive toward tail",
    "simulation_altitude_reference": "AGL relative to launch point",
    "atmosphere_altitude_reference": "MSL = launch_altitude_msl + altitude_agl",
}
