"""Versioned migration registry for canonical configuration payloads."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Callable

from src.config.models import SCHEMA_VERSION


Migration = Callable[[dict[str, Any]], dict[str, Any]]
MIGRATIONS: dict[str, tuple[str, Migration]] = {}


def register_migration(
    source_version: str,
    target_version: str,
    migration: Migration,
) -> None:
    """Register the single deterministic next step from ``source_version``."""
    if source_version in MIGRATIONS:
        raise ValueError(f"A migration from {source_version!r} is already registered")
    if source_version == target_version:
        raise ValueError("A migration must change schema_version")
    MIGRATIONS[source_version] = (target_version, migration)


def migrate_payload(
    payload: dict[str, Any],
    *,
    target_version: str = SCHEMA_VERSION,
) -> dict[str, Any]:
    """Apply a registered, linear migration chain without mutating the input."""
    migrated = deepcopy(payload)
    visited: set[str] = set()
    while migrated.get("schema_version") != target_version:
        version = migrated.get("schema_version")
        if not isinstance(version, str):
            raise ValueError("Canonical configuration is missing a string schema_version")
        if version in visited:
            raise ValueError(f"Schema migration cycle detected at {version!r}")
        visited.add(version)
        try:
            next_version, migration = MIGRATIONS[version]
        except KeyError as exc:
            raise ValueError(
                f"No migration registered from schema_version={version!r} "
                f"to {target_version!r}"
            ) from exc
        migrated = migration(migrated)
        if not isinstance(migrated, dict):
            raise TypeError(f"Migration from {version!r} did not return a JSON object")
        if migrated.get("schema_version") != next_version:
            raise ValueError(
                f"Migration from {version!r} must set schema_version to {next_version!r}"
            )
    return migrated
