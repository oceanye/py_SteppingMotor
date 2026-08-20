"""Persistent desktop-controller state with backwards-compatible JSON files.

The GUI historically stored each concern in a separate dotfile next to
``pc_gui.py``.  ``StateStore`` keeps those filenames and schemas stable while
centralising error handling and making every write atomic.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping


class StateStoreError(RuntimeError):
    """Raised when persisted state cannot be read or written safely."""


@dataclass(frozen=True)
class StatePaths:
    axis_config: Path
    calibration: Path
    foc_tune: Path
    gear_tune: Path

    @classmethod
    def under(cls, base_dir: str | os.PathLike[str]) -> "StatePaths":
        root = Path(base_dir)
        return cls(
            axis_config=root / ".stepper_axis.json",
            calibration=root / ".stepper_calib.json",
            foc_tune=root / ".foc_tune.json",
            gear_tune=root / ".gear_tune.json",
        )


class StateStore:
    """Read and atomically replace the legacy GUI JSON state files."""

    def __init__(
        self,
        base_dir: str | os.PathLike[str] | None = None,
        *,
        paths: StatePaths | None = None,
    ) -> None:
        if paths is not None and base_dir is not None:
            raise ValueError("pass either base_dir or paths, not both")
        if paths is None:
            paths = StatePaths.under(base_dir or Path.cwd())
        self.paths = paths

    def load_axis_config(self) -> dict[str, Any] | None:
        return self._load_object(self.paths.axis_config)

    def save_axis_config(self, data: Mapping[str, Any]) -> None:
        self._save_object(self.paths.axis_config, data)

    def load_calibration(self) -> dict[str, Any] | None:
        return self._load_object(self.paths.calibration)

    def save_calibration(self, data: Mapping[str, Any]) -> None:
        self._save_object(self.paths.calibration, data)

    def load_foc_tune(self) -> dict[str, Any] | None:
        return self._load_object(self.paths.foc_tune)

    def save_foc_tune(self, data: Mapping[str, Any]) -> None:
        self._save_object(self.paths.foc_tune, data)

    def load_gear_tune(self) -> dict[str, Any] | None:
        return self._load_object(self.paths.gear_tune)

    def save_gear_tune(self, data: Mapping[str, Any]) -> None:
        self._save_object(self.paths.gear_tune, data)

    @staticmethod
    def _load_object(path: Path) -> dict[str, Any] | None:
        if not path.exists():
            return None
        try:
            with path.open("r", encoding="utf-8") as handle:
                value = json.load(handle)
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise StateStoreError(f"cannot read {path.name}: {exc}") from exc
        if not isinstance(value, dict):
            raise StateStoreError(f"cannot read {path.name}: top level must be an object")
        return value

    @staticmethod
    def _save_object(path: Path, data: Mapping[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temp_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                newline="\n",
                prefix=f".{path.name}.",
                suffix=".tmp",
                dir=path.parent,
                delete=False,
            ) as handle:
                temp_path = Path(handle.name)
                json.dump(dict(data), handle, ensure_ascii=False, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_path, path)
            temp_path = None
        except (OSError, TypeError, ValueError) as exc:
            raise StateStoreError(f"cannot write {path.name}: {exc}") from exc
        finally:
            if temp_path is not None:
                try:
                    temp_path.unlink()
                except FileNotFoundError:
                    pass
                except OSError:
                    # The original error is more actionable.  A stale uniquely
                    # named temporary file is safe and can be removed later.
                    pass
