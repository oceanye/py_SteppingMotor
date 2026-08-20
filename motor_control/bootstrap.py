"""Desktop process bootstrap helpers kept separate from importable domain code."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import time
from typing import TextIO


_redirected_stream: TextIO | None = None


def configure_standard_streams(project_dir: str | os.PathLike[str]) -> None:
    """Preserve tracebacks when the GUI is launched with ``pythonw``.

    The function is idempotent and intentionally called by the compatibility
    launcher before importing the Tk application.
    """

    global _redirected_stream
    if _redirected_stream is not None:
        return
    needs_redirect = (
        Path(sys.executable).name.lower().startswith("pythonw")
        or sys.stderr is None
        or sys.stdout is None
    )
    if not needs_redirect:
        return
    log_dir = Path(project_dir) / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stream = (log_dir / "gui_stderr.log").open(
        "a", buffering=1, encoding="utf-8", errors="replace")
    stream.write(
        f"--- session {time.strftime('%Y-%m-%d %H:%M:%S')} "
        "(pythonw 启动，stderr 已重定向) ---\n")
    sys.stderr = stream
    sys.stdout = stream
    _redirected_stream = stream
