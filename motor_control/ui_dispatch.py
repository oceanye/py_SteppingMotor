"""Thread-to-UI dispatch without importing Tk.

The desktop app schedules :meth:`UiDispatcher.drain` from ``root.after``.
Workers may either post fire-and-forget updates or synchronously call an action
when an API response must reflect the state actually applied on the UI thread.
"""

from __future__ import annotations

import queue
import threading
from collections.abc import Callable
from typing import Any


class UiDispatchTimeout(TimeoutError):
    pass


class UiDispatcher:
    def __init__(self, on_async_error: Callable[[BaseException], None] | None = None):
        self._actions: queue.Queue[Callable[[], Any]] = queue.Queue()
        self._owner_thread = threading.get_ident()
        self._on_async_error = on_async_error

    def post(self, callback: Callable[[], Any]) -> None:
        self._actions.put(callback)

    def call(self, callback: Callable[[], Any], timeout: float = 2.0) -> Any:
        """Run ``callback`` on the owner thread and return its result.

        Calling from the owner thread executes immediately.  Worker callers
        block only until the UI's normal drain cycle processes the action.
        Exceptions raised by the callback are re-raised in the worker.
        """
        if threading.get_ident() == self._owner_thread:
            return callback()
        completed = threading.Event()
        outcome: dict[str, Any] = {}
        state_lock = threading.Lock()
        state = {"phase": "queued"}

        def invoke() -> None:
            with state_lock:
                if state["phase"] == "cancelled":
                    return
                state["phase"] = "running"
            try:
                outcome["value"] = callback()
            except BaseException as exc:  # propagated to the waiting caller
                outcome["error"] = exc
            finally:
                with state_lock:
                    state["phase"] = "done"
                completed.set()

        self.post(invoke)
        if not completed.wait(timeout):
            with state_lock:
                if state["phase"] == "queued":
                    state["phase"] = "cancelled"
                    raise UiDispatchTimeout(
                        "UI thread did not process the action in time"
                    )
            # The owner picked up the callback at the timeout boundary.  Once
            # mutation has begun it cannot safely be cancelled, so return its
            # real result instead of reporting a failure followed by a late
            # state change.
            completed.wait()
        if "error" in outcome:
            raise outcome["error"]
        return outcome.get("value")

    def drain(self, limit: int = 100) -> int:
        """Execute up to ``limit`` queued actions on the owner thread."""
        if threading.get_ident() != self._owner_thread:
            raise RuntimeError("UiDispatcher.drain must run on its owner thread")
        processed = 0
        for _ in range(limit):
            try:
                callback = self._actions.get_nowait()
            except queue.Empty:
                break
            try:
                callback()
            except BaseException as exc:
                if self._on_async_error is not None:
                    self._on_async_error(exc)
            processed += 1
        return processed
