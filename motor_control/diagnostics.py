"""Bounded, asynchronous diagnostics. Never performs network or motor I/O."""
from __future__ import annotations

import atexit
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from functools import wraps
import hashlib
import ipaddress
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import platform
import queue
import re
import subprocess
import sys
import threading
import time
import traceback
import uuid


SCHEMA = "motor-diagnostics-v1"
CHANNELS = ("gui", "motor", "telemetry", "events", "flight")
LOG_FILE_PATTERN = r"(?:gui|motor|telemetry|events|flight)\.jsonl(?:\.\d+)?|session\.json"
_operation = ContextVar("motor_diagnostic_operation", default=None)
_active = {}
_active_lock = threading.Lock()
_SECRET_KEY = re.compile(r"password|passwd|secret|token|authorization|cookie|api[_-]?key|credential", re.I)


def sanitize(value):
    """Redact common credentials/identifiers; preserve mechanical measurements."""
    if is_dataclass(value) and not isinstance(value, type):
        value = asdict(value)
    if isinstance(value, dict):
        return {str(k): "<redacted>" if _SECRET_KEY.search(str(k)) else sanitize(v)
                for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [sanitize(v) for v in value]
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    text = str(value)
    text = re.sub(r"(?i)([a-z]:[\\/](?:users|documents and settings)[\\/])[^\\/\s\"']+",
                  r"\1<user>", text)
    text = re.sub(r"(/(?:home|Users)/)[^/\s\"']+", r"\1<user>", text)
    text = re.sub(r"(?i)(https?://)[^/\s@]+@", r"\1<credentials>@", text)
    text = re.sub(r"(?i)\b(Bearer|Basic)\s+[\w.~/+\-=]+", r"\1 <redacted>", text)
    text = re.sub(r'''(?i)((?:password|passwd|secret|token|authorization|cookie|api[_-]?key|credential)[\w-]*["']?\s*[:=]\s*)(?:"[^"\n]*"|'[^'\n]*'|[^\s,;&]+)''',
                  r"\1<redacted>", text)
    text = re.sub(r"\b(?:gh[pousr]_[A-Za-z0-9_]+|github_pat_[A-Za-z0-9_]+|sk-[A-Za-z0-9_-]{12,})\b",
                  "<credential>", text)
    text = re.sub(r"(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])", "<ip>", text)
    def ipv6(match):
        try:
            return "<ip6>" if ipaddress.ip_address(match[0]).version == 6 else match[0]
        except ValueError:
            return match[0]
    text = re.sub(r"(?<!\w)(?:[0-9a-fA-F]*:){2,}[0-9a-fA-F:.]*(?!\w)", ipv6, text)
    text = re.sub(r"\b[A-Za-z0-9_.+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}\b", "<email>", text)
    return text


def operation_id():
    return _operation.get()


@contextmanager
def operation_scope(identifier=None):
    token = _operation.set(identifier or uuid.uuid4().hex[:16])
    try:
        yield _operation.get()
    finally:
        _operation.reset(token)


def logged_action(function):
    """Observe UI actions without changing return values or error semantics."""
    @wraps(function)
    def wrapped(self, *args, **kwargs):
        recorder = getattr(self, "diagnostics", None)
        if recorder is None:
            return function(self, *args, **kwargs)
        with operation_scope(operation_id()):
            recorder.event("gui", "action.begin", action=function.__name__,
                           args=args, kwargs=kwargs)
            try:
                result = function(self, *args, **kwargs)
            except Exception as exc:
                recorder.exception("gui", "action.error", exc, action=function.__name__)
                raise
            recorder.event("gui", "action.return", action=function.__name__)
            return result
    return wrapped


def environment_summary(project_dir):
    root = Path(project_dir)
    summary = {"python": platform.python_version(), "platform": platform.platform(),
               "executable": Path(sys.executable).name}
    for name, args in (("commit", ["rev-parse", "HEAD"]),
                       ("branch", ["branch", "--show-current"]),
                       ("worktree", ["status", "--porcelain", "--untracked-files=no"])):
        try:
            result = subprocess.run(["git", *args], cwd=root, capture_output=True,
                                    text=True, encoding="utf-8", errors="replace", timeout=3,
                                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            summary[name] = result.stdout.strip() if result.returncode == 0 else "unavailable"
        except (OSError, subprocess.SubprocessError):
            summary[name] = "unavailable"
    summary["source_sha256"] = {}
    for name in ("desktop_app.py", "serial_session.py", "diagnostics.py", "gait_executor.py",
                 "gait_coordinates.py", "gait_planner.py", "gait_avoidance.py",
                 "gait_clearance.py", "gait_x42s_plan.py", "gait_x42s_wait.py",
                 "x42s_motion.py", "x42s_runtime.py", "x42s_safety.py",
                 "diagnostic_policy.py", "ui/gait_tab.py", "ui/gait_twin.py"):
        path = root / "motor_control" / name
        if path.is_file():
            summary["source_sha256"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return sanitize(summary)


class _JsonlHandler(RotatingFileHandler):
    rotations = 0

    def doRollover(self):
        super().doRollover()
        self.rotations += 1

    def handleError(self, _record):
        raise  # Let the writer mark a failure; do not recurse via stderr hooks.


class DiagnosticRecorder:
    """One writer, bounded queue, five rotating streams, explicit loss counters.

    Queue overflow/disk failures never block control threads. They are visible in
    health(), the UI, session end and export metadata. Recent CLOSED sessions are
    retained (20 by default); active/crashed/foreign directories are not removed.
    """
    def __init__(self, project_dir, *, max_bytes=2_000_000, backups=2,
                 queue_size=8192, keep_sessions=20, metadata=None):
        self.project_dir = Path(project_dir).resolve()
        self.session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:10]
        self.directory = self.project_dir / "logs" / "diagnostics" / self.session_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self._queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._files_lock = threading.Lock()
        self._seq = 0
        self._closed = False
        self._dropped = 0
        self._failed = 0
        self._last_error = None
        self._start = time.monotonic()
        from .diagnostic_policy import EvidencePolicy
        self._evidence_policy = EvidencePolicy()
        self._handlers = {c: _JsonlHandler(self.directory / f"{c}.jsonl",
                          maxBytes=max_bytes, backupCount=backups, encoding="utf-8")
                          for c in CHANNELS}
        for handler in self._handlers.values():
            handler.setFormatter(logging.Formatter("%(message)s"))
        self.manifest = {"schema": SCHEMA, "session_id": self.session_id, "closed": False,
                         "started_utc": datetime.now(timezone.utc).isoformat(),
                         "limits": {"max_bytes_per_file": max_bytes, "backups": backups,
                                    "queue_size": queue_size, "keep_closed_sessions": keep_sessions},
                         "metadata": sanitize(metadata or {})}
        self.manifest["evidence_policy"] = {
            "events": "actions, command writes/replies, safety and health transitions; no routine poll traffic",
            "flight": "bounded pre-fault 20s / post-fault 10s, pre-buffer max4096 records",
            "retention": "all streams rotate; dropped=0 does NOT imply complete session history"}
        self._write_manifest()
        self._thread = threading.Thread(target=self._write_loop, name="diagnostic-writer", daemon=True)
        self._thread.start()
        self.event("gui", "session.start", **self.manifest["metadata"])
        self._prune_closed(keep_sessions)
        atexit.register(self.close)

    def _write_manifest(self):
        (self.directory / "session.json").write_text(
            json.dumps(sanitize(self.manifest), ensure_ascii=False, indent=2), encoding="utf-8")

    def event(self, channel, event, *, level="INFO", **fields):
        # All work here is bounded; actual filesystem writes belong to the writer.
        try:
            cleaned = sanitize(fields)
            with self._lock:
                if self._closed:
                    return
                self._seq += 1
                record = {"schema": SCHEMA, "session_id": self.session_id, "seq": self._seq,
                          "utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                          "elapsed_ms": round((time.monotonic() - self._start) * 1000, 3),
                          "thread": threading.current_thread().name,
                          "channel": channel, "event": event, "level": level,
                          "operation_id": cleaned.pop("operation_id", operation_id()), "data": cleaned}
                try:
                    self._queue.put_nowait((channel, record))
                except queue.Full:
                    self._dropped += 1
        except Exception as exc:
            with self._lock:
                self._failed += 1
                self._last_error = str(sanitize(exc))

    def exception(self, channel, event, error, **fields):
        self.event(channel, event, level="ERROR", error_type=type(error).__name__,
                   error=str(error), traceback="".join(traceback.format_exception(
                       type(error), error, error.__traceback__)), **fields)

    def health(self):
        with self._lock:
            return {"session_id": self.session_id, "queued": self._queue.qsize(),
                    "dropped": self._dropped, "write_errors": self._failed,
                    "last_error": self._last_error, "closed": self._closed,
                    "rotations": {c: h.rotations for c, h in self._handlers.items()}}

    def _write_loop(self):
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                if isinstance(item, threading.Event):
                    item.set()
                    continue
                channel, record = item
                text = json.dumps(record, ensure_ascii=False, allow_nan=False)
                with self._files_lock:
                    self._handlers[channel].emit(logging.LogRecord(
                        "motor.diagnostic", logging.INFO, "", 0, text, (), None))
                    if channel not in ("events", "flight"):
                        for target, evidence in self._evidence_policy.route(record):
                            self._handlers[target].emit(logging.LogRecord(
                                "motor.diagnostic", logging.INFO, "", 0,
                                json.dumps(evidence, ensure_ascii=False, allow_nan=False), (), None))
            except Exception as exc:
                with self._lock:
                    self._failed += 1
                    self._last_error = str(sanitize(exc))
            finally:
                self._queue.task_done()

    def flush(self, timeout=3):
        if not self._thread.is_alive():
            return self._queue.empty()
        marker = threading.Event()
        try:
            self._queue.put(marker, timeout=timeout)
        except queue.Full:
            return False
        return marker.wait(timeout)

    def snapshot_files(self):
        """Consistent, bounded read for export; only the writer pauses briefly."""
        if not self.flush():
            raise RuntimeError("日志队列未及时写完，请稍后导出")
        with self._files_lock:
            files = {p.name: p.read_text(encoding="utf-8", errors="replace")
                     for p in self.directory.iterdir() if p.is_file() and not p.is_symlink()
                     and re.fullmatch(LOG_FILE_PATTERN, p.name)}
        files["session.json"] = json.dumps({**self.manifest, "health": self.health()}, ensure_ascii=False)
        return files

    def close(self):
        with self._lock:
            if self._closed:
                return
        self.event("gui", "session.end", health=self.health())
        with self._lock:
            self._closed = True
        try:
            self._queue.put(None, timeout=3)
        except queue.Full:
            return  # Do not mark the session fully closed while writer is live.
        self._thread.join(3)
        if self._thread.is_alive():
            return
        with self._files_lock:
            for handler in self._handlers.values():
                handler.close()
            self.manifest.update(closed=True, health=self.health(),
                                 ended_utc=datetime.now(timezone.utc).isoformat())
            try:
                self._write_manifest()
            except OSError:
                pass

    def _prune_closed(self, keep):
        # Restrict deletion to our CLOSED session manifests and known log files.
        try:
            root = self.directory.parent.resolve()
            candidates = []
            for path in root.iterdir():
                if (path.is_symlink() or not path.is_dir() or path.resolve().parent != root
                        or not re.fullmatch(r"\d{8}T\d{6}-[0-9a-f]{10}", path.name)):
                    continue
                try:
                    meta = json.loads((path / "session.json").read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    continue
                if meta.get("schema") == SCHEMA and meta.get("closed") is True:
                    candidates.append(path)
            for path in sorted(candidates, reverse=True)[max(0, keep):]:
                children = list(path.iterdir())
                if not all(p.is_file() and not p.is_symlink() and p.resolve().parent == path.resolve()
                           and re.fullmatch(LOG_FILE_PATTERN, p.name)
                           for p in children):
                    continue
                for child in children:
                    child.unlink()
                path.rmdir()
        except OSError as exc:
            self.exception("gui", "retention.error", exc)


def start_diagnostics(project_dir):
    key = str(Path(project_dir).resolve())
    with _active_lock:
        current = _active.get(key)
        if current is None or current.health()["closed"]:
            current = DiagnosticRecorder(key, metadata=environment_summary(key))
            _active[key] = current
        return current


def install_exception_hooks(recorder, root=None):
    """Capture Python/Tk/background exceptions while preserving existing hooks."""
    if root is not None:
        previous = root.report_callback_exception
        def tk_error(kind, error, tb):
            recorder.exception("gui", "exception.tk", error.with_traceback(tb))
            previous(kind, error, tb)
        root.report_callback_exception = tk_error
        return
    previous_sys, previous_thread = sys.excepthook, threading.excepthook
    def uncaught(kind, error, tb):
        recorder.exception("gui", "exception.process", error.with_traceback(tb))
        recorder.flush()
        previous_sys(kind, error, tb)
    def thread_error(args):
        recorder.exception("gui", "exception.thread", args.exc_value.with_traceback(args.exc_traceback),
                           worker=args.thread.name if args.thread else None)
        previous_thread(args)
    sys.excepthook, threading.excepthook = uncaught, thread_error
