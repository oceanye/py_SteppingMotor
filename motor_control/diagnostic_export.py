"""Allow-listed diagnostic bundles; no network, credentials, or arbitrary files."""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import re
import uuid
import zipfile

from .diagnostics import SCHEMA, environment_summary, sanitize, LOG_FILE_PATTERN


_LOG = re.compile(LOG_FILE_PATTERN)
_LEGACY = re.compile(r"gui_\d{4}-\d{2}-\d{2}\.log|gui_stderr\.log|raw_\d{8}_\d{6}\.log")
_SESSION = re.compile(r"\d{8}T\d{6}-[0-9a-f]{10}")


def _safe_file(path, root):
    return (path.is_file() and not path.is_symlink()
            and path.resolve().is_relative_to(root.resolve()))


def _tail(path, limit):
    with path.open("rb") as handle:
        size = path.stat().st_size
        handle.seek(max(0, size-limit))
        raw = handle.read(limit)
    if size > limit:
        raw = raw.partition(b"\n")[2]  # Skip a partial JSON/UTF-8 line.
    return raw.decode("utf-8", errors="replace"), size > limit


def _redacted_text(name, text):
    if ".jsonl" in name:
        lines = []
        for line in text.splitlines():
            try:
                item = sanitize(json.loads(line))
            except ValueError:
                item = {"event": "export.unparseable_line", "raw": sanitize(line)}
            lines.append(json.dumps(item, ensure_ascii=False, allow_nan=False))
        return "\n".join(lines) + "\n"
    if name.endswith(".json"):
        return json.dumps(sanitize(json.loads(text)), ensure_ascii=False, indent=2)
    return sanitize(text)


def export_bundle(project_dir, output=None, *, recorder=None, label="field-diagnostics",
                  max_sessions=3, max_file_bytes=1_000_000, max_total_bytes=24_000_000):
    """Snapshot recent logs, re-redact them, and produce ZIP + telemetry CSV.

    Legacy logs are labelled separately and may come from older app versions.
    Session timestamps and source hashes are retained for provenance. Limits and
    omitted/truncated files are explicit in manifest.json; no silent 'full log'
    claim. Raw project config files and the Git directory are never collected.
    max_total_bytes limits source logs; generated CSV and manifest are additional.
    """
    root = Path(project_dir).resolve()
    log_root = root / "logs"
    if not log_root.resolve().is_relative_to(root):
        raise ValueError("日志目录不能指向项目外部")
    if output is not None and Path(output).exists():
        raise FileExistsError(output)
    entries = {}
    excluded = []
    truncated = []
    remaining = max_total_bytes

    def add(name, text, was_truncated=False):
        nonlocal remaining
        content = _redacted_text(name, text).encode("utf-8")
        if len(content) > remaining:
            excluded.append(name)
            return
        entries[name] = content
        remaining -= len(content)
        if was_truncated:
            truncated.append(name)

    active_id = None
    if recorder is not None:
        active_id = recorder.session_id
        priority = lambda name: (0 if name == "session.json" else 1 if name.startswith("events.")
                                 else 2 if name.startswith("flight.") else 3, name)
        for name, content in sorted(recorder.snapshot_files().items(), key=lambda pair: priority(pair[0])):
            if _LOG.fullmatch(name):
                encoded = content.encode("utf-8")
                cut = len(encoded) > max_file_bytes and ".jsonl" in name
                if cut:
                    content = encoded[-max_file_bytes:].partition(b"\n")[2].decode("utf-8", errors="replace")
                add(f"sessions/{active_id}/{name}", content, cut)
    diagnostic_root = log_root / "diagnostics"
    if diagnostic_root.is_dir() and diagnostic_root.resolve().is_relative_to(log_root.resolve()):
        sessions = sorted((p for p in diagnostic_root.iterdir() if p.is_dir()
                           and not p.is_symlink() and _SESSION.fullmatch(p.name)
                           and p.resolve().parent == diagnostic_root.resolve()
                           and p.name != active_id), reverse=True)
        for directory in sessions[:max(0, max_sessions-(1 if active_id else 0))]:
            for path in sorted(directory.iterdir(), key=lambda p: (
                    0 if p.name == "session.json" else 1 if p.name.startswith("events.")
                    else 2 if p.name.startswith("flight.") else 3, p.name)):
                if _LOG.fullmatch(path.name) and _safe_file(path, log_root):
                    content, cut = _tail(path, max_file_bytes)
                    try:
                        add(f"sessions/{directory.name}/{path.name}", content, cut)
                    except ValueError:
                        excluded.append(f"sessions/{directory.name}/{path.name}")
    if log_root.is_dir():
        legacy = sorted((p for p in log_root.iterdir() if _LEGACY.fullmatch(p.name)
                         and _safe_file(p, log_root)), key=lambda p:p.stat().st_mtime, reverse=True)
        for path in legacy[:4]:
            content, cut = _tail(path, max_file_bytes)
            add(f"legacy/{path.name}", content, cut)

    # CSV is a convenient view of EXACT measured/commanded fields, not invented
    # encoder values. Unknown values remain empty; motor and output units differ.
    columns = ("session_id", "utc", "seq", "role", "driver_id", "motor_deg",
               "command_motor_deg", "output_position", "unit", "gear_ratio", "lead_mm",
               "error_motor_deg", "speed_motor_rpm", "current_a", "temperature_c",
               "online", "enabled", "reached", "fault", "age_ms")
    samples = []
    sample_ids = set()
    for name, content in entries.items():
        if not any(f"/{stream}.jsonl" in name for stream in ("telemetry", "flight", "events")):
            continue
        for line in content.decode("utf-8").splitlines():
            if not line.strip():
                continue
            record = json.loads(line)
            if isinstance(record, dict) and record.get("event") == "x42s.telemetry":
                identity = (record.get("session_id"), record.get("seq"))
                if identity in sample_ids:
                    continue
                sample_ids.add(identity)
                samples.append({**record.get("data", {}), **{k:record.get(k) for k in columns[:3]}})
    table = io.StringIO(newline="")
    writer = csv.DictWriter(table, fieldnames=columns, extrasaction="ignore")
    writer.writeheader()
    for sample in sorted(samples, key=lambda s:(s.get("utc") or "", s.get("seq") or 0)):
        writer.writerow({k:("'"+v if isinstance(v,str) and v[:1] in ("=","+","-","@") else v)
                         for k,v in sample.items()})
    entries["telemetry.csv"] = table.getvalue().encode("utf-8-sig")
    manifest = {"schema": "motor-diagnostic-bundle-v1", "created_utc": datetime.now(timezone.utc).isoformat(),
                "label": sanitize(label), "environment": environment_summary(root),
                "privacy": "Allow-listed logs only; credentials, user paths, IPs and email addresses redacted. Review free-text before publication.",
                "limits": {"max_sessions":max_sessions,"max_file_bytes":max_file_bytes,"max_total_bytes":max_total_bytes,
                           "total_budget_scope":"source logs; generated CSV/manifest additional"},
                "truncated_files":truncated,"omitted_files":excluded,"telemetry_rows":len(samples),
                "active_health": recorder.health() if recorder else None,
                "files": {name:{"bytes":len(data),"sha256":hashlib.sha256(data).hexdigest()} for name,data in entries.items()}}
    entries["manifest.json"] = json.dumps(sanitize(manifest), ensure_ascii=False, indent=2).encode("utf-8")
    if output is None:
        output = log_root / "exports" / ("diagnostics-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
                                          + "-" + uuid.uuid4().hex[:6] + ".zip")
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, mode="x", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(entries.items()):
            archive.writestr(name, data)
    return output
