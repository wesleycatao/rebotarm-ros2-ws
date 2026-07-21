"""Persistent, append-only execution audit trail.

The GUI writes one JSON object per line.  JSONL keeps individual records
recoverable if a process is interrupted while appending the final line and is
straightforward to archive or feed into external compliance tooling.
"""

from __future__ import annotations

import csv
import json
import os
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


AUDIT_SCHEMA_VERSION = 1
CSV_COLUMNS = (
    "schema_version",
    "event_id",
    "run_id",
    "event",
    "timestamp_utc",
    "started_at_utc",
    "finished_at_utc",
    "operator",
    "program",
    "program_version",
    "program_hash",
    "mode",
    "start_index",
    "step_sequence",
    "step_index",
    "step_id",
    "step_type",
    "parameters",
    "result",
    "duration_ms",
    "message",
    "cancel_source",
)


def utc_now_iso() -> str:
    """Return a millisecond-resolution UTC timestamp."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def default_audit_path() -> Path:
    """Return the per-user execution audit path following XDG conventions."""
    base = os.environ.get("XDG_STATE_HOME")
    root = Path(base).expanduser() if base else Path.home() / ".local" / "state"
    return root / "rebotarm_gui" / "audit" / "execution.jsonl"


def new_run_id() -> str:
    return str(uuid.uuid4())


class ExecutionAuditLog:
    """Append execution events durably and export them as CSV.

    Write failures are reported through ``last_error`` and a false return
    value.  The motion runner can therefore fail closed before moving when the
    audit destination is unavailable.
    """

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path).expanduser() if path is not None else default_audit_path()
        self.last_error = ""
        self._lock = threading.RLock()

    def append(self, event: str, *, run_id: str, **fields: Any) -> bool:
        record = {
            "schema_version": AUDIT_SCHEMA_VERSION,
            "event_id": str(uuid.uuid4()),
            "run_id": str(run_id),
            "event": str(event),
            "timestamp_utc": utc_now_iso(),
        }
        record.update(fields)
        try:
            line = json.dumps(
                record,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            with self._lock:
                self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                with self.path.open("a", encoding="utf-8") as stream:
                    stream.write(line + "\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                try:
                    os.chmod(self.path, 0o600)
                except OSError:
                    pass
            return True
        except (OSError, TypeError, ValueError) as exc:
            self.last_error = str(exc)
            return False

    def iter_records(self) -> Iterable[dict[str, Any]]:
        """Yield valid records; an interrupted/malformed line is skipped."""
        if not self.path.is_file():
            return
        with self._lock:
            try:
                with self.path.open("r", encoding="utf-8") as stream:
                    for line in stream:
                        try:
                            record = json.loads(line)
                        except (json.JSONDecodeError, TypeError):
                            continue
                        if isinstance(record, dict):
                            yield record
            except OSError as exc:
                self.last_error = str(exc)
                return

    def export_csv(self, destination: str | Path) -> int:
        """Export all valid JSONL records and return the written row count."""
        target = Path(destination).expanduser()
        if target.resolve() == self.path.resolve():
            raise ValueError("o CSV de destino não pode sobrescrever o JSONL de auditoria")
        target.parent.mkdir(parents=True, exist_ok=True)
        count = 0
        with target.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, extrasaction="ignore")
            writer.writeheader()
            for record in self.iter_records():
                row = dict(record)
                parameters = row.get("parameters")
                if isinstance(parameters, (dict, list)):
                    row["parameters"] = json.dumps(
                        parameters, ensure_ascii=False, sort_keys=True, separators=(",", ":")
                    )
                writer.writerow({column: row.get(column, "") for column in CSV_COLUMNS})
                count += 1
        try:
            os.chmod(target, 0o600)
        except OSError:
            pass
        return count
