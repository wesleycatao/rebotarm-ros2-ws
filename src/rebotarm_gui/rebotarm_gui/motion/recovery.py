"""Durable execution checkpoint used by guided recipe recovery."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path


def default_checkpoint_path() -> Path:
    base = os.environ.get("XDG_STATE_HOME")
    root = Path(base).expanduser() if base else Path.home() / ".local" / "state"
    return root / "rebotarm_gui" / "recovery" / "checkpoint.json"


@dataclass
class RecoveryCheckpoint:
    program_path: str
    program_hash: str
    root_step_index: int
    opcode_index: int
    step_id: str
    reason: str
    run_id: str


class RecoveryStore:
    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path).expanduser() if path else default_checkpoint_path()

    def save(self, checkpoint: RecoveryCheckpoint) -> None:
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        payload = json.dumps(asdict(checkpoint), indent=2, sort_keys=True)
        with temporary.open("w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.path)
        directory_fd = os.open(self.path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)

    def load(self) -> RecoveryCheckpoint | None:
        if not self.path.is_file():
            return None
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            return RecoveryCheckpoint(**data)
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            return None

    def clear(self) -> None:
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
