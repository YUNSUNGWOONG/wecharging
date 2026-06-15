"""OTA firmware update path with verification + rollback (Phase 4).

A staged, fail-safe firmware rollout for a fleet robot:

    IDLE -> DOWNLOADING -> VERIFYING -> STAGED -> ACTIVATING -> ACTIVE
                               |                       |
                               v (bad checksum)        v (health check fails)
                            FAILED                 ROLLED_BACK

The running ("known-good") version is only committed once the new image passes
its post-activation health check; a checksum mismatch or a failed health check
never changes it, so a bad update can't brick the robot. Pure-Python FSM; the
actual transport / flashing is a thin layer that drives these transitions.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class OtaState(IntEnum):
    IDLE = 0
    DOWNLOADING = 1
    VERIFYING = 2
    STAGED = 3
    ACTIVATING = 4
    ACTIVE = 5
    FAILED = 6          # download/verify failed; still on known-good version
    ROLLED_BACK = 7     # activated image failed health check; reverted


@dataclass(frozen=True)
class FirmwareImage:
    version: str
    sha256: str
    size_bytes: int = 0


@dataclass
class OtaUpdater:
    """Drives one robot's firmware update through the staged rollout."""
    current_version: str
    state: OtaState = OtaState.IDLE
    target: Optional[FirmwareImage] = None
    failure_reason: str = ""

    _IN_PROGRESS = (OtaState.DOWNLOADING, OtaState.VERIFYING,
                    OtaState.STAGED, OtaState.ACTIVATING)

    @property
    def in_progress(self) -> bool:
        return self.state in self._IN_PROGRESS

    def begin(self, image: FirmwareImage) -> None:
        """Start an update to `image`. Refuses if one is already running or the
        target equals the running version."""
        if self.in_progress:
            raise RuntimeError("an update is already in progress")
        if image.version == self.current_version:
            raise ValueError("target version equals the running version")
        self.target = image
        self.failure_reason = ""
        self.state = OtaState.DOWNLOADING

    def complete_download(self, actual_sha256: str) -> None:
        """Finish download; verify the checksum -> STAGED or FAILED."""
        if self.state != OtaState.DOWNLOADING:
            raise RuntimeError("not downloading")
        self.state = OtaState.VERIFYING
        if actual_sha256 == self.target.sha256:
            self.state = OtaState.STAGED
        else:
            self._fail("checksum mismatch")

    def activate(self) -> None:
        """Boot into the staged image (STAGED -> ACTIVATING)."""
        if self.state != OtaState.STAGED:
            raise RuntimeError("nothing staged to activate")
        self.state = OtaState.ACTIVATING

    def confirm_health(self, ok: bool) -> None:
        """Post-activation health check. Commit on success, roll back on fail."""
        if self.state != OtaState.ACTIVATING:
            raise RuntimeError("not activating")
        if ok:
            self.current_version = self.target.version   # commit known-good
            self.state = OtaState.ACTIVE
        else:
            self.failure_reason = "post-update health check failed"
            self.state = OtaState.ROLLED_BACK             # stays on prior version

    def abort(self, reason: str = "aborted") -> None:
        """Cancel an in-progress update; the running version is untouched."""
        if not self.in_progress:
            return
        self._fail(reason)

    def _fail(self, reason: str) -> None:
        self.failure_reason = reason
        self.state = OtaState.FAILED
