"""OTA firmware update path tests (Phase 4)."""
import pytest

from skyvolt_safety.ota import OtaUpdater, OtaState, FirmwareImage

IMG = FirmwareImage(version="1.2.0", sha256="abc123", size_bytes=4096)


def _staged():
    u = OtaUpdater(current_version="1.1.0")
    u.begin(IMG)
    u.complete_download("abc123")
    return u


def test_happy_path_commits_new_version():
    u = OtaUpdater(current_version="1.1.0")
    u.begin(IMG)
    assert u.state == OtaState.DOWNLOADING
    u.complete_download("abc123")
    assert u.state == OtaState.STAGED
    u.activate()
    assert u.state == OtaState.ACTIVATING
    u.confirm_health(True)
    assert u.state == OtaState.ACTIVE
    assert u.current_version == "1.2.0"      # committed


def test_checksum_mismatch_fails_and_keeps_version():
    u = OtaUpdater(current_version="1.1.0")
    u.begin(IMG)
    u.complete_download("WRONG")
    assert u.state == OtaState.FAILED
    assert "checksum" in u.failure_reason
    assert u.current_version == "1.1.0"      # unchanged


def test_failed_health_check_rolls_back():
    u = _staged()
    u.activate()
    u.confirm_health(False)
    assert u.state == OtaState.ROLLED_BACK
    assert u.current_version == "1.1.0"      # reverted to known-good
    assert "health" in u.failure_reason


def test_begin_same_version_rejected():
    u = OtaUpdater(current_version="1.2.0")
    with pytest.raises(ValueError):
        u.begin(IMG)


def test_begin_while_in_progress_rejected():
    u = OtaUpdater(current_version="1.1.0")
    u.begin(IMG)
    with pytest.raises(RuntimeError):
        u.begin(FirmwareImage(version="1.3.0", sha256="def"))


def test_invalid_transitions_raise():
    u = OtaUpdater(current_version="1.1.0")
    with pytest.raises(RuntimeError):
        u.activate()                  # nothing staged
    with pytest.raises(RuntimeError):
        u.confirm_health(True)        # not activating
    u.begin(IMG)
    with pytest.raises(RuntimeError):
        u.activate()                  # still downloading


def test_abort_cancels_and_keeps_version():
    u = OtaUpdater(current_version="1.1.0")
    u.begin(IMG)
    u.abort("operator cancelled")
    assert u.state == OtaState.FAILED
    assert u.current_version == "1.1.0"
    assert u.failure_reason == "operator cancelled"


def test_can_retry_after_failure():
    u = OtaUpdater(current_version="1.1.0")
    u.begin(IMG)
    u.complete_download("WRONG")          # FAILED
    assert not u.in_progress
    u.begin(IMG)                          # retry allowed
    u.complete_download("abc123")
    u.activate()
    u.confirm_health(True)
    assert u.current_version == "1.2.0"


def test_in_progress_flag_tracks_active_phases():
    u = OtaUpdater(current_version="1.1.0")
    assert not u.in_progress
    u.begin(IMG)
    assert u.in_progress
    u.complete_download("abc123")
    assert u.in_progress                  # STAGED counts as in-progress
    u.activate(); u.confirm_health(True)
    assert not u.in_progress              # ACTIVE
