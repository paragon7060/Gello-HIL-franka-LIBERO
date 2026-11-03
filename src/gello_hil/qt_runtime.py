"""Reject Qt bindings with unsafe Python singleton ownership before opening a device."""

from __future__ import annotations

import platform
import sys


class QtRuntimeError(RuntimeError):
    pass


def validate_versions(python_version, pyside_version, shiboken_version):
    remedy = "Install the GUI extra: python -m pip install -e '.[gui]' (PySide6-Essentials 6.8.3)."
    if pyside_version != shiboken_version:
        raise QtRuntimeError(
            f"Mixed Qt bindings: PySide6 {pyside_version}, Shiboken {shiboken_version}. {remedy}"
        )
    if tuple(python_version[:2]) < (3, 12) and pyside_version.split("+")[0] == "6.12.0":
        raise QtRuntimeError(
            "Unsafe Qt runtime: PySide6 6.12.0 loses None references on Python 3.10/3.11 "
            f"and eventually aborts with none_dealloc. {remedy}"
        )


def verify_qt_runtime():
    """Check versions and actual void-call ownership, without a window or serial bus."""
    from PySide6 import QtCore
    import shiboken6

    validate_versions(sys.version_info, QtCore.__version__, shiboken6.__version__)
    probe = QtCore.QObject()
    probe.setObjectName("gello-runtime-check")  # warm up conversion caches
    before = sys.getrefcount(None)
    for _ in range(64):
        probe.setObjectName("gello-runtime-check")
    after = sys.getrefcount(None)
    delta = after - before
    if delta < -4:
        raise QtRuntimeError(
            f"Unsafe Qt binding ownership: None lost {-delta} references in 64 calls. "
            "Reinstall matched PySide6-Essentials/shiboken6 6.8.3 before collecting."
        )
    return {
        "python": platform.python_version(),
        "pyside": QtCore.__version__,
        "shiboken": shiboken6.__version__,
        "void_call_samples": 64,
        "none_refcount_delta": delta,
    }
