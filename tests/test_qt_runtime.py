import sys

import pytest

from gello_hil.qt_runtime import QtRuntimeError, validate_versions, verify_qt_runtime


@pytest.mark.parametrize("python", [(3, 10, 14), (3, 11, 9)])
def test_known_none_dealloc_runtime_is_rejected_before_widget_creation(python):
    with pytest.raises(QtRuntimeError, match="none_dealloc"):
        validate_versions(python, "6.12.0", "6.12.0")


def test_mixed_pyside_shiboken_is_rejected():
    with pytest.raises(QtRuntimeError, match="Mixed Qt"):
        validate_versions((3, 10, 14), "6.8.3", "6.12.0")


def test_validated_runtime_has_no_cumulative_none_loss():
    pytest.importorskip("PySide6")
    from PySide6 import QtCore

    result = verify_qt_runtime()
    assert result["none_refcount_delta"] >= -4
    probe = QtCore.QObject()
    probe.setObjectName("stress")
    before = sys.getrefcount(None)
    for _ in range(100_000):
        probe.setObjectName("stress")
    assert sys.getrefcount(None) - before >= -4


def test_ownership_probe_rejects_a_future_reference_count_regression(monkeypatch):
    pytest.importorskip("PySide6")
    import gello_hil.qt_runtime as runtime

    counter = iter([2000, 1936])
    monkeypatch.setattr(runtime.sys, "getrefcount", lambda value: next(counter))
    with pytest.raises(QtRuntimeError, match="lost 64 references"):
        verify_qt_runtime()
