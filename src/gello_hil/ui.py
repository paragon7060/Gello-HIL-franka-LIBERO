"""Small OpenCV operator display used by mapping and collection."""

from __future__ import annotations

import textwrap

import cv2
import numpy as np

from .session import decode_key


def poll_command(window: str, delay=1):
    key = cv2.waitKey(delay)
    try:
        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            return "abort"
    except cv2.error:
        return "abort"
    return decode_key(key)


def wrap_pixels(text, width, *, font_scale=.58, thickness=1):
    """Wrap all text by rendered width so instructions never silently disappear."""
    lines, line = [], ""
    for word in str(text).split():
        proposed = (line + " " + word).strip()
        if line and cv2.getTextSize(proposed, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0][0] > width:
            lines.append(line)
            line = word
        else:
            line = proposed
        while cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0][0] > width:
            split = len(line)-1
            while split > 1 and cv2.getTextSize(line[:split], cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0][0] > width:
                split -= 1
            lines.append(line[:split])
            line = line[split:]
    return lines + [line]


def operator_panel(
    rgb_frame,
    *,
    instruction: str,
    status: str,
    active_steps: int,
    max_steps: int,
    mapping_mode: str,
    joints=None,
    errors=None,
    notice: str = "",
    scale: float = 2.0,
    controls: tuple[str, str] | None = None,
) -> np.ndarray:
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("window scale must be positive and finite")
    frame = cv2.cvtColor(np.asarray(rgb_frame, dtype=np.uint8), cv2.COLOR_RGB2BGR)
    frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    color = (0, 255, 0) if status == "ACTIVE" else (0, 220, 255)
    lines = [("GELLO HIL / FRANKA / LIBERO", (255, 255, 255), 0.72)]
    lines += [(line, (0, 255, 255), 0.62) for line in textwrap.wrap(instruction, 43)]
    lines += [
        (f"STATUS: {status}", color, 0.86),
        (f"ACTIVE STEPS: {active_steps}/{max_steps}", (255, 255, 255), 0.62),
        (f"MAPPING: {mapping_mode}", (255, 255, 255), 0.58),
        ((controls or ("ENTER resume/re-anchor | SPACE clutch", "q abort (attempt is still saved)"))[0], (220, 220, 220), 0.56),
        ((controls or ("ENTER resume/re-anchor | SPACE clutch", "q abort (attempt is still saved)"))[1], (220, 220, 220), 0.56),
    ]
    if joints is not None:
        values = np.asarray(joints)
        errs = np.zeros(7) if errors is None else np.asarray(errors)
        lines.append(("TARGET / ERROR (rad)", (255, 255, 255), 0.58))
        for index, (joint, error) in enumerate(zip(values, errs), start=1):
            joint_color = (0, 80, 255) if abs(error) >= 0.25 else (180, 255, 180)
            lines.append((f"J{index}: {joint:+.3f}   err {error:+.3f}", joint_color, 0.56))
    if notice:
        lines += [(line, (0, 120, 255), 0.54) for line in textwrap.wrap(notice, 48)]
    lines = [(part, color, size) for text, color, size in lines
             for part in wrap_pixels(text, 580, font_scale=size, thickness=2)]
    height = max(frame.shape[0], 36 + sum(32 if size < .7 else 40 for _, _, size in lines) + 12)
    panel = np.zeros((height, 620, 3), dtype=np.uint8)
    frame = np.pad(frame, ((0, height - frame.shape[0]), (0, 0), (0, 0)))
    y = 36
    for text, line_color, size in lines:
        cv2.putText(
            panel, text, (18, y), cv2.FONT_HERSHEY_SIMPLEX, size, line_color, 2, cv2.LINE_AA
        )
        y += 32 if size < 0.7 else 40
    return np.concatenate((frame, panel), axis=1)


def close_window(name: str) -> None:
    try:
        cv2.destroyWindow(name)
    except cv2.error:
        return
    cv2.waitKey(1)
    cv2.waitKey(1)
