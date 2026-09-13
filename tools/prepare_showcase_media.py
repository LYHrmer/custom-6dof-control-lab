"""Copy the verified original video and extract actual frames for local presentation.

Run with a Python that provides OpenCV. This does not change the simulation venv,
the reference video, or any original project. No generated motion or metrics.
"""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def main():
    import cv2
    source = ROOT / "references/robot_defender/user_gravity_demo.mp4"
    source_manifest = json.loads((source.parent / "manifest.json").read_text())
    expected = source_manifest["sha256"][source.name]
    if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
        raise ValueError("reference video differs from preserved input")
    folder = ROOT / "showcase/assets/media"
    folder.mkdir(parents=True, exist_ok=True)
    target = folder / "gravity-demo.mp4"
    if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() != expected:
        raise ValueError("different existing video; refusing overwrite")
    if not target.exists():
        shutil.copy2(source, target)
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError("cannot decode supplied video")
    fps = capture.get(cv2.CAP_PROP_FPS)
    report = {"source_sha256": expected, "duration_s": capture.get(cv2.CAP_PROP_FRAME_COUNT) / fps,
              "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
              "frames": {}, "note": "Original footage; extracted frames. No robot telemetry or measured performance data."}
    try:
        for second in (2, 7, 12, 17):
            capture.set(cv2.CAP_PROP_POS_MSEC, second * 1000)
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError(f"cannot decode frame at {second}s")
            filename = f"frame-{second:02d}.jpg"
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                raise RuntimeError("cannot encode poster")
            payload = encoded.tobytes()
            poster = folder / filename
            if poster.exists() and poster.read_bytes() != payload:
                raise ValueError(f"different existing frame; refusing overwrite: {filename}")
            if not poster.exists():
                poster.write_bytes(payload)
            report["frames"][filename] = {"requested_time_s": second, "sha256": hashlib.sha256(payload).hexdigest()}
    finally:
        capture.release()
    (folder / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
