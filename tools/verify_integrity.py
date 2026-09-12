"""Verify preserved inputs and the pinned controller, without modifying them."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def verify():
    errors = []
    count = 0
    skipped = []
    for folder in [ROOT / "inputs", ROOT / "references/robot_defender"]:
        manifest = json.loads((folder / "manifest.json").read_text())
        if folder.name == "robot_defender" and not any((folder / name).exists() for name in manifest["sha256"]):
            skipped.append("optional Robot-Defender reference not imported")
            continue
        for name, expected in manifest["sha256"].items():
            path = folder / name
            count += 1
            if not path.is_file() or digest(path) != expected:
                errors.append(str(path.relative_to(ROOT)))
    lock = json.loads((ROOT / "dependencies/upstream-lock.json").read_text())
    wheel = ROOT / "dependencies/compliant_control_core-0.5.1+7aec01379ff9-py3-none-any.whl"
    if digest(wheel) != lock["wheel_sha256"]:
        errors.append("upstream wheel")
    dist = importlib.metadata.distribution("compliant-control-core")
    for name, expected in lock["files_sha256"].items():
        if name.startswith("src/"):
            installed = Path(dist.locate_file(name.removeprefix("src/")))
            if not installed.is_relative_to(ROOT / ".venv") or digest(installed) != expected:
                errors.append("installed " + name)
    print(json.dumps({"preserved_files_checked": count, "passed": not errors, "errors": errors, "skipped": skipped}, indent=2))
    return not errors


if __name__ == "__main__":
    raise SystemExit(0 if verify() else 1)
