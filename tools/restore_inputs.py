"""Restore omitted local inputs into a clone, matching tracked manifests exactly."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def extract(source, target):
    with zipfile.ZipFile(source) as archive:
        for member in archive.infolist():
            if not (target / member.filename).resolve().is_relative_to(target.resolve()):
                raise ValueError("unsafe ZIP path")
        archive.extractall(target)


def restore(destination, prepare):
    manifest = json.loads((destination / "manifest.json").read_text())["sha256"]
    destination = destination.absolute()
    for name in manifest:
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"unsafe manifest path: {name}")
        target = destination / relative
        if any(path.is_symlink() for path in [target, *target.parents]):
            raise ValueError(f"symlink in restore target; refusing: {target}")
        if not target.resolve().is_relative_to(destination.resolve()):
            raise ValueError(f"restore target escapes destination: {target}")
    with tempfile.TemporaryDirectory(prefix="custom6dof-restore-") as directory:
        stage = Path(directory)
        prepare(stage)
        # Validate everything, including existing files, before restoring anything.
        for name, expected in manifest.items():
            proposed = stage / name
            existing = destination / name
            if not proposed.is_file() or sha256(proposed) != expected:
                raise ValueError(f"source hash mismatch: {name}")
            if existing.exists() and (not existing.is_file() or sha256(existing) != expected):
                raise ValueError(f"existing work differs; refusing overwrite: {existing}")
        for name in manifest:
            target = destination / name
            if not target.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(stage / name, target)
                target.chmod(0o444)
    print(f"Restored/verified {len(manifest)} files under {destination}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zip", type=Path, default=Path("/home/lyh/桌面/自定义控制器3.0-tool-4.zip"))
    parser.add_argument("--processed", type=Path, default=Path("/home/lyh/桌面/custom_arm_tool4"))
    parser.add_argument("--advice", type=Path, default=Path("/home/lyh/桌面/URDF处理与项目改进建议.md"))
    parser.add_argument("--defender", type=Path)
    parser.add_argument("--video", type=Path)
    args = parser.parse_args()
    if bool(args.defender) != bool(args.video):
        parser.error("--defender and --video must be supplied together")
    def prepare_inputs(stage):
        shutil.copy2(args.zip, stage / "original.zip")
        shutil.copy2(args.advice, stage / "claude_advice.md")
        shutil.copytree(args.processed, stage / "processed")
        extract(stage / "original.zip", stage / "original")
    restore(ROOT / "inputs", prepare_inputs)
    if args.defender:
        def prepare_defender(stage):
            shutil.copy2(args.defender, stage / "Robot-Defender.zip")
            shutil.copy2(args.video, stage / "user_gravity_demo.mp4")
            extract(stage / "Robot-Defender.zip", stage)
        restore(ROOT / "references/robot_defender", prepare_defender)


if __name__ == "__main__":
    main()
