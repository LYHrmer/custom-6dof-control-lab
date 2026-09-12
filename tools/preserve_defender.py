"""Preserve the additional user-supplied code and video, without executing code."""
from pathlib import Path
import json
import shutil
import zipfile
import hashlib

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

dest = ROOT / "references" / "robot_defender"
if dest.exists():
    raise SystemExit("reference snapshot exists; refusing overwrite")
dest.mkdir(parents=True)
source = Path("/home/lyh/桌面/Robot-Defender.zip")
video = Path("/home/lyh/文档/xwechat_files/wxid_byi6dg2u66wi22_3837/temp/RWTemp/2026-09/9e20f478899dc29eb19741386f9343c8/9195cf48f3960f3d30e2a5b443b50c0c.mp4")
shutil.copy2(source, dest / source.name)
shutil.copy2(video, dest / "user_gravity_demo.mp4")
with zipfile.ZipFile(dest / source.name) as archive:
    for member in archive.infolist():
        if not (dest / member.filename).resolve().is_relative_to(dest.resolve()):
            raise ValueError("unsafe archive path")
    archive.extractall(dest)
files = {str(p.relative_to(dest)): sha256(p) for p in sorted(dest.rglob("*")) if p.is_file()}
(dest / "manifest.json").write_text(json.dumps({"sources": [str(source), str(video)], "validation_evidence": "User reports physical gravity compensation validation; recording not a calibrated torque measurement or firmware identity proof.", "sha256": files}, indent=2, ensure_ascii=False) + "\n")
for path in dest.rglob("*"):
    if path.is_file():
        path.chmod(0o444)
print(f"Preserved {len(files)} reference files")
