"""One-time immutable input snapshot, with hashes. Refuses an existing snapshot."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    dest = ROOT / "inputs"
    if dest.exists():
        raise SystemExit("inputs already exists; refusing to overwrite")
    dest.mkdir()
    sources = {
        "original.zip": Path("/home/lyh/桌面/自定义控制器3.0-tool-4.zip"),
        "claude_advice.md": Path("/home/lyh/桌面/URDF处理与项目改进建议.md"),
        "processed": Path("/home/lyh/桌面/custom_arm_tool4"),
    }
    for name, source in sources.items():
        if source.is_dir():
            shutil.copytree(source, dest / name)
        else:
            shutil.copy2(source, dest / name)
    extracted = dest / "original"
    extracted.mkdir()
    with zipfile.ZipFile(dest / "original.zip") as archive:
        for member in archive.infolist():
            target = (extracted / member.filename).resolve()
            if not target.is_relative_to(extracted.resolve()):
                raise ValueError("unsafe archive path")
        archive.extractall(extracted)
    manifest = {str(p.relative_to(dest)): sha256(p) for p in sorted(dest.rglob("*")) if p.is_file()}
    (dest / "manifest.json").write_text(json.dumps({"sources": {k: str(v) for k, v in sources.items()}, "sha256": manifest}, indent=2, ensure_ascii=False) + "\n")
    for path in dest.rglob("*"):
        if path.is_file():
            path.chmod(0o444)
    print(f"Preserved {len(manifest)} files in {dest}")


if __name__ == "__main__":
    main()
