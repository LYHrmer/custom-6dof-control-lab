"""Build deterministic minimal wheel from immutable Git objects, never a checkout.

Algorithm files are byte-for-byte upstream. This is a dependency build recipe,
not a separately maintained fork. All writes occur in this project.
"""
import base64
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path("/home/lyh/robot-arm-compliant-control-lab")
COMMIT = "7aec01379ff9a8b135cbac75f18102eb9a27ea8f"
VERSION = "0.5.1+7aec01379ff9"
DIST = f"compliant_control_core-{VERSION}.dist-info"
FILES = ["__init__.py", "controllers.py", "franka_control.py"]


def git_blob(path):
    return subprocess.check_output(["rtk", "proxy", "git", "-C", str(SOURCE), "show", f"{COMMIT}:{path}"])


def main():
    members, sources = {}, {}
    for name in FILES:
        source = "src/compliant_control_lab/" + name
        blob = git_blob(source)
        members["compliant_control_lab/" + name] = blob
        sources[source] = hashlib.sha256(blob).hexdigest()
    licence = git_blob("LICENSE")
    members[f"{DIST}/LICENSE"] = licence
    sources["LICENSE"] = hashlib.sha256(licence).hexdigest()
    provenance = {"source_repository": str(SOURCE), "commit": COMMIT, "files_sha256": sources, "policy": "Unmodified selected modules; rebuild from Git object to update. Not editable upstream."}
    members[f"{DIST}/UPSTREAM.json"] = (json.dumps(provenance, indent=2) + "\n").encode()
    members[f"{DIST}/METADATA"] = (f"Metadata-Version: 2.1\nName: compliant-control-core\nVersion: {VERSION}\nSummary: Minimal unchanged controller modules from pinned compliant-control-lab\nLicense: MIT\nRequires-Python: >=3.10\nRequires-Dist: numpy>=1.24,<3\n\n").encode()
    members[f"{DIST}/WHEEL"] = b"Wheel-Version: 1.0\nGenerator: custom6dof-pinned-extractor\nRoot-Is-Purelib: true\nTag: py3-none-any\n"
    record = io.StringIO(newline="")
    writer = csv.writer(record)
    for name, content in sorted(members.items()):
        digest = base64.urlsafe_b64encode(hashlib.sha256(content).digest()).decode().rstrip("=")
        writer.writerow([name, "sha256=" + digest, len(content)])
    writer.writerow([f"{DIST}/RECORD", "", ""])
    members[f"{DIST}/RECORD"] = record.getvalue().encode()
    out = ROOT / "dependencies"
    out.mkdir(exist_ok=True)
    wheel = out / f"compliant_control_core-{VERSION}-py3-none-any.whl"
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, content in sorted(members.items()):
            info = zipfile.ZipInfo(name, (2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, content)
    data = buffer.getvalue()
    if wheel.exists() and wheel.read_bytes() != data:
        raise SystemExit("Pinned wheel already exists with different bytes; refusing overwrite")
    wheel.write_bytes(data)
    provenance["wheel_sha256"] = hashlib.sha256(data).hexdigest()
    (out / "upstream-lock.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(wheel)


if __name__ == "__main__":
    main()
