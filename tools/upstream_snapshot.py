"""Hash all upstream regular files, including environment and frozen results."""
import argparse
import hashlib
import json
from pathlib import Path

SOURCE = Path("/home/lyh/robot-arm-compliant-control-lab")


def snapshot():
    files = {}
    for path in sorted(SOURCE.rglob("*")):
        if path.is_symlink():
            files[str(path.relative_to(SOURCE))] = "symlink:" + str(path.readlink())
        elif path.is_file():
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            files[str(path.relative_to(SOURCE))] = digest.hexdigest()
    return files


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--compare", type=Path)
    args = parser.parse_args()
    current = snapshot()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(current, indent=2) + "\n")
    print(f"Hashed {len(current)} upstream entries")
    if args.compare:
        before = json.loads(args.compare.read_text())
        changed = [name for name in sorted(before.keys() | current.keys()) if before.get(name) != current.get(name)]
        print(json.dumps({"unchanged": not changed, "changed": changed}, indent=2))
        raise SystemExit(bool(changed))
