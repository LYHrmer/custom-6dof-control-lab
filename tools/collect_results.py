"""Collect existing completed runs into a compact, trace-hashed acceptance report."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    results = {}
    for name in ("gravity", "impedance", "perturb"):
        folder = ROOT / "reports/runs" / name
        result = json.loads((folder / "summary.json").read_text())
        if not result["passed"] or not result["acceptance_complete"]:
            raise ValueError(f"incomplete acceptance: {name}")
        current_config = digest(ROOT / "configs/simulation.json")
        if result["config_sha256"] != current_config:
            raise ValueError(f"stale config: {name}")
        for source, expected in result["source_sha256"].items():
            if digest(ROOT / source) != expected:
                raise ValueError(f"stale source {source}: {name}")
        result["trace_sha256"] = digest(folder / "trace.csv")
        results[name] = result
    before_path, after_path = ROOT / "reports/upstream_before.json", ROOT / "reports/upstream_after.json"
    before, after = json.loads(before_path.read_text()), json.loads(after_path.read_text())
    changed = [key for key in before.keys() | after.keys() if before.get(key) != after.get(key)]
    runtime_prefixes = ("src/", ".venv/", ".local-deps/", "results/", "dist/", "build/", "cpp/", "cmake/")
    runtime_changes = [key for key in changed if key in before and key.startswith(runtime_prefixes)]
    new_runtime_paths = [key for key in changed if key not in before and key.startswith(runtime_prefixes)]
    baseline_code_changes = [key for key in changed if key in before and key.startswith(("tests/", "tools/"))]
    junit = ET.parse(ROOT / "reports/pytest.xml").getroot()
    suites = [junit] if junit.tag == "testsuite" else list(junit)
    test_summary = {name: sum(int(suite.get(name, 0)) for suite in suites) for name in ("tests", "failures", "errors", "skipped")}
    report = {
        "schema_version": 1, "scope": "free-space numerical implementation validation only",
        "passed": not runtime_changes and not baseline_code_changes and not test_summary["failures"] and not test_summary["errors"],
        "environment": {"python": platform.python_version(), "platform": platform.platform(), **{name: importlib.metadata.version(name) for name in ("numpy", "scipy", "mujoco", "pytest", "compliant-control-core")}},
        "tests": test_summary, "upstream": {"baseline_entries": len(before), "after_entries": len(after), "full_tree_unchanged": not changed, "changed": sorted(changed), "existing_runtime_assets_unchanged": not runtime_changes, "runtime_changes": runtime_changes, "new_runtime_paths": new_runtime_paths, "existing_test_tool_code_unchanged": not baseline_code_changes, "existing_test_tool_changes": baseline_code_changes, "observation": "Concurrent workspace changes were observed. This project used a pinned wheel and did not run upstream tools/tests. Full-tree immutability is NOT claimed. Existing runtime assets means baseline src, environment, results, dist, build, cpp and cmake files; newly appearing outputs are listed separately. All differences are reported, not reverted.", "before_manifest_sha256": digest(before_path), "after_manifest_sha256": digest(after_path)},
        "runs": results,
    }
    target = ROOT / "reports/phase1_results.json"
    target.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": report["passed"], "tests": test_summary, "upstream_unchanged": not changed, "report": str(target)}, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
