"""Pinned dependency/model provenance and non-overwriting clone restoration."""

import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import runpy
import zipfile

import pytest


ROOT = Path(__file__).resolve().parents[1]
UPSTREAM_COMMIT = "7aec01379ff9a8b135cbac75f18102eb9a27ea8f"
UPSTREAM_MODULES = {"__init__.py", "controllers.py", "franka_control.py"}


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def test_installed_controller_modules_match_pinned_source_bytes():
    lock = read_json(ROOT / "dependencies/upstream-lock.json")
    assert lock["commit"] == UPSTREAM_COMMIT
    entries = {name: sha for name, sha in lock["files_sha256"].items() if name.startswith("src/")}
    assert {Path(name).name for name in entries} == UPSTREAM_MODULES
    distribution = importlib.metadata.distribution("compliant-control-core")
    for name, expected in entries.items():
        relative = Path(name.removeprefix("src/"))
        installed = Path(distribution.locate_file(relative)).resolve()
        assert installed.is_relative_to((ROOT / ".venv").resolve()), installed
        module_name = ".".join(relative.with_suffix("").parts)
        if module_name.endswith(".__init__"):
            module_name = module_name.removesuffix(".__init__")
        module = importlib.import_module(module_name)
        assert Path(module.__file__).resolve() == installed, module_name
        assert digest(installed) == expected, name


def test_distributed_wheel_matches_lock_and_contains_only_selected_python_modules():
    lock = read_json(ROOT / "dependencies/upstream-lock.json")
    wheels = list((ROOT / "dependencies").glob("compliant_control_core-*.whl"))
    assert len(wheels) == 1
    assert digest(wheels[0]) == lock["wheel_sha256"]
    with zipfile.ZipFile(wheels[0]) as archive:
        python_files = {name for name in archive.namelist() if name.endswith(".py")}
        assert python_files == {f"compliant_control_lab/{name}" for name in UPSTREAM_MODULES}
        for name, expected in lock["files_sha256"].items():
            if name.startswith("src/"):
                assert hashlib.sha256(archive.read(name.removeprefix("src/"))).hexdigest() == expected


def test_preserved_primary_inputs_match_manifest():
    folder = ROOT / "inputs"
    manifest = read_json(folder / "manifest.json")["sha256"]
    assert "original.zip" in manifest
    for name, expected in manifest.items():
        path = folder / name
        assert path.is_file(), f"restore missing input: {name}"
        assert digest(path) == expected, name


@pytest.mark.parametrize("manifest_path", sorted((ROOT / "models").glob("*/manifest.json")),
                         ids=lambda path: path.parent.name)
def test_versioned_model_scene_and_preserved_source_hashes(manifest_path):
    manifest = read_json(manifest_path)
    assert manifest_path.parent.name == manifest["model_version"]
    assert digest(manifest_path.parent / "scene.xml") == manifest["scene_sha256"]
    inputs = {name: sha for name, sha in manifest["source_sha256"].items() if name.startswith("inputs/")}
    assert inputs
    for name, expected in inputs.items():
        assert digest(ROOT / name) == expected, name
    # Historical source code/config may evolve; frozen scenes and preserved inputs may not.
    config_snapshot = manifest_path.parent / "simulation_config.json"
    if config_snapshot.exists():
        assert digest(config_snapshot) == manifest["source_sha256"]["configs/simulation.json"]


def test_active_model_matches_current_config_and_generation_sources():
    config = read_json(ROOT / "configs/simulation.json")
    folder = ROOT / "models" / config["model_version"]
    manifest = read_json(folder / "manifest.json")
    assert read_json(folder / "simulation_config.json") == config
    for name, expected in manifest["source_sha256"].items():
        assert digest(ROOT / name) == expected, f"active model source changed without regeneration: {name}"


@pytest.fixture
def restore_function():
    # run_path avoids importing/caching an editable tools module as an installed package.
    return runpy.run_path(str(ROOT / "tools/restore_inputs.py"))["restore"]


def restoration_case(tmp_path, payloads):
    destination = tmp_path / "inputs"
    destination.mkdir()
    hashes = {name: hashlib.sha256(payload).hexdigest() for name, payload in payloads.items()}
    (destination / "manifest.json").write_text(json.dumps({"sha256": hashes}), encoding="utf-8")

    def prepare(stage):
        for name, payload in payloads.items():
            path = stage / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)

    return destination, prepare


def test_restore_adds_missing_verified_files_without_changing_matching_existing_work(tmp_path, restore_function):
    payloads = {"original.zip": b"archive bytes", "processed/asset.stl": b"mesh bytes"}
    destination, prepare = restoration_case(tmp_path, payloads)
    existing = destination / "original.zip"
    existing.write_bytes(payloads["original.zip"])
    before = existing.stat()
    restore_function(destination, prepare)
    after = existing.stat()
    assert (after.st_ino, after.st_mtime_ns, after.st_mode) == (before.st_ino, before.st_mtime_ns, before.st_mode)
    for name, payload in payloads.items():
        assert (destination / name).read_bytes() == payload
    assert destination.joinpath("processed/asset.stl").stat().st_mode & 0o222 == 0


def test_restore_checks_all_existing_work_before_writing_anything(tmp_path, restore_function):
    destination, prepare = restoration_case(tmp_path, {"first_missing": b"new", "existing": b"expected"})
    (destination / "existing").write_bytes(b"user edits")
    with pytest.raises(ValueError, match="refusing overwrite"):
        restore_function(destination, prepare)
    assert not (destination / "first_missing").exists()
    assert (destination / "existing").read_bytes() == b"user edits"


@pytest.mark.parametrize("symlink_parent", [False, True], ids=["dangling-file", "directory"])
def test_restore_rejects_symlinks_that_would_write_outside_destination(tmp_path, restore_function, symlink_parent):
    name = "linked/asset" if symlink_parent else "asset"
    destination, prepare = restoration_case(tmp_path, {name: b"expected"})
    outside = tmp_path / "outside"
    if symlink_parent:
        outside.mkdir()
        (destination / "linked").symlink_to(outside, target_is_directory=True)
        unintended_target = outside / "asset"
    else:
        (destination / name).symlink_to(outside)
        unintended_target = outside
    with pytest.raises(ValueError):
        restore_function(destination, prepare)
    assert not unintended_target.exists()
