"""Audit checks that discriminate invalid inputs and corroborate source evidence."""

from copy import deepcopy
from pathlib import Path
import struct

import numpy as np
import pytest

from custom6dof.audit import (
    PROJECT_ROOT, chain_checks, duplicate_tool_check, inertia_checks,
    mujoco_probe, parse_urdf, read_stl, rotation,
)


@pytest.fixture(scope="module")
def source():
    path = next((PROJECT_ROOT / "inputs/original").glob("*/urdf/*.urdf"))
    return path, parse_urdf(path)


def test_urdf_fixed_axis_rotation_convention():
    roll = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    pitch = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    yaw = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    np.testing.assert_allclose(rotation([np.pi / 2] * 3), yaw @ pitch @ roll, atol=1e-15)


def test_spd_does_not_imply_physically_valid_inertia():
    checks = inertia_checks(np.diag([1., 1., 3.]))
    assert checks["positive_definite"]
    assert not checks["principal_triangle_inequality"]
    assert not inertia_checks(np.diag([-1., 1., 1.]))["positive_definite"]
    assert inertia_checks(np.diag([1., 1., 1.]))["principal_triangle_inequality"]


def test_corrupt_stl_count_is_rejected(tmp_path):
    path = tmp_path / "corrupt.stl"
    path.write_bytes(b"\x00" * 80 + struct.pack("<I", 2) + b"\x00" * 50)
    with pytest.raises(ValueError, match="Malformed"):
        read_stl(path)


def test_binary_stl_preserves_coordinates_even_with_solid_header(tmp_path):
    path = tmp_path / "one.stl"
    coordinates = [1., 2., 3., -1., 2., 3., 0., 4., 5.]
    triangle = struct.pack("<12fH", 0., 0., 1., *coordinates, 0)
    path.write_bytes(b"solid still binary".ljust(80, b" ") + struct.pack("<I", 1) + triangle)
    np.testing.assert_array_equal(read_stl(path), np.array(coordinates).reshape(1, 3, 3))


def test_chain_detects_cycle_and_duplicate_parent(source):
    _, model = source
    assert chain_checks(model)["serial_chain"]
    changed = deepcopy(model)
    changed["joints"]["Joint1"]["parent"] = "Link_tool"
    assert not chain_checks(changed)["valid_tree"]
    changed = deepcopy(model)
    changed["joints"]["extra"] = deepcopy(changed["joints"]["Joint_tool"])
    assert not chain_checks(changed)["valid_tree"]


def test_duplicate_requires_geometry_as_well_as_cad_name(source):
    path, model = source
    result = duplicate_tool_check(model, path.parent.parent, path.with_suffix(".csv"))
    assert result["duplicate_supported"]
    assert result["com_distance_m"] < 1e-8
    assert result["geometry"]["max_m"] < 1e-7
    assert 0.23 < result["inertia_relative_frobenius_difference"] < 0.24
    changed = deepcopy(model)
    changed["joints"]["Joint_tool"]["xyz_m"][0] += 0.001
    result = duplicate_tool_check(changed, path.parent.parent, path.with_suffix(".csv"))
    assert result["same_cad_instance_string"]
    assert not result["duplicate_supported"]
    assert result["recommended_model_correction"] is None


def test_preserved_models_compile_differently(source):
    path, _ = source
    original = mujoco_probe(path, path.parent.parent)
    assert original["available"] and not original["compiled"]
    assert "200000" in original["error"] and "base_link.STL" in original["error"]
    package = PROJECT_ROOT / "inputs/processed"
    processed = mujoco_probe(package / "urdf/custom_arm_tool4.urdf", package)
    assert processed["compiled"]
    assert (processed["nv"], processed["nu"], processed["nsensor"]) == (6, 0, 0)
