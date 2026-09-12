#!/usr/bin/env python3
"""Compare the archived six-axis C++ gravity model offline; never board I/O.

The fitted header targets the ORIGINAL model, including its duplicate tool
mass. Report that parity separately from the corrected project's MuJoCo model.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from custom6dof.kinematics import ORIGINAL_URDF, URDFChain  # noqa: E402

REFERENCE = ROOT / "references/robot_defender/Robot-Defender"
GRAVITY = REFERENCE / "zephyr_app/src/model/arm_gravity"
HARNESS = ROOT / "tests/fixtures/defender_gravity_host.cpp"
SCENE = ROOT / "models" / json.loads((ROOT / "configs/simulation.json").read_text())["model_version"] / "scene.xml"
SAMPLES = REFERENCE / "scripts/mujoco_gravity_fit/data/parity_samples.csv"
FLOAT_GATE_NM = 5e-5


def compile_host(output: Path) -> Path:
    if shutil.which("g++") is None:
        raise RuntimeError("g++ is required for the optional reference parity check")
    required = [GRAVITY / "src/arm_gravity.cpp", GRAVITY / "inc/arm_gravity.h",
                GRAVITY / "inc/arm_gravity_fitted_model.h", HARNESS]
    if any(not path.is_file() for path in required):
        raise FileNotFoundError("import Robot-Defender.zip before the optional reference check")
    subprocess.run(["rtk", "proxy", "g++", "-std=c++17", "-O2", "-Wall", "-Wextra",
                    "-I", str(GRAVITY / "inc"), str(HARNESS), str(required[0]),
                    "-o", str(output)], check=True, capture_output=True, text=True)
    return output


def host_torques(binary: Path, q, up) -> np.ndarray:
    q, up = np.asarray(q, dtype=float), np.asarray(up, dtype=float)
    if q.ndim != 2 or q.shape[1] != 6 or q.shape[0] == 0 or up.shape != (len(q), 3):
        raise ValueError("expected nonempty q (n,6) and up (n,3)")
    if not np.isfinite(q).all() or not np.isfinite(up).all():
        raise ValueError("samples must be finite")
    if np.any(np.max(np.abs(up), axis=1) == 0):
        raise ValueError("up vector must be nonzero")
    payload = "".join(" ".join(f"{x:.17g}" for x in row) + "\n"
                      for row in np.column_stack((up, q)))
    result = subprocess.run(["rtk", "proxy", str(binary)], input=payload, text=True,
                            capture_output=True, check=True)
    values = np.fromstring(result.stdout, sep=" ")
    if values.size != q.size or not np.isfinite(values).all():
        raise RuntimeError("host output is malformed or nonfinite")
    return values.reshape(q.shape)


def potential_gradient(chain: URDFChain, q, up) -> np.ndarray:
    """Independent potential-energy finite difference, including tilted gravity."""
    gravity = -9.81 * np.asarray(up) / np.linalg.norm(up)
    eps = 1e-6
    return np.array([(chain.potential(q + e, gravity) - chain.potential(q - e, gravity))
                     / (2 * eps) for e in np.eye(6) * eps])


def evaluate(samples: int = 128, seed: int = 20260912) -> dict:
    if samples < 32:
        raise ValueError("at least 32 independent samples are required")
    original = URDFChain(deduplicate_tool=False)
    corrected = URDFChain(deduplicate_tool=True)
    rng = np.random.default_rng(seed)
    revolute = [j for j in original.joints if j.get("type") == "revolute"]
    lower = np.array([float(j.find("limit").get("lower")) for j in revolute]) + .24
    upper = np.array([float(j.find("limit").get("upper")) for j in revolute]) - .24
    q = rng.uniform(lower, upper, size=(samples, 6))
    up = rng.normal(size=(samples, 3))
    up /= np.linalg.norm(up, axis=1)[:, None]
    shipped = np.loadtxt(SAMPLES, delimiter=",", skiprows=1, ndmin=2)
    if shipped.shape[1] != 15 or len(shipped) < 32 or not np.isfinite(shipped).all():
        raise ValueError("archived parity CSV is malformed")
    with tempfile.TemporaryDirectory(prefix="custom6dof-defender-") as temporary:
        binary = compile_host(Path(temporary) / "gravity_host")
        predicted = host_torques(binary, q, up)
        shipped_predicted = host_torques(binary, shipped[:, 3:9], shipped[:, :3])
    original_tau = np.array([potential_gradient(original, qi, ui) for qi, ui in zip(q, up)])
    corrected_tau = np.array([potential_gradient(corrected, qi, ui) for qi, ui in zip(q, up)])
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    if model.nv != 6 or model.nq != 6:
        raise ValueError("corrected model must have six generalized coordinates")
    qadr = [model.joint(name).qposadr[0] for name in original.names]
    vadr = [model.joint(name).dofadr[0] for name in original.names]
    mujoco_tau = []
    for qi, ui in zip(q, up):
        data.qpos[qadr] = qi
        data.qvel[:] = 0
        model.opt.gravity[:] = -9.81 * ui
        mujoco.mj_forward(model, data)
        mujoco_tau.append(data.qfrc_bias[vadr].copy())
    maximum = lambda error: float(np.max(np.abs(error)))
    metrics = {
        "archived_csv_cpp_error_max_nm": maximum(shipped_predicted - shipped[:, 9:15]),
        "original_urdf_cpp_error_max_nm": maximum(predicted - original_tau),
        "corrected_urdf_mujoco_error_max_nm": maximum(np.array(mujoco_tau) - corrected_tau),
        "old_fitted_cpp_vs_corrected_model_max_nm": maximum(predicted - corrected_tau),
        "removed_duplicate_gravity_max_per_joint_nm": np.max(np.abs(original_tau - corrected_tau), axis=0).tolist(),
    }
    passed = (metrics["archived_csv_cpp_error_max_nm"] < FLOAT_GATE_NM
              and metrics["original_urdf_cpp_error_max_nm"] < FLOAT_GATE_NM
              and metrics["corrected_urdf_mujoco_error_max_nm"] < 1e-7)
    source_files = [ORIGINAL_URDF, SCENE, HARNESS, Path(__file__).resolve(), ROOT / "src/custom6dof/kinematics.py", GRAVITY / "src/arm_gravity.cpp",
                    GRAVITY / "inc/arm_gravity.h", GRAVITY / "inc/arm_gravity_fitted_model.h", SAMPLES]
    return {
        "passed": passed, "seed": seed, "independent_samples": samples,
        "archived_samples": len(shipped), "gravity_directions": "uniform sphere, including inverted base",
        "metrics": metrics,
        "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in source_files},
        "interpretation": "Offline implementation parity only. Old fitted parameters retain the duplicate CAD tool mass; do not use them for the corrected model or as hardware-identified parameters.",
        "hardware_io": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=128)
    parser.add_argument("--seed", type=int, default=20260912)
    parser.add_argument("--output", type=Path, default=ROOT / "reports/defender_parity.json")
    args = parser.parse_args()
    result = evaluate(args.samples, args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"passed": result["passed"], "metrics": result["metrics"],
                      "report": str(args.output)}, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
