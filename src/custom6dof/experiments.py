"""Repeatable free-space gravity hold and slow Cartesian impedance experiments."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from compliant_control_lab.franka_control import FrankaImpedanceController, FrankaTarget
from .adapter import ArmAdapter
from .kinematics import ROOT


def configuration():
    return json.loads((ROOT / "configs/simulation.json").read_text())


def controller(config):
    return FrankaImpedanceController(
        translational_stiffness=np.array(config["translational_stiffness_n_m"]),
        translational_damping=np.array(config["translational_damping_ns_m"]),
        rotational_stiffness=np.array(config["rotational_stiffness_nm_rad"]),
        rotational_damping=np.array(config["rotational_damping_nms_rad"]),
    )


def reference(t, start_position, start_rotation, config):
    duration = config["trajectory_ramp_s"]
    u = float(np.clip(t / duration, 0, 1))
    blend = 10*u**3 - 15*u**4 + 6*u**5
    rate = (30*u**2 - 60*u**3 + 30*u**4) / duration if 0 < t < duration else 0.0
    delta = np.array(config["trajectory_delta_m"])
    rotvec = np.array(config["trajectory_rotation_rad"])
    return FrankaTarget(start_position + blend * delta,
                        Rotation.from_rotvec(blend * rotvec).as_matrix() @ start_rotation,
                        rate * delta, rate * rotvec, 0.0)


def run(mode="impedance", duration_s=None, perturb=False, model_path=None):
    if mode not in {"gravity", "impedance"}:
        raise ValueError("only gravity and impedance supported")
    config = configuration()
    duration = config["duration_s"] if duration_s is None else float(duration_s)
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    actual_model_path = ROOT / "models" / config["model_version"] / "scene.xml" if model_path is None else Path(model_path)
    arm = ArmAdapter(actual_model_path)
    arm.reset(np.array(config["initial_q_rad"]))
    initial_q = arm.joint_position()
    p0, r0 = arm.pose()
    ctrl = controller(config)
    ctrl.reset(arm.state())
    diagnostic = arm.singularity_diagnostic(config["singularity_length_scale_m"])
    if diagnostic["scaled_singular_values"][-1] < 1e-3:
        raise ValueError("simulation start too close to singularity for this experiment")
    rows = []
    saturated_steps = 0
    dt = arm.model.opt.timestep
    for _ in range(int(np.ceil(duration / dt))):
        t = float(arm.data.time)
        target = reference(t, p0, r0, config) if mode == "impedance" else FrankaTarget(p0, r0, np.zeros(3), np.zeros(3), 0.0)
        tau = arm.gravity() if mode == "gravity" else arm.controller_torque(ctrl, target, np.array(config["joint_damping_nms_rad"]))
        # Evaluation perturbation only. Controller has no access to this value.
        external_tau = np.zeros(6)
        if perturb and 5 <= t < 5.25:
            external_tau[2] = 0.02
        arm.data.qfrc_applied[:] = 0
        arm.data.qfrc_applied[arm.dof_indices] = external_tau
        applied = arm.step(tau)
        saturated_steps += int(np.any(np.abs(applied - tau) > 1e-12))
        p, r = arm.pose()
        target_after = reference(arm.data.time, p0, r0, config) if mode == "impedance" else target
        sensor = arm.read_joint_sensors()
        row = {"time_s": float(arm.data.time),
               "position_error_m": float(np.linalg.norm(target_after.position - p)),
               "orientation_error_rad": float(Rotation.from_matrix(target_after.rotation @ r.T).magnitude()),
               "max_joint_speed_rad_s": float(np.max(np.abs(sensor["velocity_rad_s"]))),
               "saturated": int(np.any(np.abs(applied - tau) > 1e-12))}
        for i in range(6):
            row[f"sim_q_{i+1}_rad"] = float(sensor["position_rad"][i])
            row[f"sim_qd_{i+1}_rad_s"] = float(sensor["velocity_rad_s"][i])
            row[f"requested_tau_{i+1}_nm"] = float(tau[i])
            row[f"sim_actuator_tau_{i+1}_nm"] = float(sensor["actuator_torque_nm"][i])
            row[f"eval_external_tau_{i+1}_nm"] = float(external_tau[i])
        if not np.isfinite(list(row.values())).all():
            raise RuntimeError("non-finite simulation result")
        rows.append(row)
    minimum_duration = 2.0 if mode == "gravity" else config["trajectory_ramp_s"] + 4.0
    if perturb:
        minimum_duration = max(minimum_duration, 9.25)
    complete = duration >= minimum_duration
    summary = {"mode": mode, "perturbation": perturb, "duration_s": float(arm.data.time),
        "acceptance_complete": complete, "minimum_acceptance_duration_s": minimum_duration,
        "perturbation_applied_steps": sum(row["eval_external_tau_3_nm"] != 0 for row in rows),
        "steps": len(rows), "nq": arm.model.nq, "nv": arm.model.nv, "nu": arm.model.nu,
        "nsensor": arm.model.nsensor, "start_singularity": diagnostic,
        "final_position_error_m": rows[-1]["position_error_m"],
        "final_orientation_error_rad": rows[-1]["orientation_error_rad"],
        "peak_position_error_m": max(r["position_error_m"] for r in rows),
        "peak_joint_speed_rad_s": max(r["max_joint_speed_rad_s"] for r in rows),
        "final_joint_drift_rad": float(np.max(np.abs(arm.joint_position() - initial_q))),
        "saturated_steps": saturated_steps,
        "max_abs_requested_torque_nm": [max(abs(row[f"requested_tau_{i+1}_nm"]) for row in rows) for i in range(6)],
        "external_wrench_observation": None, "estimated_external_wrench": None,
        "normal_force_field": "0 compatibility placeholder unused by impedance, not a sensor",
        "evaluation_truth": "eval_external_tau_* known injected joint torque; never passed to controller",
        "scope": "ideal free-space simulation; not hardware validation or sensorless force estimation",
        "config_sha256": hashlib.sha256((ROOT / "configs/simulation.json").read_bytes()).hexdigest(),
        "model_sha256": hashlib.sha256(actual_model_path.read_bytes()).hexdigest(),
        "model_path": str(actual_model_path.relative_to(ROOT)) if actual_model_path.is_relative_to(ROOT) else str(actual_model_path),
        "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in [Path(__file__).resolve(), ROOT / "src/custom6dof/adapter.py", ROOT / "dependencies/upstream-lock.json"]},
        "upstream_commit": json.loads((ROOT / "dependencies/upstream-lock.json").read_text())["commit"],
    }
    summary["passed"] = (complete and (not perturb or summary["perturbation_applied_steps"] > 0) and saturated_steps == 0 and summary["peak_joint_speed_rad_s"] < config["velocity_acceptance_rad_s"] and
        ((mode == "gravity" and summary["final_joint_drift_rad"] < 1e-6) or
         (mode == "impedance" and summary["final_position_error_m"] < 0.0005 and summary["final_orientation_error_rad"] < 0.005)))
    return summary, rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["gravity", "impedance"], default="impedance")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--perturb", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "reports/runs/latest")
    args = parser.parse_args()
    summary, rows = run(args.mode, args.duration, args.perturb)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "trace.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    raise SystemExit(0 if summary["passed"] else 1)


if __name__ == "__main__":
    main()
