"""Six-axis, simulation-only bridge to the pinned Cartesian wrench controllers.

All spatial quantities use world coordinates at ``tool_site``. The joint torque
sensor reports MuJoCo actuator force, not measured motor torque or external force.
"""

from pathlib import Path

import mujoco
import numpy as np

from compliant_control_lab.franka_control import (
    FrankaActuationContext,
    FrankaController,
    FrankaState,
    FrankaTarget,
)


DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[2] / "models/tool4_v2/scene.xml"
JOINT_NAMES = tuple(f"Joint{index}" for index in range(1, 7))


def _vector(values: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite six-vector")
    return result.copy()


class ArmAdapter:
    """Name-mapped six-axis direct-torque MuJoCo runtime, with no hardware I/O."""

    joint_names = JOINT_NAMES

    def __init__(self, model_path: str | Path = DEFAULT_MODEL_PATH):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self._gravity_data = mujoco.MjData(self.model)
        model = self.model
        if (model.nq, model.nv, model.njnt, model.nu) != (6, 6, 6, 6):
            raise ValueError("model must contain exactly six hinge joints and six actuators")
        self.joint_ids = np.array([self._id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES])
        if np.any(model.jnt_type[self.joint_ids] != mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("all named joints must be hinge joints")
        self.qpos_indices = model.jnt_qposadr[self.joint_ids].copy()
        self.dof_indices = model.jnt_dofadr[self.joint_ids].copy()
        self.actuator_ids = np.array([
            self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{name}") for name in JOINT_NAMES
        ])
        self.site_id = self._id(mujoco.mjtObj.mjOBJ_SITE, "tool_site")
        self._validate_actuators()
        self.torque_limits = model.actuator_ctrlrange[self.actuator_ids].copy()
        self._sensor_addresses = self._validate_sensors()
        mujoco.mj_forward(model, self.data)

    def _id(self, kind: mujoco.mjtObj, name: str) -> int:
        result = mujoco.mj_name2id(self.model, kind, name)
        if result < 0:
            raise ValueError(f"model is missing required {name!r}")
        return result

    def _validate_actuators(self) -> None:
        model = self.model
        ids = self.actuator_ids
        if (
            np.any(model.actuator_trntype[ids] != mujoco.mjtTrn.mjTRN_JOINT)
            or not np.array_equal(model.actuator_trnid[ids, 0], self.joint_ids)
            or not np.allclose(model.actuator_gear[ids], [1, 0, 0, 0, 0, 0], rtol=0, atol=1e-14)
            or np.any(model.actuator_dyntype[ids] != mujoco.mjtDyn.mjDYN_NONE)
            or np.any(model.actuator_gaintype[ids] != mujoco.mjtGain.mjGAIN_FIXED)
            or not np.allclose(model.actuator_gainprm[ids, 0], 1, rtol=0, atol=1e-14)
            or np.any(model.actuator_biastype[ids] != mujoco.mjtBias.mjBIAS_NONE)
        ):
            raise ValueError("actuators must be direct unit-gear torque motors on their named joints")
        limits = model.actuator_ctrlrange[ids]
        if (
            not np.all(model.actuator_ctrllimited[ids])
            or not np.all(np.isfinite(limits))
            or np.any(limits[:, 0] >= limits[:, 1])
            or np.any(limits[:, 0] > 0)
            or np.any(limits[:, 1] < 0)
        ):
            raise ValueError("actuators require finite simulation control limits containing zero")
        force_limited = model.actuator_forcelimited[ids].astype(bool)
        force_limits = model.actuator_forcerange[ids]
        if np.any(force_limited & (
            (force_limits[:, 0] > limits[:, 0]) | (force_limits[:, 1] < limits[:, 1])
        )) or np.any(model.jnt_actfrclimited[self.joint_ids]):
            raise ValueError("additional force limits must not narrow the mapped torque limits")

    def _validate_sensors(self) -> dict[str, np.ndarray]:
        model = self.model
        addresses = {}
        for prefix, sensor_type, object_type, object_ids in (
            ("pos", mujoco.mjtSensor.mjSENS_JOINTPOS, mujoco.mjtObj.mjOBJ_JOINT, self.joint_ids),
            ("vel", mujoco.mjtSensor.mjSENS_JOINTVEL, mujoco.mjtObj.mjOBJ_JOINT, self.joint_ids),
            ("tau", mujoco.mjtSensor.mjSENS_ACTUATORFRC, mujoco.mjtObj.mjOBJ_ACTUATOR, self.actuator_ids),
        ):
            ids = np.array([
                self._id(mujoco.mjtObj.mjOBJ_SENSOR, f"{prefix}_{name}") for name in JOINT_NAMES
            ])
            if (
                np.any(model.sensor_type[ids] != sensor_type)
                or np.any(model.sensor_objtype[ids] != object_type)
                or not np.array_equal(model.sensor_objid[ids], object_ids)
                or np.any(model.sensor_dim[ids] != 1)
            ):
                raise ValueError(f"{prefix} sensors must map to the corresponding named objects")
            addresses[prefix] = model.sensor_adr[ids].copy()
        return addresses

    def reset(self, q: np.ndarray, qvel: np.ndarray | None = None) -> None:
        q = _vector(q, "q")
        velocity = np.zeros(6) if qvel is None else _vector(qvel, "qvel")
        limited = self.model.jnt_limited[self.joint_ids].astype(bool)
        bounds = self.model.jnt_range[self.joint_ids]
        if np.any(limited & ((q < bounds[:, 0]) | (q > bounds[:, 1]))):
            raise ValueError("q is outside the model's simulation joint ranges")
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_indices] = q
        self.data.qvel[self.dof_indices] = velocity
        mujoco.mj_forward(self.model, self.data)

    def joint_position(self) -> np.ndarray:
        return self.data.qpos[self.qpos_indices].copy()

    def joint_velocity(self) -> np.ndarray:
        return self.data.qvel[self.dof_indices].copy()

    def pose(self) -> tuple[np.ndarray, np.ndarray]:
        return (
            self.data.site_xpos[self.site_id].copy(),
            self.data.site_xmat[self.site_id].reshape(3, 3).copy(),
        )

    def jacobian(self) -> np.ndarray:
        translation = np.zeros((3, self.model.nv))
        rotation = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, translation, rotation, self.site_id)
        return np.vstack((translation[:, self.dof_indices], rotation[:, self.dof_indices]))

    def gravity(self) -> np.ndarray:
        """Model gravity only, evaluated at zero velocity without changing the live state."""
        mujoco.mj_resetData(self.model, self._gravity_data)
        self._gravity_data.qpos[:] = self.data.qpos
        mujoco.mj_forward(self.model, self._gravity_data)
        return self._gravity_data.qfrc_bias[self.dof_indices].copy()

    def state(self) -> FrankaState:
        position, rotation = self.pose()
        jacobian = self.jacobian()
        twist = jacobian @ self.joint_velocity()
        return FrankaState(
            position=position,
            rotation=rotation,
            linear_velocity=twist[:3],
            angular_velocity=twist[3:],
            normal_force=0.0,  # Compatibility placeholder; no external-force observation exists.
            actuation=FrankaActuationContext(
                cartesian_jacobian=jacobian,
                joint_torque_offset=self.gravity(),
                lower_torque_limit=self.torque_limits[:, 0],
                upper_torque_limit=self.torque_limits[:, 1],
            ),
        )

    def read_joint_sensors(self) -> dict[str, np.ndarray]:
        """Simulation observations; actuator torque is not an external torque estimate."""
        return {
            "position_rad": self.data.sensordata[self._sensor_addresses["pos"]].copy(),
            "velocity_rad_s": self.data.sensordata[self._sensor_addresses["vel"]].copy(),
            "actuator_torque_nm": self.data.sensordata[self._sensor_addresses["tau"]].copy(),
        }

    def apply_torque(self, tau: np.ndarray) -> np.ndarray:
        """Clip to the model's simulation caps and return the command in joint-name order."""
        tau = _vector(tau, "tau")
        applied = np.clip(tau, self.torque_limits[:, 0], self.torque_limits[:, 1])
        self.data.ctrl[self.actuator_ids] = applied
        return applied

    def step(self, tau: np.ndarray) -> np.ndarray:
        applied = self.apply_torque(tau)
        mujoco.mj_step(self.model, self.data)
        # Refresh pose/J/sensors at the newly integrated state before the next controller call.
        mujoco.mj_forward(self.model, self.data)
        return applied

    def controller_torque(
        self,
        controller: FrankaController,
        target: FrankaTarget,
        joint_damping: float | np.ndarray = 0.0,
    ) -> np.ndarray:
        """Compute J.T*w + g - Dqdot; the caller applies limits through step/apply_torque."""
        damping = np.asarray(joint_damping, dtype=float)
        if damping.ndim == 0:
            damping = np.full(6, damping)
        damping = _vector(damping, "joint_damping")
        if np.any(damping < 0):
            raise ValueError("joint_damping must be nonnegative")
        state = self.state()
        assert state.actuation is not None
        offset = state.actuation.joint_torque_offset - damping * self.joint_velocity()
        # A controller using the optional actuation context must see the same offset we apply.
        state = FrankaState(
            position=state.position, rotation=state.rotation,
            linear_velocity=state.linear_velocity, angular_velocity=state.angular_velocity,
            normal_force=state.normal_force,
            actuation=FrankaActuationContext(
                cartesian_jacobian=state.actuation.cartesian_jacobian,
                joint_torque_offset=offset,
                lower_torque_limit=self.torque_limits[:, 0],
                upper_torque_limit=self.torque_limits[:, 1],
            ),
        )
        wrench = _vector(controller.compute(state, target, self.model.opt.timestep), "wrench")
        assert state.actuation is not None
        return state.actuation.joint_torque(wrench)

    def singularity_diagnostic(self, characteristic_length_m: float = 0.3) -> dict:
        """SVD after dividing translational rows by a declared characteristic length."""
        if not np.isfinite(characteristic_length_m) or characteristic_length_m <= 0:
            raise ValueError("characteristic_length_m must be finite and positive")
        scaled = self.jacobian()
        scaled[:3] /= characteristic_length_m
        singular_values = np.linalg.svd(scaled, compute_uv=False)
        return {
            "characteristic_length_m": float(characteristic_length_m),
            "scaled_singular_values": singular_values.tolist(),
            "rank": int(np.linalg.matrix_rank(scaled)),
            "condition_number": float(singular_values[0] / singular_values[-1])
            if singular_values[-1] > 0 else float("inf"),
        }
