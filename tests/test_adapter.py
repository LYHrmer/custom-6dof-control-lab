"""Adapter contracts tested against a deliberately reordered synthetic six-axis chain."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from compliant_control_lab.franka_control import FrankaTarget
from custom6dof.adapter import ArmAdapter


def _xml(*, gear="1", actuator_tag="motor", tool_offset="0.04 0.02 0.01"):
    # Neither the serial chain nor the actuator/sensor order matches Joint1..Joint6.
    order = (3, 1, 6, 2, 5, 4)
    axes = ("0 0 1", "0 1 0", "1 0 0", "0 1 0", "1 0 0", "0 0 1")
    body = "".join(
        f'<body name="body{i}" pos="0.1 0 0"><joint name="Joint{i}" '
        f'axis="{axis}" range="-2.5 2.5"/><inertial pos="0.03 0 0" '
        'mass="0.2" diaginertia="0.001 0.002 0.002"/>'
        for i, axis in zip(order, axes)
    )
    body += f'<site name="tool_site" pos="{tool_offset}" euler="0 0 0.3"/>'
    body += "</body>" * 6
    actuators = "".join(
        f'<{actuator_tag} name="motor_Joint{i}" joint="Joint{i}" '
        f'gear="{gear}" ctrllimited="true" ctrlrange="-{i} {i}"/>'
        for i in (6, 2, 4, 1, 5, 3)
    )
    sensors = "".join(
        f'<jointpos name="pos_Joint{i}" joint="Joint{i}"/>'
        f'<actuatorfrc name="tau_Joint{i}" actuator="motor_Joint{i}"/>'
        f'<jointvel name="vel_Joint{i}" joint="Joint{i}"/>'
        for i in reversed(order)
    )
    return (
        '<mujoco><compiler angle="radian"/><option timestep="0.001" gravity="0 0 -9.81"/>'
        f'<worldbody>{body}</worldbody><actuator>{actuators}</actuator>'
        f'<sensor>{sensors}</sensor></mujoco>'
    )


def _adapter(tmp_path: Path, **options):
    path = tmp_path / "synthetic.xml"
    path.write_text(_xml(**options), encoding="utf-8")
    return ArmAdapter(path)


@pytest.fixture
def arm(tmp_path):
    return _adapter(tmp_path)


def test_joint_actuator_and_sensor_mapping_uses_names(arm):
    q = np.linspace(-0.2, 0.3, 6)
    velocity = np.linspace(0.01, 0.06, 6)
    torque = np.linspace(0.1, 0.6, 6)
    arm.reset(q, velocity)
    assert not np.array_equal(arm.qpos_indices, np.arange(6))
    assert not np.array_equal(arm.actuator_ids, np.arange(6))
    np.testing.assert_allclose(arm.joint_position(), q)
    np.testing.assert_allclose(arm.joint_velocity(), velocity)
    arm.apply_torque(torque)
    mujoco.mj_forward(arm.model, arm.data)
    np.testing.assert_allclose(arm.data.qfrc_actuator[arm.dof_indices], torque)
    sensors = arm.read_joint_sensors()
    np.testing.assert_allclose(sensors["position_rad"], q)
    np.testing.assert_allclose(sensors["velocity_rad_s"], velocity)
    np.testing.assert_allclose(sensors["actuator_torque_nm"], torque)


def test_limit_clipping_is_in_named_joint_order(arm):
    applied = arm.apply_torque(np.array([9, -9, 9, -9, 9, -9]))
    np.testing.assert_allclose(applied, [1, -2, 3, -4, 5, -6])
    np.testing.assert_allclose(arm.data.ctrl[arm.actuator_ids], applied)


@pytest.mark.parametrize("bad", [np.zeros(7), np.zeros((6, 1)), [0, 0, 0, 0, 0, np.nan], [np.inf] * 6])
def test_invalid_torque_is_rejected_without_altering_control(arm, bad):
    before = arm.data.ctrl.copy()
    with pytest.raises(ValueError, match="finite six-vector"):
        arm.apply_torque(bad)
    np.testing.assert_array_equal(arm.data.ctrl, before)


def test_reset_validates_before_clearing_existing_state(arm):
    arm.reset(np.ones(6) * 0.1)
    with pytest.raises(ValueError, match="simulation joint ranges"):
        arm.reset(np.ones(6) * 3)
    with pytest.raises(ValueError, match="qvel"):
        arm.reset(np.zeros(6), [np.nan] * 6)
    np.testing.assert_allclose(arm.joint_position(), 0.1)


def test_reset_accepts_arrays_sharing_live_data_storage(arm):
    # A caller may pass MuJoCo arrays directly; resetting must not zero them first.
    arm.reset(np.ones(6) * 0.1, np.ones(6) * 0.02)
    arm.reset(arm.data.qpos, arm.data.qvel)
    np.testing.assert_allclose(arm.joint_position(), 0.1)
    np.testing.assert_allclose(arm.joint_velocity(), 0.02)


def test_tcp_position_and_rotation_include_fixed_site_transform(arm):
    arm.reset(np.zeros(6))
    position, rotation = arm.pose()
    np.testing.assert_allclose(position, [0.64, 0.02, 0.01], atol=1e-14)
    expected = np.array([
        [np.cos(0.3), -np.sin(0.3), 0],
        [np.sin(0.3), np.cos(0.3), 0],
        [0, 0, 1],
    ])
    np.testing.assert_allclose(rotation, expected, atol=1e-14)
    position[:] = 999
    assert np.max(np.abs(arm.pose()[0])) < 1


def test_jacobian_and_twist_respect_named_joint_order(arm):
    q = np.array([0.1, -0.2, 0.3, 0.2, -0.1, 0.4])
    velocity = np.linspace(-0.02, 0.03, 6)
    arm.reset(q, velocity)
    jacobian = arm.jacobian()
    epsilon = 1e-6
    finite_difference = np.empty((3, 6))
    for index in range(6):
        delta = np.eye(6)[index] * epsilon
        arm.reset(q + delta)
        plus = arm.pose()[0]
        arm.reset(q - delta)
        minus = arm.pose()[0]
        finite_difference[:, index] = (plus - minus) / (2 * epsilon)
    np.testing.assert_allclose(jacobian[:3], finite_difference, atol=1e-9)
    arm.reset(q, velocity)
    state = arm.state()
    np.testing.assert_allclose(
        np.r_[state.linear_velocity, state.angular_velocity], jacobian @ velocity, atol=1e-14
    )
    assert state.actuation.cartesian_jacobian.shape == (6, 6)
    assert state.normal_force == 0.0  # Explicit compatibility placeholder, not a sensor.


def test_gravity_ignores_velocity_and_preserves_live_data(arm):
    q = np.linspace(-0.3, 0.2, 6)
    arm.reset(q, np.ones(6) * 1.2)
    before = (arm.data.qpos.copy(), arm.data.qvel.copy(), arm.data.time)
    moving_gravity = arm.gravity()
    np.testing.assert_array_equal(arm.data.qpos, before[0])
    np.testing.assert_array_equal(arm.data.qvel, before[1])
    assert arm.data.time == before[2]
    arm.reset(q)
    np.testing.assert_allclose(arm.gravity(), moving_gravity, atol=1e-14)


def test_step_refreshes_pose_and_sensors_at_integrated_state(arm):
    arm.reset(np.zeros(6), np.ones(6) * 0.02)
    arm.step(arm.gravity())
    position, rotation = arm.pose()
    refreshed = mujoco.MjData(arm.model)
    refreshed.qpos[:] = arm.data.qpos
    refreshed.qvel[:] = arm.data.qvel
    mujoco.mj_forward(arm.model, refreshed)
    np.testing.assert_allclose(position, refreshed.site_xpos[arm.site_id], atol=1e-14)
    np.testing.assert_allclose(rotation, refreshed.site_xmat[arm.site_id].reshape(3, 3), atol=1e-14)
    np.testing.assert_allclose(arm.read_joint_sensors()["position_rad"], arm.joint_position())
    assert arm.data.time == pytest.approx(arm.model.opt.timestep)


@pytest.mark.parametrize("options", [{"gear": "2"}, {"actuator_tag": "position"}])
def test_incompatible_actuators_are_rejected(tmp_path, options):
    with pytest.raises(ValueError, match="direct unit-gear torque motors"):
        _adapter(tmp_path, **options)


def test_wrong_sensor_joint_mapping_is_rejected(tmp_path):
    xml = _xml().replace('name="pos_Joint1" joint="Joint1"', 'name="pos_Joint1" joint="Joint2"')
    path = tmp_path / "bad_sensor.xml"
    path.write_text(xml, encoding="utf-8")
    with pytest.raises(ValueError, match="sensors must map"):
        ArmAdapter(path)


def test_hidden_force_limit_is_rejected(tmp_path):
    xml = _xml().replace('name="motor_Joint1"', 'name="motor_Joint1" forcelimited="true" forcerange="-0.1 0.1"')
    path = tmp_path / "bad_limit.xml"
    path.write_text(xml, encoding="utf-8")
    with pytest.raises(ValueError, match="additional force limits"):
        ArmAdapter(path)


def test_controller_offset_matches_optional_context_and_has_no_posture_term(arm):
    arm.reset(np.linspace(-0.2, 0.3, 6), np.ones(6) * 0.04)
    wrench = np.linspace(-0.1, 0.2, 6)
    captured = []

    class Controller:
        def compute(self, state, target, dt):
            captured.append(state.actuation)
            assert dt == arm.model.opt.timestep
            return wrench

    position, rotation = arm.pose()
    target = FrankaTarget(position, rotation, np.zeros(3), np.zeros(3), 0.0)
    torque = arm.controller_torque(Controller(), target, joint_damping=0.2)
    expected = arm.jacobian().T @ wrench + arm.gravity() - 0.2 * arm.joint_velocity()
    np.testing.assert_allclose(torque, expected, atol=1e-14)
    np.testing.assert_allclose(captured[0].joint_torque(wrench), torque, atol=1e-14)
    with pytest.raises(ValueError, match="nonnegative"):
        arm.controller_torque(Controller(), target, joint_damping=-1)


def test_singularity_scaling_is_explicit(arm):
    arm.reset(np.linspace(-0.3, 0.2, 6))
    report = arm.singularity_diagnostic()
    expected = arm.jacobian()
    expected[:3] /= 0.3
    np.testing.assert_allclose(report["scaled_singular_values"], np.linalg.svd(expected, compute_uv=False))
    assert report["characteristic_length_m"] == 0.3
    with pytest.raises(ValueError, match="finite and positive"):
        arm.singularity_diagnostic(0)
