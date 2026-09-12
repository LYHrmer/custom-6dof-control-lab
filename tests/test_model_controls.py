import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from custom6dof.adapter import ArmAdapter
from custom6dof.kinematics import ROOT, URDFChain
from custom6dof.experiments import run


@pytest.fixture(scope="module")
def arm():
    return ArmAdapter()


def test_model_scope_and_mass(arm):
    assert (arm.model.nq, arm.model.nv, arm.model.nu, arm.model.nsensor) == (6, 6, 6, 18)
    assert np.all(arm.model.geom_contype == 0)
    assert np.all(arm.model.geom_conaffinity == 0)
    assert arm.model.body("Link_tool").mass[0] == 0
    assert arm.model.body_mass.sum() == pytest.approx(4.389753219787648, abs=1e-12)
    assert not np.any(arm.model.dof_damping)
    assert not np.any(arm.model.dof_frictionloss)
    assert not np.any(arm.model.dof_armature)


def test_fk_jacobian_and_gravity_independent_references(arm):
    chain = URDFChain()
    rng = np.random.default_rng(60912)
    eps = 1e-6
    for q in rng.uniform(-1.2, 1.2, (20, 6)):
        arm.reset(q)
        p, r = arm.pose()
        transform = chain.fk(q)
        np.testing.assert_allclose(p, transform[:3, 3], atol=2e-12)
        np.testing.assert_allclose(r, transform[:3, :3], atol=2e-12)
        jacobian = arm.jacobian()
        np.testing.assert_allclose(jacobian, chain.jacobian(q), atol=2e-12)
        numerical = np.empty((6, 6))
        for i in range(6):
            delta = np.eye(6)[i] * eps
            plus, minus = chain.fk(q + delta), chain.fk(q - delta)
            numerical[:3, i] = (plus[:3, 3] - minus[:3, 3]) / (2 * eps)
            numerical[3:, i] = Rotation.from_matrix(plus[:3, :3] @ minus[:3, :3].T).as_rotvec() / (2 * eps)
        np.testing.assert_allclose(jacobian, numerical, atol=1e-8)
        np.testing.assert_allclose(arm.gravity(), chain.gravity(q), atol=1e-8)


def test_simulation_start_is_separated_from_zero_singularity(arm):
    arm.reset(np.zeros(6))
    zero = arm.singularity_diagnostic()["scaled_singular_values"][-1]
    q = json.loads((ROOT / "configs/simulation.json").read_text())["initial_q_rad"]
    arm.reset(q)
    start = arm.singularity_diagnostic()["scaled_singular_values"][-1]
    assert zero < 1e-6
    assert start > 0.01


def test_gravity_hold():
    summary, _ = run("gravity", 2.0)
    assert summary["passed"], summary


def test_short_run_cannot_claim_completed_impedance_or_disturbance_validation():
    summary, _ = run("impedance", 0.001, True)
    assert not summary["passed"]
    assert not summary["acceptance_complete"]
    assert summary["perturbation_applied_steps"] == 0


@pytest.mark.parametrize("perturb", [False, True])
def test_slow_impedance_with_optional_unknown_to_controller_disturbance(perturb):
    summary, rows = run("impedance", 10.0, perturb)
    assert summary["passed"], summary
    assert summary["external_wrench_observation"] is None
    assert summary["estimated_external_wrench"] is None
    assert any(row["eval_external_tau_3_nm"] != 0 for row in rows) == perturb
