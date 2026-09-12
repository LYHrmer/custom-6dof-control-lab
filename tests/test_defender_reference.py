"""Optional, offline tests of the user-supplied six-axis reference snapshot."""
import importlib.util
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("defender_parity", ROOT / "tools/defender_parity.py")
parity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parity)

pytestmark = pytest.mark.skipif(
    not parity.GRAVITY.is_dir() or shutil.which("g++") is None,
    reason="optional Robot-Defender snapshot and host g++ required",
)


@pytest.fixture(scope="module")
def host(tmp_path_factory):
    return parity.compile_host(tmp_path_factory.mktemp("defender") / "gravity_host")


@pytest.mark.parametrize("text", ["", "0 0 1 0 0 0 0 0\n", "0 0 1 0 0 0 0 0 0 extra\n",
                                  "0 0 1 nan 0 0 0 0 0\n", "0 0 0 0 0 0 0 0 0\n",
                                  "0 0 1 inf 0 0 0 0 0\n"])
def test_host_rejects_invalid_input(host, text):
    result = subprocess.run(["rtk", "proxy", str(host)], input=text, capture_output=True, text=True)
    assert result.returncode != 0
    assert result.stdout.strip() == ""


def test_up_scale_and_two_rows(host):
    q = np.tile([.1, -.2, .3, -.4, .5, -.6], (2, 1))
    tau = parity.host_torques(host, q, [[1, 2, 3], [2, 4, 6]])
    assert tau.shape == (2, 6)
    np.testing.assert_allclose(tau[0], tau[1], atol=1e-7, rtol=0)


def test_cpp_targets_original_mass_while_new_model_uses_corrected_mass():
    report = parity.evaluate(samples=32, seed=137)
    assert report["passed"], report["metrics"]
    assert report["metrics"]["old_fitted_cpp_vs_corrected_model_max_nm"] > 1e-3
    assert report["hardware_io"] is False
