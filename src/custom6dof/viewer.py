"""Display the derived model under the same free-space impedance controller."""
import argparse
import json
from pathlib import Path
import struct
import time
import zlib


def write_png(path, rgb):
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff)
    height, width, _ = rgb.shape
    scanlines = b"".join(b"\x00" + row.tobytes() for row in rgb)
    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    data += chunk(b"IDAT", zlib.compress(scanlines)) + chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", type=Path, help="headless render; set MUJOCO_GL=egl before launch")
    args = parser.parse_args()
    import mujoco
    import numpy as np
    from .adapter import ArmAdapter
    from .experiments import configuration, controller, reference
    config = configuration()
    arm = ArmAdapter()
    arm.reset(np.array(config["initial_q_rad"]))
    p0, r0 = arm.pose()
    ctrl = controller(config)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0.1, 0.04]
    camera.distance = 0.85
    camera.azimuth = 145
    camera.elevation = -22
    if args.snapshot:
        with mujoco.Renderer(arm.model, height=480, width=640) as renderer:
            renderer.update_scene(arm.data, camera=camera)
            write_png(args.snapshot, renderer.render())
        print(args.snapshot)
        return
    import mujoco.viewer
    with mujoco.viewer.launch_passive(arm.model, arm.data) as viewer:
        viewer.cam.lookat[:] = camera.lookat
        viewer.cam.distance = camera.distance
        viewer.cam.azimuth = camera.azimuth
        viewer.cam.elevation = camera.elevation
        while viewer.is_running():
            start = time.monotonic()
            for _ in range(10):
                target = reference(arm.data.time, p0, r0, config)
                tau = arm.controller_torque(ctrl, target, np.array(config["joint_damping_nms_rad"]))
                arm.step(tau)
            viewer.sync()
            time.sleep(max(0, 10 * arm.model.opt.timestep - (time.monotonic() - start)))


if __name__ == "__main__":
    main()
