"""Generate versioned free-space MJCF from original URDF and processed visuals."""
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from custom6dof.kinematics import ROOT, ORIGINAL_URDF, URDFChain, origin


def numbers(values):
    return " ".join(f"{v:.17g}" for v in values)


def pose_attrs(transform):
    q = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return {"pos": numbers(transform[:3, 3]), "quat": numbers(q[[3, 0, 1, 2]])}


def main():
    config_path = ROOT / "configs/simulation.json"
    config = json.loads(config_path.read_text())
    chain = URDFChain()
    target = ROOT / "models" / config["model_version"]
    target.mkdir(parents=True, exist_ok=True)
    model = ET.Element("mujoco", model=config["model_version"] + "_free_space")
    ET.SubElement(model, "compiler", angle="radian", inertiafromgeom="false", fusestatic="false")
    ET.SubElement(model, "option", timestep=str(config["timestep_s"]), gravity=numbers(config["gravity_m_s2"]), integrator="implicitfast")
    asset = ET.SubElement(model, "asset")
    mesh_files = []
    for name in chain.links:
        if name == "Link_tool":
            continue
        mesh = ROOT / "inputs/processed/meshes/visual" / f"{name}.STL"
        mesh_files.append(mesh)
        ET.SubElement(asset, "mesh", name=name, file=os.path.relpath(mesh, target))
    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "light", pos="0 -1 2", dir="0 0 -1")
    ET.SubElement(world, "geom", name="display_floor", type="plane", pos="0 0 -0.22", size="1 1 .01", rgba=".17 .19 .22 1", contype="0", conaffinity="0")
    bodies = {}

    def add_link(parent, name, transform):
        body = ET.SubElement(parent, "body", name=name, **pose_attrs(transform))
        bodies[name] = body
        if name == "Link_tool":
            ET.SubElement(body, "site", name="tool_site", pos="0 0 0", size=".003", rgba="1 .2 .1 1")
            return body
        link = chain.links[name]
        inertial = link.find("inertial")
        frame = origin(inertial.find("origin"))
        i = inertial.find("inertia").attrib
        tensor = np.array([[float(i["ixx"]), float(i["ixy"]), float(i["ixz"])], [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])], [float(i["ixz"]), float(i["iyz"]), float(i["izz"])]])
        tensor = frame[:3, :3] @ tensor @ frame[:3, :3].T
        ET.SubElement(body, "inertial", pos=numbers(frame[:3, 3]), mass=inertial.find("mass").get("value"), fullinertia=numbers(tensor[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]))
        visual_frame = origin(link.find("visual/origin"))
        ET.SubElement(body, "geom", name="visual_" + name, type="mesh", mesh=name, contype="0", conaffinity="0", group="1", rgba=".74 .77 .8 1", **pose_attrs(visual_frame))
        return body

    add_link(world, chain.base, np.eye(4))
    for joint in chain.joints:
        child = joint.find("child").get("link")
        parent = bodies[joint.find("parent").get("link")]
        body = add_link(parent, child, origin(joint.find("origin")))
        if joint.get("type") == "revolute":
            limits = joint.find("limit")
            ET.SubElement(body, "joint", name=joint.get("name"), type="hinge", axis=joint.find("axis").get("xyz"), limited="true", range=f'{limits.get("lower")} {limits.get("upper")}', damping="0", frictionloss="0", armature="0")
    actuator = ET.SubElement(model, "actuator")
    sensors = ET.SubElement(model, "sensor")
    for joint, cap in zip(chain.names, config["ideal_actuator_cap_nm"]):
        ET.SubElement(actuator, "motor", name="motor_" + joint, joint=joint, gear="1", ctrllimited="true", ctrlrange=numbers([-cap, cap]), forcelimited="true", forcerange=numbers([-cap, cap]))
        ET.SubElement(sensors, "jointpos", name="pos_" + joint, joint=joint)
        ET.SubElement(sensors, "jointvel", name="vel_" + joint, joint=joint)
        ET.SubElement(sensors, "actuatorfrc", name="tau_" + joint, actuator="motor_" + joint)
    keyframes = ET.SubElement(model, "keyframe")
    ET.SubElement(keyframes, "key", name="simulation_start", qpos=numbers(config["initial_q_rad"]))
    ET.indent(model)
    xml = ET.tostring(model, encoding="unicode") + "\n"
    scene = target / "scene.xml"
    if scene.exists() and scene.read_text() != xml:
        raise SystemExit("Versioned scene differs: create a new model_version instead of overwriting")
    scene.write_text(xml)
    compiled = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(compiled)
    data.qpos[:] = config["initial_q_rad"]
    mujoco.mj_forward(compiled, data)
    jac = chain.jacobian(config["initial_q_rad"])
    scaled = jac.copy()
    scaled[:3] /= config["singularity_length_scale_m"]
    sources = [ORIGINAL_URDF, config_path, Path(__file__).resolve(), ROOT / "src/custom6dof/kinematics.py", *mesh_files]
    manifest = {
        "model_version": config["model_version"], "mujoco_version": mujoco.__version__,
        "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "nq": compiled.nq, "nv": compiled.nv, "nu": compiled.nu, "nsensor": compiled.nsensor,
        "dynamic_mass_kg": float(compiled.body_mass[2:].sum()),
        "total_body_mass_kg": float(compiled.body_mass.sum()),
        "initial_scaled_jacobian_singular_values": np.linalg.svd(scaled, compute_uv=False).tolist(),
        "initial_gravity_nm": data.qfrc_bias.tolist(),
        "corrections": ["Remove Link_tool duplicate inertial and visual/collision; retain Joint_tool transform and Link_tool frame. Keep Link6 inertial convention as exported."],
        "assumptions": ["All contact disabled: processed meshes used only for rendering.", "Original +/-3.14 ranges are unverified export values, not verified mechanical stops.", "No inferred friction/damping/armature. Zero means idealized omission, not measurement.", "Ideal direct torque motor cap 10 Nm each is a numerical envelope, not DM4310 capability.", "Tool site at exported Link_tool origin, not a measured contact TCP.", "CAD inertias retained despite suspected sign issue; no hardware accuracy claim."]
    }
    (target / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (target / "simulation_config.json").write_text(config_path.read_text())
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
