"""Read-only, reproducible audit of the preserved CAD export and processed model.

Run ``python -m custom6dof.audit --output reports/model_audit.json``.
Numbers are interpreted using URDF conventions; physical calibration is separate.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def vector(value: str | None, default: str = "0 0 0") -> np.ndarray:
    result = np.fromstring(default if value is None else value, sep=" ")
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"Expected three finite coordinates, got {value!r}")
    return result


def origin(element: ET.Element | None) -> tuple[np.ndarray, np.ndarray]:
    attrs = {} if element is None else element.attrib
    return vector(attrs.get("xyz")), vector(attrs.get("rpy"))


def rotation(rpy: list[float] | np.ndarray) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    return Rotation.from_euler("xyz", rpy).as_matrix()


def inertia_matrix(attrs: dict[str, str]) -> np.ndarray:
    xx, xy, xz, yy, yz, zz = (float(attrs[k]) for k in
                             ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])


def inertia_checks(matrix: np.ndarray) -> dict:
    finite = bool(np.isfinite(matrix).all())
    if not finite:
        return {"finite": False, "positive_definite": False,
                "principal_triangle_inequality": False}
    eigenvalues = np.linalg.eigvalsh(matrix)
    margin = float(eigenvalues[0] + eigenvalues[1] - eigenvalues[2])
    return {"finite": True, "principal_moments_kg_m2": eigenvalues.tolist(),
            "positive_definite": bool(eigenvalues[0] > 0),
            "principal_triangle_inequality": bool(margin >= -1e-12),
            "triangle_margin_kg_m2": margin}


def parse_urdf(path: Path) -> dict:
    root = ET.parse(path).getroot()
    links = {}
    for element in root.findall("link"):
        inertial = element.find("inertial")
        xyz, rpy = origin(None if inertial is None else inertial.find("origin"))
        mass = None if inertial is None else float(inertial.find("mass").get("value"))
        matrix = None if inertial is None else inertia_matrix(inertial.find("inertia").attrib)
        data = {"mass_kg": mass, "com_xyz_m": xyz.tolist(), "inertia_rpy_rad": rpy.tolist(),
                "inertia_kg_m2": None if matrix is None else matrix.tolist()}
        if matrix is not None:
            data["checks"] = {"positive_mass": bool(np.isfinite(mass) and mass > 0),
                              **inertia_checks(matrix)}
        for role in ("visual", "collision"):
            data[role] = []
            for geometry in element.findall(role):
                mesh = geometry.find("geometry/mesh")
                if mesh is None:
                    continue
                xyz, rpy = origin(geometry.find("origin"))
                data[role].append({"mesh": mesh.get("filename"), "xyz_m": xyz.tolist(),
                                   "rpy_rad": rpy.tolist(),
                                   "scale": vector(mesh.get("scale"), "1 1 1").tolist()})
        links[element.get("name")] = data
    joints = {}
    for element in root.findall("joint"):
        xyz, rpy = origin(element.find("origin"))
        axis = element.find("axis")
        limit, dynamics = element.find("limit"), element.find("dynamics")
        joints[element.get("name")] = {
            "type": element.get("type"), "parent": element.find("parent").get("link"),
            "child": element.find("child").get("link"), "xyz_m": xyz.tolist(),
            "rpy_rad": rpy.tolist(),
            "axis": vector(None if axis is None else axis.get("xyz"), "1 0 0").tolist(),
            "limit": {} if limit is None else {k: float(v) for k, v in limit.attrib.items()},
            "dynamics": {} if dynamics is None else {k: float(v) for k, v in dynamics.attrib.items()},
        }
    return {"robot_name": root.get("name"), "links": links, "joints": joints}


def chain_checks(model: dict) -> dict:
    links, joints = model["links"], model["joints"]
    children = [j["child"] for j in joints.values()]
    roots = sorted(set(links) - set(children))
    adjacency = {name: [] for name in links}
    invalid = []
    for name, joint in joints.items():
        if joint["parent"] not in links or joint["child"] not in links:
            invalid.append(name)
        else:
            adjacency[joint["parent"]].append(joint["child"])
    visited, active = set(), set()

    def visit(name):
        if name in active:
            return False
        if name in visited:
            return True
        active.add(name)
        valid = all(visit(child) for child in adjacency[name])
        active.remove(name)
        visited.add(name)
        return valid

    acyclic = all(visit(name) for name in links)
    connected = set()
    queue = roots[:]
    while queue:
        name = queue.pop()
        if name not in connected:
            connected.add(name)
            queue.extend(adjacency[name])
    valid_tree = (not invalid and len(roots) == 1 and len(set(children)) == len(children)
                  and acyclic and connected == set(links))
    return {"link_count": len(links), "joint_count": len(joints), "roots": roots,
            "valid_tree": valid_tree,
            "serial_chain": bool(valid_tree and all(len(v) <= 1 for v in adjacency.values())),
            "revolute_joints": [n for n, j in joints.items() if j["type"] == "revolute"],
            "fixed_joints": [n for n, j in joints.items() if j["type"] == "fixed"],
            "invalid_link_references": invalid}


def read_stl(path: Path) -> np.ndarray:
    """Read triangle vertices without geometry repair or implicit rescaling."""
    blob = path.read_bytes()
    if len(blob) >= 84:
        count = struct.unpack_from("<I", blob, 80)[0]
        if len(blob) == 84 + 50 * count:
            dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                              ("attribute", "<u2")])
            return np.frombuffer(blob, dtype=dtype, offset=84, count=count)["vertices"].astype(float)
    try:
        rows = [line.split()[1:] for line in blob.decode("ascii").splitlines()
                if line.strip().startswith("vertex ")]
        vertices = np.asarray(rows, dtype=float)
        if vertices.size and vertices.shape[1] == 3 and len(vertices) % 3 == 0:
            return vertices.reshape(-1, 3, 3)
    except (UnicodeDecodeError, ValueError):
        pass
    raise ValueError(f"Malformed or unsupported STL: {path}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def mesh_path(package: Path, filename: str) -> Path:
    if filename.startswith("package://"):
        return package / filename[len("package://"):].split("/", 1)[1]
    return package / filename


def mesh_inventory(package: Path, base: Path) -> list[dict]:
    result = []
    for path in sorted(package.rglob("*")):
        if path.suffix.lower() != ".stl":
            continue
        triangles = read_stl(path)
        vertices = triangles.reshape(-1, 3)
        result.append({"path": str(path.relative_to(base)), "sha256": sha256(path),
                       "triangle_count": len(triangles),
                       "bounds_min_assumed_m": vertices.min(axis=0).tolist(),
                       "bounds_max_assumed_m": vertices.max(axis=0).tolist()})
    return result


def nearest_vertex_metrics(a: np.ndarray, b: np.ndarray) -> dict:
    """Symmetric nearest *vertex* distances, not surface/contact errors."""
    distances = np.concatenate((cKDTree(b).query(a)[0], cKDTree(a).query(b)[0]))
    return {"rms_m": float(np.sqrt(np.mean(distances**2))),
            "p95_m": float(np.percentile(distances, 95)),
            "max_m": float(distances.max()),
            "sample_count": len(distances),
            "method": "symmetric nearest STL vertex; triangle vertices retain multiplicity"}


def duplicate_tool_check(model: dict, package: Path, csv_path: Path) -> dict:
    joint = model["joints"]["Joint_tool"]
    parent, tool = model["links"]["Link6"], model["links"]["Link_tool"]
    if joint["type"] != "fixed" or joint["parent"] != "Link6" or joint["child"] != "Link_tool":
        raise ValueError("Joint_tool no longer describes the audited Link6 -> Link_tool transform")
    r, t = rotation(joint["rpy_rad"]), np.asarray(joint["xyz_m"])

    def vertices(link):
        visual = link["visual"][0]
        raw = read_stl(mesh_path(package, visual["mesh"])).reshape(-1, 3)
        return ((raw * visual["scale"]) @ rotation(visual["rpy_rad"]).T
                + visual["xyz_m"])

    com_a = np.asarray(parent["com_xyz_m"])
    com_b = r @ np.asarray(tool["com_xyz_m"]) + t
    ra, rb = rotation(parent["inertia_rpy_rad"]), rotation(tool["inertia_rpy_rad"])
    ia, ib = np.asarray(parent["inertia_kg_m2"]), np.asarray(tool["inertia_kg_m2"])

    def mismatch(a, b):
        a_link = ra @ a @ ra.T
        b_link = r @ rb @ b @ rb.T @ r.T
        # Compare tensors at their respective COMs, with the common Link6 axes.
        return float(np.linalg.norm(a_link - b_link) / np.linalg.norm(a_link))

    def flip_products(matrix):
        return 2 * np.diag(np.diag(matrix)) - matrix

    with csv_path.open(encoding="utf-8-sig", newline="") as stream:
        rows = {row["Link Name"]: row for row in csv.DictReader(stream)}
    cad = {name: rows[name]["SW Components"] for name in ("Link6", "Link_tool")}
    same = cad["Link6"] == cad["Link_tool"]
    geometry = nearest_vertex_metrics(vertices(parent), vertices(tool) @ r.T + t)
    supported = bool(same and parent["mass_kg"] == tool["mass_kg"]
                     and geometry["max_m"] < 1e-6
                     and np.linalg.norm(com_a - com_b) < 1e-6)
    return {"fixed_transform_child_to_parent": {"rotation": r.tolist(), "translation_m": t.tolist()},
            "cad_components": cad, "same_cad_instance_string": same,
            "mass_each_kg": [parent["mass_kg"], tool["mass_kg"]],
            "com_distance_m": float(np.linalg.norm(com_a - com_b)),
            "geometry": geometry,
            "inertia_relative_frobenius_difference": mismatch(ia, ib),
            "inertia_comparison": "each tensor about its COM, rotated to Link6 axes; denominator ||I_Link6||F",
            "hypothesis_only_flip_both_off_diagonal_relative_difference": mismatch(flip_products(ia), flip_products(ib)),
            "hypothesis_action": "diagnostic only; no inertia sign changes authorized by this numerical comparison",
            "duplicate_supported": supported,
            "recommended_model_correction": ("retain Link6 physical properties and geometry once; retain Link_tool as a massless frame"
                                             if supported else None)}


def differences(before, after, path="") -> list[dict]:
    if isinstance(before, dict) and isinstance(after, dict):
        return [change for key in sorted(set(before) | set(after))
                for change in differences(before.get(key), after.get(key), f"{path}.{key}".strip("."))]
    if before == after:
        return []
    return [{"parameter": path, "original": before, "processed": after}]


def zero_pose_check(model: dict) -> dict:
    """URDF-only geometric Jacobian at the tool origin, with q = 0."""
    root = chain_checks(model)["roots"][0]
    poses = {root: (np.eye(3), np.zeros(3))}
    world_axes, joint_origins = [], []
    pending = list(model["joints"].values())
    while pending:
        remaining = []
        for joint in pending:
            if joint["parent"] not in poses:
                remaining.append(joint)
                continue
            rp, pp = poses[joint["parent"]]
            r = rp @ rotation(joint["rpy_rad"])
            p = pp + rp @ np.asarray(joint["xyz_m"])
            poses[joint["child"]] = (r, p)
            if joint["type"] == "revolute":
                world_axes.append(r @ np.asarray(joint["axis"]))
                joint_origins.append(p)
        if len(remaining) == len(pending):
            raise ValueError("URDF joint graph is disconnected or cyclic")
        pending = remaining
    p_tool = poses["Link_tool"][1]
    jacobian = np.column_stack([np.r_[np.cross(a, p_tool - p), a]
                               for a, p in zip(world_axes, joint_origins)])
    # A raw spatial Jacobian mixes metres and radians. State the scaling explicitly.
    scaled = jacobian.copy()
    scaled[:3] /= 0.1
    singular = np.linalg.svd(scaled, compute_uv=False)
    return {"q_rad": [0.0] * len(world_axes), "tool_position_m": p_tool.tolist(),
            "jacobian_world_linear_then_angular": jacobian.tolist(),
            "translational_scaling_length_m": 0.1,
            "scaled_singular_values": singular.tolist(),
            "scaled_condition_number": float(singular[0] / singular[-1]),
            "note": "condition is scaling-dependent; zero pose is not a validated control initial pose"}


def mujoco_probe(urdf_path: Path, package: Path) -> dict:
    try:
        import mujoco
    except ImportError:
        return {"available": False, "reason": "mujoco is not installed in this interpreter"}
    root = ET.parse(urdf_path).getroot()
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", str(mesh_path(package, mesh.get("filename")).resolve()))
    extension = root.find("mujoco")
    if extension is None:
        extension = ET.SubElement(root, "mujoco")
    compiler = extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(extension, "compiler")
    compiler.set("strippath", "false")
    try:
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    except ValueError as error:
        return {"available": True, "version": mujoco.__version__, "compiled": False,
                "error": str(error),
                "probe_changes": "resolve package mesh paths; compiler strippath=false only"}
    return {"available": True, "version": mujoco.__version__, "compiled": True,
            "nq": model.nq, "nv": model.nv, "nu": model.nu, "nsensor": model.nsensor,
            "probe_changes": "resolve package mesh paths; compiler strippath=false only"}


def audit(project_root: Path = PROJECT_ROOT) -> dict:
    project_root = Path(project_root).resolve()
    urdfs = sorted((project_root / "inputs/original").glob("*/urdf/*.urdf"))
    if len(urdfs) != 1:
        raise ValueError(f"Expected one preserved original URDF; found {len(urdfs)}")
    original_path = urdfs[0]
    original_package = original_path.parent.parent
    processed_package = project_root / "inputs/processed"
    processed_path = processed_package / "urdf/custom_arm_tool4.urdf"
    csv_path = original_path.with_suffix(".csv")
    original, processed = parse_urdf(original_path), parse_urdf(processed_path)
    duplicate = duplicate_tool_check(original, original_package, csv_path)
    source_paths = [original_path, processed_path, csv_path, project_root / "inputs/claude_advice.md"]
    with csv_path.open(encoding="utf-8-sig", newline="") as stream:
        cad_rows = list(csv.DictReader(stream))
    models = {}
    for name, model, path, package in (("original", original, original_path, original_package),
                                        ("processed", processed, processed_path, processed_package)):
        models[name] = {**model, "chain": chain_checks(model),
                       "total_mass_kg": sum(link["mass_kg"] or 0 for link in model["links"].values()),
                       "moving_mass_kg": sum(link["mass_kg"] or 0 for key, link in model["links"].items() if key != "base_link"),
                       "mujoco_import": mujoco_probe(path, package)}
    return {
        "schema_version": 1,
        "units": {"length": "URDF m; STL interpreted consistently with URDF, not physically measured",
                  "mass": "kg", "inertia": "kg m^2", "joint_velocity": "rad/s",
                  "joint_effort": "N m per URDF convention, not verified motor ratings"},
        "sources": [{"path": str(p.relative_to(project_root)), "sha256": sha256(p)} for p in source_paths],
        "models": models,
        "original_to_processed_changes": differences(original, processed),
        "meshes": {"original": mesh_inventory(original_package, project_root),
                   "processed": mesh_inventory(processed_package, project_root)},
        "duplicate_tool": duplicate,
        "zero_pose": zero_pose_check(original),
        "cad_motor_name_evidence": [
            {"link": row["Link Name"], "components": row["SW Components"]}
            for row in cad_rows if "4310" in row["SW Components"] or "4340" in row["SW Components"]],
        "findings": [
            {"status": "supported_by_export_evidence" if duplicate["duplicate_supported"] else "requires_review",
             "claim": "Link6 and Link_tool duplicate the same exported physical instance",
             "evidence": "duplicate_tool: CSV identity, transformed vertices, COM and mass",
             "action": ("remove one physical copy in a derived model, preserving both source exports and the tool frame"
                        if duplicate["duplicate_supported"] else "retain physical copies pending independent evidence")},
            {"status": "unresolved", "claim": "products-of-inertia sign convention may be inconsistent",
             "evidence": "duplicate_tool inertia comparison and alternate-sign diagnostic",
             "action": "retain original signs pending CAD/exporter verification; SPD is not proof of correct sign convention"},
            {"status": "unverified_hardware", "claim": "export limits and processed effort/dynamics are not hardware calibration",
             "evidence": "models.*.joints and original_to_processed_changes",
             "action": "do not treat 100 N m, 7.9 N m, damping or friction as a DM4310 rating or measured parameter"},
        ],
        "unknowns": [
            "Actual joint hard stops, encoder zero/sign, velocity and continuous/peak torque limits remain unmeasured.",
            "User confirms DM4310 on hardware; CAD component names also include DM-J4340. Reconcile CAD/BOM with the actual arm.",
            "Torque feedback definition, units, rotor versus gearbox-output side, gearbox ratio and control mode require motor documentation and logged verification.",
            "CAD masses, material densities, motor inclusion, inertia signs and centre-of-mass placement are not physically validated.",
            "STL files contain no reliable unit declaration; interpreting coordinates as metres does not independently establish physical scale.",
            "Ordinary MuJoCo mesh collision uses a convex hull: fork cavities and internal clearances are not validated contact geometry. Visual nearest-vertex error is not contact error.",
            "Motor torque feedback is not an end-effector wrench. Simulation input, estimated wrench and evaluation truth require separate channels.",
            "Removing the duplicate fixes an export inconsistency; it does not validate the retained Link6 inertia against CAD or hardware.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=PROJECT_ROOT)
    parser.add_argument("--output", type=Path, default=Path("reports/model_audit.json"))
    args = parser.parse_args()
    report = audit(args.project_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Audit written to {args.output}")
    print(json.dumps({"duplicate_tool": report["duplicate_tool"],
                      "mujoco": {key: value["mujoco_import"] for key, value in report["models"].items()}},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
