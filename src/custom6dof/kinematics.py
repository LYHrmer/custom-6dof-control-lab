"""Independent URDF serial-chain FK, Jacobian and potential-energy reference.

No MuJoCo calls: this is the numerical oracle for the simulation adapter.
The derivation retains URDF axes/origins and drops only the duplicate tool mass.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
ORIGINAL_URDF = ROOT / "inputs/original/自定义控制器3.0-tool-4/urdf/自定义控制器3.0-tool-4.urdf"


def vector(text):
    return np.fromstring(text, sep=" ")


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, :3] = Rotation.from_euler("xyz", vector(element.get("rpy", "0 0 0"))).as_matrix()
        transform[:3, 3] = vector(element.get("xyz", "0 0 0"))
    return transform


class URDFChain:
    def __init__(self, path=ORIGINAL_URDF, deduplicate_tool=True):
        self.root = ET.parse(path).getroot()
        self.links = {link.get("name"): link for link in self.root.findall("link")}
        joints = self.root.findall("joint")
        children = {j.find("child").get("link") for j in joints}
        roots = set(self.links) - children
        if len(roots) != 1:
            raise ValueError("expected one root")
        self.base = roots.pop()
        self.joints = []
        current = self.base
        while True:
            following = [j for j in joints if j.find("parent").get("link") == current]
            if not following:
                break
            if len(following) != 1 or following[0] in self.joints:
                raise ValueError("expected acyclic serial chain")
            self.joints.append(following[0])
            current = following[0].find("child").get("link")
        if len(self.joints) != len(joints):
            raise ValueError("disconnected chain")
        self.names = tuple(j.get("name") for j in self.joints if j.get("type") == "revolute")
        if self.names != tuple(f"Joint{i}" for i in range(1, 7)):
            raise ValueError("unexpected joint mapping")
        self.deduplicate_tool = deduplicate_tool

    def frames(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("q must be finite shape (6,)")
        frames = {self.base: np.eye(4)}
        axes, points = [], []
        index = 0
        for joint in self.joints:
            frame = frames[joint.find("parent").get("link")] @ origin(joint.find("origin"))
            if joint.get("type") == "revolute":
                axis = vector(joint.find("axis").get("xyz"))
                axis /= np.linalg.norm(axis)
                axes.append(frame[:3, :3] @ axis)
                points.append(frame[:3, 3].copy())
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                frame = frame @ rotation
                index += 1
            elif joint.get("type") != "fixed":
                raise ValueError("unsupported joint")
            frames[joint.find("child").get("link")] = frame
        return frames, np.array(axes), np.array(points)

    def fk(self, q):
        return self.frames(q)[0]["Link_tool"]

    def jacobian(self, q):
        frames, axes, points = self.frames(q)
        position = frames["Link_tool"][:3, 3]
        return np.vstack((np.cross(axes, position - points).T, axes.T))

    def potential(self, q, gravity=(0, 0, -9.81)):
        frames = self.frames(q)[0]
        potential = 0.0
        for name, link in self.links.items():
            if self.deduplicate_tool and name == "Link_tool":
                continue
            inertial = link.find("inertial")
            mass = float(inertial.find("mass").get("value"))
            com = (frames[name] @ origin(inertial.find("origin")))[:3, 3]
            potential -= mass * np.dot(gravity, com)
        return float(potential)

    def gravity(self, q, epsilon=1e-6):
        q = np.asarray(q, dtype=float)
        eye = np.eye(6) * epsilon
        return np.array([(self.potential(q + e) - self.potential(q - e)) / (2 * epsilon) for e in eye])
