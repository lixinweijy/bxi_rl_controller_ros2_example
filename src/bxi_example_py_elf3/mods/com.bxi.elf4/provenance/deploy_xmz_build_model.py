"""Build an ELF4 MuJoCo deployment model from this repository's ELF4 URDF.

The CAD meshes are decimated only for visualization. Simple collision shapes
are added for simulation because the visual-only GMR preview model has no
robot collisions. The URDF and USD are left untouched.
"""

from __future__ import annotations

import argparse
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


DEPLOY_DIR = Path(__file__).resolve().parent
URDF = DEPLOY_DIR / "source" / "elf4.urdf"
SOURCE_MESH_DIR = DEPLOY_DIR / "model" / "meshes"
OUTPUT = DEPLOY_DIR / "model" / "elf4.xml"


def write_msh(mesh, path: Path) -> None:
    """Write MuJoCo's indexed binary mesh format without keeping all links in memory."""
    mesh.merge_vertices()
    vertices = np.ascontiguousarray(mesh.vertices, dtype="<f4")
    faces = np.ascontiguousarray(mesh.faces, dtype="<i4")
    with path.open("wb") as stream:
        stream.write(np.array([len(vertices), 0, 0, len(faces)], dtype="<i4").tobytes())
        stream.write(vertices.tobytes())
        stream.write(faces.tobytes())


def add_collision(body: ET.Element, *, shape: str, pos: str | None = None,
                  size: str | None = None, fromto: str | None = None,
                  friction: str = "0.8 0.02 0.002") -> None:
    attrs = {"name": f"{body.get('name')}_collision", "type": shape,
             "contype": "1", "conaffinity": "2", "group": "3", "rgba": "0.3 0.7 0.3 0.2",
             "friction": friction}
    if pos is not None:
        attrs["pos"] = pos
    if size is not None:
        attrs["size"] = size
    if fromto is not None:
        attrs["fromto"] = fromto
    ET.SubElement(body, "geom", attrs)


def build(output: Path = OUTPUT, max_faces: int = 150_000,
          urdf: Path = URDF, source_mesh_dir: Path = SOURCE_MESH_DIR) -> Path:
    """Export compact ELF4 visuals plus contact primitives from the URDF."""
    if not urdf.is_file():
        raise FileNotFoundError(f"ELF4 URDF not found: {urdf}")
    if not source_mesh_dir.is_dir():
        raise FileNotFoundError(f"ELF4 mesh directory not found: {source_mesh_dir}")
    if not 1 <= max_faces <= 199_999:
        raise ValueError("--max-faces must be between 1 and 199999 for MuJoCo STL loading")
    output.parent.mkdir(parents=True, exist_ok=True)
    mesh_dir = output.parent / "meshes"
    mesh_dir.mkdir(exist_ok=True)
    source_by_name = {source.stem: source for source in source_mesh_dir.glob("*.STL")}
    source_by_name.update({source.stem: source for source in source_mesh_dir.glob("*.msh")})
    required_names = {
        Path(mesh.get("filename")).stem
        for mesh in ET.parse(urdf).getroot().findall(".//visual/geometry/mesh")
    }
    missing = required_names - source_by_name.keys()
    if missing:
        raise FileNotFoundError(f"ELF4 visual meshes missing from {source_mesh_dir}: {sorted(missing)}")
    sources = [source_by_name[name] for name in sorted(required_names)]
    for source in sources:
        target = mesh_dir / f"{source.stem}.msh"
        if source.suffix == ".msh":
            if source.resolve() != target.resolve():
                shutil.copy2(source, target)
            continue
        import trimesh

        mesh = trimesh.load_mesh(source, process=False)
        if len(mesh.faces) > max_faces:
            mesh = mesh.simplify_quadric_decimation(face_count=max_faces)
        write_msh(mesh, target)
        print(f"{source.name}: {len(mesh.faces)} faces", flush=True)
        del mesh

    with tempfile.TemporaryDirectory() as scratch:
        temp = Path(scratch)
        # MuJoCo's URDF importer looks for meshes beside the URDF.
        urdf_text = re.sub(r'filename="[^"]*/([^/"\\]+)\.(?:STL|msh)"',
                           r'filename="\1.msh"', urdf.read_text(), flags=re.IGNORECASE)
        (temp / "elf4.urdf").write_text(urdf_text)
        for mesh in mesh_dir.glob("*.msh"):
            (temp / mesh.name).symlink_to(mesh)
        model = mujoco.MjModel.from_xml_path(str(temp / "elf4.urdf"))
        intermediate = temp / "imported.xml"
        mujoco.mj_saveLastXML(str(intermediate), model)
        root = ET.parse(intermediate).getroot()
        root.set("model", "elf4_sim2sim")
        compiler = root.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(root, "compiler")
        compiler.set("meshdir", "meshes")
        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", {"offwidth": "960", "offheight": "720"})
        asset = root.find("asset")
        if asset is None:
            asset = ET.SubElement(root, "asset")
        for mesh in asset.findall("mesh"):
            mesh.set("file", Path(mesh.get("file")).name)
        existing = {mesh.get("name") for mesh in asset.findall("mesh")}
        for mesh in mesh_dir.glob("*.msh"):
            if mesh.stem not in existing:
                ET.SubElement(asset, "mesh", {"name": mesh.stem, "file": mesh.name})

        world = root.find("worldbody")
        if world is None:
            raise ValueError("URDF import produced no worldbody")
        old_children = list(world)
        for child in old_children:
            world.remove(child)
        torso = ET.SubElement(world, "body", {"name": "torso_link", "pos": "0 0 1.35"})
        torso.append(ET.Element("freejoint", {"name": "torso_joint"}))
        urdf_inertial = ET.parse(urdf).getroot().find("link[@name='torso_link']/inertial")
        inertia = urdf_inertial.find("inertia")
        torso.append(ET.Element("inertial", {
            "pos": urdf_inertial.find("origin").get("xyz"),
            "mass": urdf_inertial.find("mass").get("value"),
            "fullinertia": " ".join(inertia.get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
        }))
        torso.extend(old_children)
        ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "size": "0 0 1",
                                      "contype": "2", "conaffinity": "1", "rgba": ".55 .57 .6 1",
                                      "friction": "0.9 0.02 0.002"})
        ET.SubElement(world, "light", {"name": "preview_light", "pos": "2 -3 5",
                                       "dir": "-0.3 0.3 -1", "diffuse": "1 1 1", "ambient": ".6 .6 .6"})
        for body in torso.iter("body"):
            name = body.get("name")
            if not name:
                continue
            for geom in list(body.findall("geom")):
                body.remove(geom)
            if (mesh_dir / f"{name}.msh").exists():
                ET.SubElement(body, "geom", {"name": f"{name}_visual", "type": "mesh", "mesh": name,
                                              "group": "1", "rgba": ".72 .79 .88 1", "contype": "0", "conaffinity": "0"})
            if name == "torso_link":
                add_collision(body, shape="box", pos="0 0 -0.10", size="0.11 0.12 0.18")
            elif name == "waist_z_link":
                add_collision(body, shape="box", pos="0 0 -0.15", size="0.075 0.105 0.12")
            elif name.endswith("_hip_z_link"):
                add_collision(body, shape="capsule", fromto="0 0 -0.05 0 0 -0.33", size="0.055")
            elif name.endswith("_knee_y_link"):
                add_collision(body, shape="capsule", fromto="0 0 -0.04 0 0 -0.36", size="0.04")
            elif name.endswith("_ankle_x_link"):
                add_collision(body, shape="box", pos="0.03 0 -0.011", size="0.12 0.044 0.024",
                              friction="1.0 0.03 0.003")
            elif name.endswith("_shoulder_z_link"):
                add_collision(body, shape="capsule", fromto="0 0 -0.02 0 0 -0.22", size="0.045")
            elif name.endswith("_elbow_y_link"):
                add_collision(body, shape="capsule", fromto="0 0 0 0.22 0 0", size="0.032")
            for joint in body.findall("joint"):
                # The URDF contains very light wrist/ankle links. Their zero
                # rotor inertia makes the Isaac-trained PD gains numerically
                # unstable at the training timestep in MuJoCo.
                joint.set("armature", "0.02")
        ET.indent(root)
        ET.ElementTree(root).write(output, encoding="unicode", xml_declaration=True)

    check = mujoco.MjModel.from_xml_path(str(output))
    if (check.nq, check.nv, check.nbody) != (36, 35, 31):
        raise ValueError(f"Unexpected ELF4 model dimensions: nq={check.nq}, nv={check.nv}, bodies={check.nbody}")
    if sum((check.geom_contype != 0) & (check.geom_conaffinity != 0)) < 13:
        raise ValueError("ELF4 deployment model is missing its collision shapes")
    print(f"Wrote {output} with {check.ngeom} geoms and {check.nbody} bodies")
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--max-faces", type=int, default=150_000)
    parser.add_argument("--urdf", type=Path, default=URDF)
    parser.add_argument("--source-mesh-dir", type=Path, default=SOURCE_MESH_DIR)
    args = parser.parse_args()
    build(args.output, args.max_faces, args.urdf, args.source_mesh_dir)
