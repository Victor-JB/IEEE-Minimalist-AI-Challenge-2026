"""Import the complete CAD hierarchy and X-ray materials from the animation branch.

No MuJoCo dependency: retains its body frames, hinge axes and gear couplings for
the browser renderer. Meshes are unchanged; manifest translations are centimetres.
"""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REF = "ef1b0d2057c5e449a0e02638a1ff7d5b58dd3cbd"


def source(path):
    return subprocess.check_output(["git", "show", f"{REF}:arm_mujoco/{path}"], cwd=ROOT)


def build():
    root = ET.fromstring(source("robot_arm_xray.xml"))
    materials = {m.get("name"): list(map(float, m.get("rgba", "1 1 1 1").split()))
                 for m in root.findall("asset/material")}
    meshes = {m.get("name"): m.get("file") for m in root.findall("asset/mesh")}
    bodies, parts = [], []

    def vec(node, key, default, scale=1):
        return [float(v) * scale for v in node.get(key, default).split()]

    def visit(node, parent=-1):
        index = len(bodies)
        joint = node.find("joint")
        bodies.append(dict(name=node.get("name"), parent=parent,
                           position=vec(node, "pos", "0 0 0", 100),
                           quaternion=vec(node, "quat", "1 0 0 0"),
                           joint=None if joint is None else dict(
                               name=joint.get("name"), axis=vec(joint, "axis", "0 0 1"),
                               position=vec(joint, "pos", "0 0 0", 100))))
        for geom in node.findall("geom"):
            if geom.get("mesh"):
                parts.append(dict(body=index, file=meshes[geom.get("mesh")],
                                  position=vec(geom, "pos", "0 0 0", 100),
                                  quaternion=vec(geom, "quat", "1 0 0 0"),
                                  color=materials[geom.get("material")],
                                  shell=geom.get("class") == "shell"))
        for child in node.findall("body"):
            visit(child, index)

    visit(root.find("worldbody/body"))
    couplings = [dict(joint=j.get("joint1"), source=j.get("joint2"),
                      coefficients=vec(j, "polycoef", "0 1 0 0 0"))
                 for j in root.findall("equality/joint")]
    dest = ROOT / "arm/cad"
    (dest / "source-xray.xml").write_bytes(source("robot_arm_xray.xml"))
    for file in sorted({p["file"] for p in parts}):
        (dest / file).write_bytes(source(f"meshes/{file}"))
    manifest = dict(model="luxoiii · ARM v29 · complete assembly", source=f"full-model-animations@{REF}",
                    units="cm (mesh vertices in mm)", bodies=bodies, parts=parts, couplings=couplings)
    (dest / "full-model.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (dest / "SOURCE.txt").write_text(
        f"Full Fusion ARM v29 CAD from full-model-animations@{REF}.\n"
        "full-model.json preserves every mesh instance, body, hinge and gear coupling in robot_arm_xray.xml.\n"
        "Includes base plate, motor, pinion, yaw spindle, all cams, cycloid plates, pins, links and head.\n"
        "Shells default to 25% opacity. Browser motion is kinematic visualization, not measured drive motion.\n"
        "STL units: millimetres; manifest translations: centimetres.\n"
        "Regenerate with python3 scripts/import_full_model.py. Legacy model.json is retained for compatibility.\n")
    print(f"Imported {len(bodies)} bodies, {len(parts)} parts, {len(meshes)} meshes, {len(couplings)} couplings")


if __name__ == "__main__":
    build()
