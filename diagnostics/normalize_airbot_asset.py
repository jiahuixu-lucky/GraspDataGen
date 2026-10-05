"""Bake the Airbot left finger reflection into its mesh."""
import argparse
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, Vt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error(f"Output already exists: {args.output}")
source = Usd.Stage.Open(str(args.source.resolve()))
if not source:
    parser.error(f"Cannot open source: {args.source}")
stage = Usd.Stage.Open(source.Flatten())

parent = stage.GetPrimAtPath("/airbot_play/left/gripper_1")
mesh = UsdGeom.Mesh(
    stage.GetPrimAtPath("/airbot_play/left/gripper_1/gripper_1")
)
assert parent and mesh, "Unexpected Airbot mesh layout"
scale = parent.GetAttribute("xformOp:scale")
assert tuple(scale.Get()) == (1, -1, 1), "Unexpected reflection"
assert not mesh.GetNormalsAttr().Get(), "Authored normals need reflection handling"

points = mesh.GetPointsAttr().Get()
reflected = Vt.Vec3fArray([
    Gf.Vec3f(p[0], -p[1], p[2]) for p in points
])
counts = mesh.GetFaceVertexCountsAttr().Get()
indices = list(mesh.GetFaceVertexIndicesAttr().Get())
reversed_indices = []
cursor = 0
for count in counts:
    reversed_indices.extend(reversed(indices[cursor:cursor + count]))
    cursor += count
assert cursor == len(indices)

mesh.GetPointsAttr().Set(reflected)
mesh.GetFaceVertexIndicesAttr().Set(reversed_indices)
mesh.GetExtentAttr().Set(Vt.Vec3fArray([
    Gf.Vec3f(*(min(p[i] for p in reflected) for i in range(3))),
    Gf.Vec3f(*(max(p[i] for p in reflected) for i in range(3))),
]))
scale.Set(Gf.Vec3d(1, 1, 1))
args.output.parent.mkdir(parents=True, exist_ok=True)
assert stage.GetRootLayer().Export(str(args.output.resolve()))
print("Airbot normalization complete:", args.output)
