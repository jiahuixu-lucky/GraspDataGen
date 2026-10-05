"""Move YAM rigid bodies beneath the root, preserving world transforms."""
import argparse
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error(f"Output already exists: {args.output}")
original = Usd.Stage.Open(str(args.source.resolve()))
assert original, "Cannot open YAM source"
stage = Usd.Stage.Open(original.Flatten())
root = stage.GetDefaultPrim()
assert str(root.GetPath()) == "/urdf_top_assembly"

cache = UsdGeom.XformCache()
inverse = cache.GetLocalToWorldTransform(root).GetInverse()
mapping, transforms = {}, {}
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        old = prim.GetPath()
        new = root.GetPath().AppendChild(prim.GetName())
        assert not stage.GetPrimAtPath(new), f"Destination exists: {new}"
        mapping[old] = new
        transforms[new] = cache.GetLocalToWorldTransform(prim) * inverse

edit = Sdf.BatchNamespaceEdit()
for old, new in mapping.items():
    edit.Add(old, new)
assert stage.GetRootLayer().Apply(edit), "Body relocation failed"

def remap(path):
    for old, new in mapping.items():
        if path.HasPrefix(old):
            return path.ReplacePrefix(old, new)
    return path

for prim in stage.Traverse():
    for relation in prim.GetRelationships():
        targets = relation.GetTargets()
        updated = [remap(p) for p in targets]
        if updated != targets:
            relation.SetTargets(updated)
    for attribute in prim.GetAttributes():
        connections = attribute.GetConnections()
        updated = [remap(p) for p in connections]
        if updated != connections:
            attribute.SetConnections(updated)

for path, matrix in transforms.items():
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(matrix)

for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Joint):
        joint = UsdPhysics.Joint(prim)
        for relation in (joint.GetBody0Rel(), joint.GetBody1Rel()):
            for target in relation.GetTargets():
                assert stage.GetPrimAtPath(target), f"Missing joint body: {target}"

args.output.parent.mkdir(parents=True, exist_ok=True)
assert stage.GetRootLayer().Export(str(args.output.resolve()))
print("YAM normalization complete:", args.output)
