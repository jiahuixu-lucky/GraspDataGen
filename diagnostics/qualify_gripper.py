"""Qualify one gripper in a new isolated output directory."""
import argparse
import sys
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--device", type=int, default=0)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
sys.argv = [sys.argv[0], f"--/log/file={args.output.resolve() / 'kit.log'}"]

from graspdatagen.assets import check_cache, write_json
from graspdatagen.config import load_gripper
from graspdatagen.grippers import prepare_gripper
from graspdatagen.runtime import PhysxRuntime, RuntimeConfig

config = load_gripper(args.config)
runtime = PhysxRuntime(RuntimeConfig(args.device, config.calibration.steps_per_second))
try:
    prepared = prepare_gripper(runtime, config, args.output / "cache")
    check_cache(prepared)
    write_json(args.output / "result.json", {
        "passed": True,
        "prepared": str(prepared),
    })
    print("Qualification passed:", config.name, flush=True)
except BaseException as error:
    write_json(args.output / "result.json", {
        "passed": False,
        "error": str(error),
    })
    runtime.close(exit_code=1)
    raise
runtime.close()
