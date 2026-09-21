# Installation and CLI

## Environment

Run from the repository root on Linux x86_64 with Python 3.12, `uv`, a supported
NVIDIA GPU/driver and enough disk space for the Isaac Sim distribution and asset
cache. The runtime was verified on an RTX 5090 with driver 595.84. GPU commands must run
outside the agent sandbox. Do not install a separate `usd-core`: USD/PhysX plugins
must come from the same Isaac Sim release.

```bash
uv sync --locked
uv run --locked graspdatagen --help
```

`uv` creates the root `.venv` and installs this project, including the
`graspdatagen` console entry point and `physx.kit` runtime resource. No
`PYTHONPATH`, IsaacLab checkout or external Python environment is needed. Deactivate
an older virtual environment before syncing. The default development group includes
Ruff, mypy and YAML typing stubs; use `uv sync --locked --no-dev` for runtime only.

The locked runtime is Isaac Sim 6.0.1.0, NumPy 2.3.1, Warp 1.13.0, Torch
2.10.0/CUDA 12.8, trimesh 4.11.1, SciPy 1.17.0, Matplotlib 3.10.8
and PyYAML 6.0.3. Matplotlib produces inspection
and diagnostic images. The verified overrides retain torchvision 0.25.0 and
torchaudio 2.10.0 only as Isaac Sim transitive dependencies. The full Sim
distribution may also install Newton packages; project execution selects PhysX.
Isaac Sim also depends on CoACD transitively; GraspDataGen does not call it.
`uv.lock` preserves the already qualified dependency versions and index sources.

## Assets

Supply robot assets under `Assets/` and the production object collection under
`Data/our_Assets/`, or adjust the project YAML paths to an equivalent asset tree.
Both `Assets/` and `Data/` may be local symlinks and are ignored by Git. Source
files are read-only inputs; derived files go into `outputs/prepared/`.

```text
Assets/Robots/piper/Piper.usd
Assets/Robots/piper/piper_description/urdf/piper.urdf
Assets/Robots/x5/ARX.usd
Assets/Robots/x5/X5A.urdf
Data/our_Assets/bubble_tea_cup/300g/Aligned.usd
Data/our_Assets/bubble_tea_cup/300g/metadata.json
Data/our_Assets/book/000/Aligned.usd
```

These object paths are examples. The default `configs/runs/production.yaml` uses
`configs/objects/our_assets.yaml`, which lists all 25 required object USD paths.
Supply the complete collection and keep its referenced layers, meshes, materials
and textures. The three bubble tea cup variants (300g, 500g and 800g) also require
their original `metadata.json` files. The other 22 metadata files are included in
`configs/objects/our_assets_metadata/`; their mass and friction mirror the source
USD values. The manifest explicitly selects each object's physical parameters.

The older bottle and matryoshka collection remains available through
`configs/objects/production.yaml`. To use it, change the run YAML's `manifest`
and choose a new `output` directory.

The portable `configs/robots/` snapshots define authoritative TCPs and inherit
drive settings from USD. They do not depend on the original configuration checkout.

## Commands

Accept the Isaac Sim EULA according to your installation's licensing requirements
before setting the environment variable below. These CPU thread limits are the
settings used for real qualification:

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=8

uv run --locked graspdatagen inspect --manifest configs/objects/our_assets.yaml \
  --output outputs/inspect.json

uv run --locked graspdatagen prepare --manifest configs/objects/our_assets.yaml \
  --gripper configs/grippers/piper.yaml \
  --output outputs/prepare-piper.json
uv run --locked graspdatagen prepare --manifest configs/objects/our_assets.yaml \
  --gripper configs/grippers/arx_x5.yaml \
  --output outputs/prepare-arx_x5.json

uv run --locked graspdatagen generate --config configs/runs/production.yaml
uv run --locked graspdatagen generate --config configs/runs/production.yaml --resume
uv run --locked graspdatagen generate-parallel \
  --config configs/runs/production.yaml --devices 0,1

uv run --locked graspdatagen replay --run outputs/our-assets/piper--bubble_tea_cup_300g \
  --environments 1 --output outputs/replay/piper--bubble_tea_cup_300g.json

uv run --locked graspdatagen audit --run outputs/our-assets/piper--bubble_tea_cup_300g \
  --config configs/runs/audit.yaml --output outputs/audit/piper--bubble_tea_cup_300g.json
```

`prepare` is an independent inspection/prewarm command; `generate` calls the same
preparation implementation when caches are missing. To check automatic preparation
from scratch, run `generate` first with a new cache path. The default preparation
directory is `outputs/prepared`; `--cache` selects a different directory when
isolating another dataset's prepared assets. The preparation report's `gripper`
path contains `gripper.usdc` and its calibration. No separate `bots/` aliases
are generated. The run YAML uses the same directory through its `cache` field.

The production run requests 512 environments and 1,024 distinct successes per
combination. These are requested limits, not completed performance or quantity
acceptance. Adjust the run YAML for available GPU resources and the desired
object manifest; validation thresholds should retain their physical meaning.
Collision geometry follows the source asset's native PhysX settings. Preparation
retains the complete selected gripper bodies, including their visual meshes,
materials and native collision prims. Cooked hulls are cached only for sampling
and calibration; no project decomposition or visual-surface error budget is
applied. PhysX still cooks the native collision shapes used by simulation.
`prepare` stops on an invalid asset; `generate` records it and continues with
the remaining objects.

`inspect`, `prepare`, `replay` and `audit` require `--output` and use GPU 0 unless
`--device` is provided. `generate` takes device and output from its YAML. Replay
defaults to one environment and all saved grasps; `--grasp-id` selects one positive
candidate ID. Audit defaults to all cases; `--case` selects an individual physical
counterexample or sensitivity run. `--help` lists the supported arguments.

`generate-parallel` splits the YAML's grippers across the requested GPUs, creates
one independent Isaac Sim process per GPU, and writes each shard under
`<output>-gpu<N>`. This keeps `multi_gpu` disabled and gives each shard its own
output lock. Each invocation saves its shard YAML files and logs in a new
`<output>.parallel-<unique>/` directory, printed at launch. Rejected or concurrent
launches therefore preserve previous configurations and logs. Startup errors and
interrupts stop and reap the launched supervisors and terminate their worker
process groups; processes still present after the shutdown grace period are killed.

Object preparation locks each cache entry so workers sharing a cold cache reuse
the first completed build. For replay or audit after parallel generation, use
the corresponding shard path, for example
`outputs/our-assets-gpu0/piper--bubble_tea_cup_300g` for the command above.
Use `--resume` with the same configuration and device order after an interrupted
parallel run. Each shard must already have an initialized run; a launch interrupted
before initialization may require starting its missing shards with `generate`
using their saved YAML files. The command requires at least one gripper per GPU.

### Live GUI

Add `--gui` to `replay`, `generate` or `audit` to watch the physical validation.
Run from a graphical desktop terminal (including a remote desktop) with its valid
`DISPLAY` and X11 authorization. An SSH terminal without a display cannot open
an interactive window; do not assume another user's display is accessible.

```bash
uv run --locked graspdatagen replay --run outputs/our-assets/piper--bubble_tea_cup_300g \
  --environments 16 --gui --output outputs/gui-replay/piper--bubble_tea_cup_300g.json
```

The overview camera frames all environments in the current batch. The viewport
shows the gripper's retained source visuals and the object's red collision geometry.
Standard viewport camera controls allow closer inspection; the toolbar's
Pause/Play controls pause and resume the physical execution. Stop aborts the run.
Rendering runs at
approximately 30 frames per simulated second, paced no faster than real time;
slow rendering can make playback slower. Physics timesteps and acceptance limits
are unchanged. The window closes after the requested trials finish; closing it
early aborts validation. GUI execution counts against generation's time budget.

GUI replay places each saved grasp in a separate grid cell, retaining its object
orientation, relative grasp poses, commands and disturbance conditions. This
prevents successes from different original batches overlapping on screen.
Its report records `layout: grid`; default headless replay retains the original
world positions (`layout: saved`) for reproducibility checks. World translations
can affect floating-point contact results, so use headless replay for comparison
with the original saved placement.

On a server, a virtual X display can exercise rendering for diagnostics, but it
is not itself a remotely viewable desktop. A remote desktop or X forwarding is
still required to interact with the GUI. Inactive physical desktop sessions may
fail Vulkan initialization even when X11 authentication succeeds.

Generation stops at the distinct-success target, candidate budget, elapsed budget
or sampling-round limit. A started batch always finishes the full protocol.
`insufficient_valid_grasps` and `asset_invalid` are explicit per-pair outcomes;
they must not be presented as successful target completion. An existing output
requires `--resume`; changed inputs, configuration or code require a new output.
Keep cache directories with datasets because replay resolves their saved prepared
asset paths. Old output is not silently upgraded or overwritten.

## Static Grasp Inspection

From a local or remote graphical desktop with `DISPLAY` set:

```bash
uv run --locked graspdatagen export --run outputs/our-assets/piper--bubble_tea_cup_300g
uv run --locked graspdatagen view \
  --grasps outputs/our-assets/piper--bubble_tea_cup_300g/grasps.yaml --candidate 0
```

New generation exports automatically. Re-export older YAML to include
`closed_joint_positions_m`, the measured closure position of each named joint.
The viewer reads grasp states directly from YAML and uses its adjacent
`manifest.json` to locate the prepared object, gripper and TCP definition.
Keep those prepared assets with the data. NPZ grasp shards are not needed by
the viewer. Candidate selection uses YAML's zero-based IDs, not replay IDs.

The object and gripper use their retained visual surfaces and materials.
Fixed/prismatic joint frames place each finger at its recorded state.
Physics does not advance, so the poses remain static while inspecting. Use the
previous/next buttons or numeric selector to change grasps and Frame Grasp to
reset the camera. Normal Isaac Sim viewport orbit, pan and zoom remain available.
This view is an inspection tool; use `replay` for physical revalidation.

## Checks

```bash
uv run --locked ruff check src/graspdatagen
uv run --locked mypy
uv build --no-sources
```

Static checks supplement real asset preparation, generation and fresh-process
replay. This project does not use unit tests or mock simulation for acceptance.
