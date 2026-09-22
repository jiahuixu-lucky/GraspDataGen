# Installation and CLI

## Setup

Run from the repository root on Linux x86_64 with Python 3.12, [uv](https://docs.astral.sh/uv/),
and a supported NVIDIA GPU/driver. Dependencies are pinned in `uv.lock`.

```bash
uv sync --locked
uv run --locked graspdatagen --help
```

The root `.venv` contains both simulation and Web dependencies; no `PYTHONPATH`
or external IsaacLab checkout is needed. Use `--no-dev` with `uv sync` for runtime only.
GPU commands must run outside the agent sandbox.

Package import defaults to `OMNI_KIT_ACCEPT_EULA=YES`, `OPENBLAS_NUM_THREADS=1`
and `OMP_NUM_THREADS=8`, preserving existing environment values. Running Isaac Sim
requires agreement to its EULA. Custom scripts should import `graspdatagen` before
numerical libraries for the thread defaults to apply.

## Assets

Place robot assets under `Assets/` and objects under `Data/our_Assets/` (symlinks
are supported), or update the YAML paths. Source assets are read-only inputs;
prepared assets go to `outputs/prepared/`.

```text
Assets/Robots/piper/Piper.usd
Assets/Robots/piper/piper_description/urdf/piper.urdf
Assets/Robots/x5/ARX.usd
Assets/Robots/x5/X5A.urdf
Data/our_Assets/bubble_tea_cup/300g/Aligned.usd
Data/our_Assets/bubble_tea_cup/300g/metadata.json
```

Supply all 25 objects listed in `configs/objects/our_assets.yaml`, including
referenced layers, meshes, materials and textures. The three bubble tea cup variants
require their source `metadata.json`; the other metadata files are included in
`configs/objects/our_assets_metadata/`. Robot TCP definitions are in `configs/robots/`.
The older bottle/matryoshka collection uses `configs/objects/production.yaml`.

## Generate and validate

```bash
uv run --locked graspdatagen generate --config configs/runs/production.yaml
uv run --locked graspdatagen generate --config configs/runs/production.yaml --resume

uv run --locked graspdatagen replay --run outputs/our-assets/piper--bubble_tea_cup_300g \
  --output outputs/replay/piper--bubble_tea_cup_300g.json
uv run --locked graspdatagen audit --run outputs/our-assets/piper--bubble_tea_cup_300g \
  --config configs/runs/audit.yaml --output outputs/audit/piper--bubble_tea_cup_300g.json
```

The production YAML controls assets, cache, output, GPU, sampling and validation.
It requests 512 environments and 1,024 distinct successes per object/gripper pair;
adjust resource and budget settings as needed. Generation prepares missing caches
automatically and retains native PhysX collision settings.

Generation stops at the success target or a candidate, time or sampling-round limit.
`insufficient_valid_grasps` and `asset_invalid` do not mean target completion.
An existing output requires `--resume`; changed inputs, configuration or code require
new output. Keep prepared caches with datasets for replay and viewing.

Replay defaults to one environment and all saved grasps; `--grasp-id` selects a
saved candidate. Audit supports `--case` for an individual check. Both accept
`--device` (default 0); see each command's `--help` for options.

For asset inspection or cache preparation without generation:

```bash
uv run --locked graspdatagen inspect --manifest configs/objects/our_assets.yaml \
  --output outputs/inspect.json
uv run --locked graspdatagen prepare --manifest configs/objects/our_assets.yaml \
  --gripper configs/grippers/piper.yaml --output outputs/prepare-piper.json
```

`prepare --cache` overrides `outputs/prepared`; use the same `cache` in the run YAML.
`prepare` stops on invalid assets; `generate` records them and continues.

### Multiple GPUs

```bash
uv run --locked graspdatagen generate-parallel \
  --config configs/runs/production.yaml --devices 0,1
```

Grippers are split across GPUs, with one process and `<output>-gpu<N>` directory
per GPU. There must be at least one gripper per GPU. Shared preparation caches
are locked; shard configurations and logs are saved in the printed
`<output>.parallel-<unique>/` directory. Replay and audit use the shard output path.

Resume with `--resume`, unchanged configuration and the same device order.
If interrupted before a shard initializes, start that shard with `generate`
and its saved YAML first.

## GUI and export

Add `--gui` to `generate`, `replay` or `audit` from a graphical desktop with valid
`DISPLAY` and X11 authorization. CUDA physics and Vulkan renderer GPU indices can
differ; select rendering with `--renderer-gpu`. Unset `CUDA_VISIBLE_DEVICES` so
Isaac Sim can match devices. On the current workstation, `book_gui.yaml` uses
CUDA device 1 and Vulkan device 4:

```bash
env -u CUDA_VISIBLE_DEVICES uv run --locked graspdatagen generate \
  --config configs/runs/book_gui.yaml --gui --renderer-gpu 4
```

This diagnostic book run keeps its final frame open until the window is closed.
GUI replay arranges grasps in a grid; use headless replay to compare the original
saved world placements. Closing the GUI during validation aborts the run.

Generation exports `grasps.yaml` automatically. To re-export or inspect static poses:

```bash
uv run --locked graspdatagen export --run outputs/our-assets/piper--bubble_tea_cup_300g
uv run --locked graspdatagen view \
  --grasps outputs/our-assets/piper--bubble_tea_cup_300g/grasps.yaml --candidate 0
```

`view` uses recorded joint positions, the adjacent `manifest.json` and prepared
assets; no physics advances. Its candidate IDs are zero-based YAML IDs, unlike
replay's saved IDs. For browser inspection, see [Web viewer](web-viewer.md).

## Development checks

```bash
uv run --locked ruff check src/graspdatagen
uv run --locked mypy
uv build --no-sources
```

Validate behavior through real preparation, generation and fresh-process replay,
not unit tests or mock simulation.
