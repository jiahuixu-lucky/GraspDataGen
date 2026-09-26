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
Assets/Robots/x5/ARX.usd
Data/Assets/Object/Rigid/bubble_tea_cup/300g/Aligned.usd
```

Supply all 25 objects listed in `configs/objects/our_assets.yaml`, including
referenced layers, meshes, materials and textures. Object YAMLs are authoritative
for mass, friction, restitution and the original-frame up axis; separate metadata
JSON files are not required. Robot TCP definitions are in `configs/robots/`;
joint drives and limits come from USD, without a URDF dependency.

### Configuration map

- Current production: `configs/runs/production.yaml` selects
  `configs/objects/our_assets.yaml`.
- GUI diagnostic run: `configs/runs/gui-test.yaml` selects `objects: [shampoo_000]`
  from the same object manifest.
- Shared sampling, posture and validation: `configs/parameters.yaml`.
- Shared gripper calibration: `configs/calibration.yaml`.
- Legacy assets: `configs/objects/bottle_matryoshka.yaml`.
- Historical runs: `configs/experiments/`; these select `objects: [bottle]` and
  require the configured annotation directory. They are not ready-to-run production presets.
- Audit counterexamples: `configs/audit.yaml`, a separate schema from generation.

See [configuration ownership, overrides and migration](configuration.md).

## Generate and validate

```bash
uv run --locked graspdatagen generate --config configs/runs/production.yaml
uv run --locked graspdatagen generate --config configs/runs/production.yaml --resume

uv run --locked graspdatagen replay --run outputs/our-assets-production/piper--bubble_tea_cup_300g \
  --output outputs/replay/piper--bubble_tea_cup_300g.json
uv run --locked graspdatagen audit --run outputs/our-assets-production/piper--bubble_tea_cup_300g \
  --config configs/audit.yaml --output outputs/audit/piper--bubble_tea_cup_300g.json
```

The production YAML selects assets, cache, output, GPU, budgets and a shared
parameter file. Task-local section overrides contain only changed values.
It requests 2,048 environments and 1,024 distinct successes per object/gripper pair;
adjust resource and budget settings as needed. Generation prepares missing caches
automatically and retains native PhysX collision settings.

Each object/gripper pair stops at its success target or candidate, time or
sampling-round limit. The pair's time budget excludes asset preparation and is
checked between batches; it is not a hard deadline for the entire command.
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
Both preparation entrypoints use the gripper's calibration step rate. `inspect`
reads the USD without starting a simulation runtime or requiring `nvidia-smi`.
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
Isaac Sim can match devices. The `gui-test.yaml` example uses CUDA device 1
and Vulkan device 4 to inspect `shampoo_000`:

```bash
env -u CUDA_VISIBLE_DEVICES uv run --locked graspdatagen generate \
  --config configs/runs/gui-test.yaml --gui --renderer-gpu 4
```

This diagnostic run keeps its final frame open until the window is closed.
GUI replay arranges grasps in a grid; use headless replay to compare the original
saved world placements. Closing the GUI during validation aborts the run.

Generation exports `grasps-<robot>.yaml` beside the source object USD
automatically (for example, `grasps-piper.yaml` and `grasps-arx-x5.yaml`).
Re-exporting replaces the same robot’s file; different robots have separate
files. To re-export:

```bash
uv run --locked graspdatagen export --run outputs/our-assets-production/piper--bubble_tea_cup_300g
```

`view` only supports older run-directory YAML with an adjacent `manifest.json`;
it uses recorded joint positions and prepared assets without advancing physics.
Its candidate IDs are zero-based YAML IDs, unlike
replay's saved IDs. To inspect exported YAML directly beside the source USD, use the
[Web viewer](web-viewer.md).

## Development checks

```bash
uv run --locked ruff check src/graspdatagen
uv run --locked mypy
uv build --no-sources
```

Validate behavior through real preparation, generation and fresh-process replay,
not unit tests or mock simulation.
