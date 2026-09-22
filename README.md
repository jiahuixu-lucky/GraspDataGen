# GraspDataGen

GraspDataGen generates validated parallel-jaw grasp datasets from rigid USD/USDZ assets using Isaac Sim 6.0.1, PhysX, and Warp.

## Requirements

- Linux x86_64, Python 3.12, and a supported NVIDIA GPU
- [uv](https://docs.astral.sh/uv/)
- Robot assets in `Assets/` and production objects in `Data/our_Assets/`

See [docs/installation.md](docs/installation.md) for installation and asset setup.

## Quick start

```bash
uv sync --locked
uv run graspdatagen generate --config configs/runs/production.yaml
```

Inspect results with Isaac Sim:

```bash
uv run graspdatagen view --grasps outputs/our-assets/piper--bubble_tea_cup_300g/grasps.yaml
```

Inspect results in a browser (no Isaac Sim required):

```bash
uv run python -m graspdatagen.web
```

Open http://127.0.0.1:8080. Export compact YAML with `graspdatagen export`, resume an interrupted generation with `generate --resume`, or replay saved trials with `graspdatagen replay`.

GPU commands must run on the host system. Generated assets and datasets are stored in `outputs/`; source assets are never modified.

## Project layout

`configs/` contains robot, gripper, and run definitions. Application code is in `src/graspdatagen/`, with the Web application grouped under `src/graspdatagen/web/`.

## Documentation

- [Installation and CLI](docs/installation.md)
- [Web viewer](docs/web-viewer.md)
- [Gripper adaptation](docs/grippers.md)
- [Sampling and resume](docs/sampling.md)
- [Status and remaining work](docs/status.md)

中文说明：[README.zh-CN.md](README.zh-CN.md)。
