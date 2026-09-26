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
env -u CUDA_VISIBLE_DEVICES uv run --locked graspdatagen generate --config configs/runs/gui-test.yaml --gui --renderer-gpu 4
```

Inspect exported results in a browser (no Isaac Sim required):

```bash
uv run python -m graspdatagen.web
```

Open http://127.0.0.1:8080. The Web viewer discovers `grasps-<robot>.yaml`
beside configured object USDs. `graspdatagen view` is for older run-directory
YAML with an adjacent `manifest.json`. Export compact YAML with
`graspdatagen export`, resume with `generate --resume`, or replay saved trials
with `graspdatagen replay`.

GPU commands must run on the host system. Prepared assets and run datasets are stored in `outputs/`; exports are written beside the source USD without modifying the USD.

## Project layout

`configs/` contains robot, gripper, and run definitions. Application code is in `src/graspdatagen/`, with the Web application grouped under `src/graspdatagen/web/`.

## Documentation

- [Installation and CLI](docs/installation.md)
- [Configuration ownership and migration](docs/configuration.md)
- [Web viewer](docs/web-viewer.md)
- [Gripper adaptation](docs/grippers.md)
- [Sampling and resume](docs/sampling.md)
- [Status and remaining work](docs/status.md)

中文说明：[README.zh-CN.md](README.zh-CN.md)。
