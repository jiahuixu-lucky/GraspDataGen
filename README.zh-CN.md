# GraspDataGen 中文说明

GraspDataGen 基于 Isaac Sim 6.0.1、PhysX 和 Warp，从刚体 USD/USDZ 资产生成并验证平行夹爪抓取数据集。

## 快速开始

```bash
uv sync --locked
uv run graspdatagen generate --config configs/runs/production.yaml
```

常用命令：

```bash
uv run graspdatagen export --run outputs/our-assets/piper--bubble_tea_cup_300g
uv run graspdatagen view --grasps outputs/our-assets/piper--bubble_tea_cup_300g/grasps.yaml
uv run python -m graspdatagen.web
```

Web 查看器地址为 http://127.0.0.1:8080。恢复任务使用 `--resume`，重放任务使用 `graspdatagen replay`。涉及 GPU 的命令必须在宿主机运行，机器人资产放在 `Assets/`，默认物体资产放在 `Data/our_Assets/`，生成内容写入 `outputs/our-assets/`。

详细说明：

- [安装与 CLI](docs/installation.md)
- [Web 查看器](docs/web-viewer.md)
- [夹爪适配](docs/grippers.md)
- [采样与恢复](docs/sampling.md)
- [状态与待办](docs/status.md)
