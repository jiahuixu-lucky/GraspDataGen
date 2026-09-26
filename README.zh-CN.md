# GraspDataGen 中文说明

GraspDataGen 基于 Isaac Sim 6.0.1、PhysX 和 Warp，从刚体 USD/USDZ 资产生成并验证平行夹爪抓取数据集。

## 快速开始

```bash
uv sync --locked
uv run graspdatagen generate --config configs/runs/production.yaml
```

常用命令：

```bash
uv run graspdatagen export --run outputs/our-assets-production/piper--bubble_tea_cup_300g
uv run python -m graspdatagen.web
```

Web 查看器地址为 http://127.0.0.1:8080，默认发现物体 USD 旁导出的
`grasps-<robot>.yaml`。Isaac Sim 的 `view` 命令只适用于旁边有 `manifest.json`
的旧版运行目录 YAML。恢复任务使用 `--resume`，重放任务使用
`graspdatagen replay`。涉及 GPU 的命令必须在宿主机运行；运行数据写入
`outputs/our-assets-production/`，机器人和物体资产分别放在 `Assets/` 和
`Data/our_Assets/`。

详细说明：

- [安装与 CLI](docs/installation.md)
- [配置归属、覆盖与迁移](docs/configuration.md)
- [Web 查看器](docs/web-viewer.md)
- [夹爪适配](docs/grippers.md)
- [采样与恢复](docs/sampling.md)
- [状态与待办](docs/status.md)
