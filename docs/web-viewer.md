# Web Grasp Viewer

Run from the repository root after [installation](installation.md):

```bash
uv run --locked python -m graspdatagen.web
```

Open http://127.0.0.1:8080. The server reads assets on the CPU without starting
Isaac Sim or CUDA; the browser renders with Three.js.

By default, it discovers `grasps-<robot>.yaml` beside object USDs listed in
`configs/objects/our_assets.yaml`, plus run datasets under `outputs/` that contain
grasp poses. Standalone output YAML without a manifest must match the selected
object configuration. To select a dataset explicitly:

```bash
uv run --locked python -m graspdatagen.web \
  --grasps Data/Assets/Object/Rigid/bubble_tea_cup/300g/grasps-piper.yaml
```

Use `--port` to change the port and `--host 0.0.0.0` for remote access.
The viewer has no authentication and can browse server files; use a trusted
network or SSH port forwarding.

## Dataset requirements

YAML files with an adjacent `manifest.json` use its prepared caches. Exported
`grasps-<robot>.yaml` files contain no manifest path and resolve source assets through
`--objects` (default `configs/objects/our_assets.yaml`) and `--grippers`
(default `configs/grippers/*.yaml`). Select the older bottle/matryoshka collection
with `--objects configs/objects/bottle_matryoshka.yaml`.

Poses use the object-root frame, metres and Z up. Fingers use recorded closed
joint positions. Source visuals take precedence over collision meshes; unsupported
material networks fall back to display colors. This viewer inspects poses;
use `graspdatagen replay` for physical validation.

## Controls

- **Dataset:** select a dataset or use the folder button to open YAML on the server.
- **Single grasp / All grasps:** inspect one pose or the full distribution. All mode
  simplifies meshes to 3,000 triangles each (`--overview-faces`); selected grasps
  retain full geometry.
- **Navigation:** previous/next, zero-based candidate ID, timeline, playback or click a grasp.
- **Display:** opacity, wireframe, object visibility, TCP axes and grid.
- **Camera:** orbit, pan, zoom, fit, front/top views; the camera button downloads a PNG.
- **Annotate:** paint allowed surface regions; Shift+click erases. Annotations are
  stored under `annotations/grasp_regions` (`--grasp-regions` overrides it).

Each tab has independent view state. Restart the server after changing loaded
files on disk.
