# YAM and Airbot Play adapters

Normalize source assets with diagnostics/normalize_airbot_asset.py and diagnostics/normalize_yam_asset.py (--source and --output).
Outputs: outputs/adapter-assets/airbot_play.usdc and outputs/adapter-assets/yam.usdc.
Configs: configs/grippers/airbot_play.yaml and configs/grippers/yam.yaml.
Run diagnostics/qualify_gripper.py with --config, --output and --device.
Finger collider selectors support multiple meshes within a subtree.

Research validation on apple__050mm: both reached 1024 accepted grasps.
Airbot: 2048 attempts, 208.7 seconds. YAM: 2364 attempts, 458.2 seconds.
The run exited normally with no reported physics or rendering errors.
PR qualification is checked separately; other sizes and hardware remain unqualified.
