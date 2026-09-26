# Gripper Adaptation

The existing parallel-jaw implementation is shared by Piper and ARX-X5. Add a
configuration and real asset evidence for another gripper; do not add a model-name
branch to sampling or validation.

1. Provide the source USD and a portable `configs/robots/` snapshot containing
   the actual TCP parent frame, metre offset, `xyzw` rotation, active joint and
   finger body names. Drives and limits always inherit USD; there are no nullable
   actuator overrides, duplicate actuator joint lists or mandatory URDF inputs.
2. Define retained bodies, joints, TCP parent prim, both finger collider paths,
   approach/opening axes, `wrist_up_axis_base`, contact region and material source
   in `configs/grippers/`. The `calibration` path selects the shared
   `configs/calibration.yaml` file. Keep palm and fixed attachments that affect approach
   clearance. ARX retains the camera mount geometry without enabling a camera.
3. Run `uv run --locked graspdatagen prepare` with this configuration, an object
   manifest and an output report. Preparation remaps relationships, preserves
   dependencies, writes a pure articulation and measures active/mimic motion.
4. Inspect the generated `definition.json`, numeric `definition.npz`, derived
   `gripper.usdc` and inspection images in the gripper cache. The USD retains the
   selected source visual meshes, materials and native collision prims; cooked
   hull arrays support calibration and sampling only. Verify open/mid/closed
   states, finite limits, monotonic aperture, target-only two-finger contact,
   finite drive limits and convergence. Aperture comes from opposing contact
   surfaces, not rigid-body centres or twice a single-finger offset.
5. Check the TCP against the complete source robot, including the entire parent
   transform chain. Define the actual mounting-side up axis in the gripper
   configuration, then generate and replay complete physical trials in a new
   output directory. Report unsupported geometry or control limits explicitly.

TCP changes, modified source dependencies and material or geometry settings
invalidate the content-addressed prepared definition. Recalibrate and regenerate;
do not reuse candidates against a changed definition. The preparation report's
`gripper` path points to the calibrated assets under `outputs/prepared/grippers/`.
Open its `gripper.usdc` directly. Generated assets are local artifacts, governed
by their source asset licenses.

Franka Panda and the former bundled gripper models are not qualified adapters in
this version. Adding one requires the same real calibration, coordinate checks,
generation and independent replay.
