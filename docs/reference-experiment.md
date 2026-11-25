# SmolVLA training and OpenArm adaptations

## Dataset and checkpoint

The reference dataset contains 60 demonstrations and 39,229 frames at 30 Hz.
Download links will be provided in the [README](../README.md#workflow).
The complete fine-tuned checkpoint includes weights, preprocessing/postprocessing
JSON files and normalization tensors; these must stay together for inference.

## Inputs and actions

- State: measured right J1–J7 angles (radians) plus right gripper position (metres).
- Images: main camera and right wrist camera, RGB 640×480. Saved preprocessing maps
  these to `camera1` and `camera2`, resizes and normalizes them.
- Language: `Put the cube in the basket`.
- Output: 50 absolute eight-value action targets at the training rate of 30 Hz.
  They are unnormalized joint/gripper positions, not VR poses or motor torques.
- Demonstration actions came from controller reference positions; observations
  came from measured feedback. IK is used by VR teleoperation during recording,
  not by the policy during execution.

## Training record

Fine-tuned `lerobot/smolvla_base` snapshot
`5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6`, with backbone
`HuggingFaceTB/SmolVLM2-500M-Video-Instruct`. This was fine-tuning, not training from
scratch. Saved original configurations are [model configuration](https://github.com/secorolab/openarm-smolvla/blob/main/configs/config.json) and
[training configuration](https://github.com/secorolab/openarm-smolvla/blob/main/configs/train_config.json); absolute paths in these archived files describe the
original experiment and should be adapted for a new run.

- 20,000 updates, batch size 8, seed 42; four data-loading workers.
- 54 training episodes (35,424 frames), six validation episodes:
  11, 16, 20, 23, 38, 54. See [episode split](https://github.com/secorolab/openarm-smolvla/blob/main/configs/split.json).
- AdamW, peak learning rate 0.0001, gradient clipping 10; cosine schedule.
  Saved schedule specifies warm-up 1,000 and decay 30,000; the training log reported
  rescaling to warm-up 666 and decay 20,000 for this run.
- Vision encoder frozen; expert-only training and state-projection training enabled.
- Software versions are listed in the [README](../README.md#tested-dependency-versions); additional provenance is preserved in
  `../configs/artifacts.json`.

### Adaptations for this robot

- Right arm only: state and action dimensions are both 8 (J1–J7 plus gripper).
  No left-arm observations or actions are included.
- Main and right-wrist camera keys are renamed to `camera1` and `camera2`.
  The inherited configuration also lists `camera3`, but it is absent from batches;
  `empty_cameras=0` means the inspected image preprocessing skips it.
- Saved normalization statistics and preprocessing/postprocessing are reused for
  deployment; joint angles remain radians and gripper positions remain metres.
- Episodes are split before sampling, keeping all six validation episodes outside
  the training set. The training wrapper corrects filtered row indices
  and preserves episode boundaries for action chunks. Synthetic and dataset checks
  passed before the recorded training run. The original wrapper is now saved as
  [training wrapper](https://github.com/secorolab/openarm-smolvla/blob/main/scripts/train_cube.py). Use the recorded training LeRobot commit for this wrapper:
  the recorded inference commit has an older sampler that does not accept its
  `seed` argument. This sampler is not used during policy inference.

### Training and inference environments

**Training and inference can run on the same computer.** Separate computers are
optional; choose hardware with sufficient GPU memory and compute for the workload.
For the two recorded software configurations, use separate Python environments and
LeRobot checkouts so their dependencies do not overwrite each other.

For training reproduction, use the fine-tuning commit listed in the README and
[training wrapper](https://github.com/secorolab/openarm-smolvla/blob/main/scripts/train_cube.py). For inference, use the tested inference configuration in the README.
Transfer the complete checkpoint directory only when using a different computer;
on one computer, point inference directly at the saved checkpoint.

The checkpoint passed strict loading, saved preprocessing/postprocessing, prediction
and physical execution checks in the recorded inference environment. Different
training and inference versions can work, but compatibility is not guaranteed for
arbitrary versions. Recheck loading and preprocessing after dependency upgrades.

## Data transformations and repository separation

The data follows this path:

1. VR teleoperation uses IK to produce robot commands. Recording stores measured
   joint positions as observations and controller reference positions as actions.
2. The app exports raw recordings to LeRobot data tables, metadata and camera videos.
   Exporting does not turn VR controller poses into model actions: robot joint and
   gripper targets are already the recorded actions.
3. Training renames `observation.images.main` to `observation.images.camera1` and
   `observation.images.right` to `observation.images.camera2`. Saved processors apply
   the model's image and numerical preprocessing. State and action each have eight
   values; no left-arm values are added.
4. Inference loads the saved processors with the fine-tuned weights. The output
   postprocessor converts predictions back to joint angles and gripper positions.
5. The policy plugin checks and bounds those targets before the ROS app sends them
   to hardware. Runtime clipping and contact holding are deployment behavior, not
   changes to the demonstration dataset or learned weights.

The VR/ROS repository retains the app, UI, hardware bridge, cameras, teleoperation,
homing and recording/export. This repository contains the training wrapper,
model inference, offline evaluation and guarded policy plugin. The model does not
perform IK or send CAN commands directly. See the README's
[repository boundary](../README.md#repository-boundary) and
[runtime tuning](../README.md#runtime-tuning-preserved-by-this-extraction).
