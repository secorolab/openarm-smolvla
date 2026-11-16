# OpenArm SmolVLA

Right-arm cube-pickup policy, runtime guards and model experiment records.
The task instruction is **Put the cube in the basket**.

## Repository boundary

- `vive-focus-vision-ros2`: Unity/VR application, ROS hardware, teleoperation,
  homing, cameras, desktop panel, recording and LeRobot dataset export.
- This repository: checkpoint inference, shadow testing, offline evaluation,
  policy timing/trajectory execution, prediction clipping, contact holding,
  policy tests and training/model provenance.
- `~/robot_ai`: raw episodes, exported datasets, checkpoints and Python environment.
  They are deliberately not copied into Git.

The ROS app provides `external_control.hpp`, a model-independent extension interface.
This repository builds a shared library implementing that interface. The app loads
it only when `external_control_library` is explicitly supplied. Ordinary app/UI
launches have no model dependency and no policy services. Faults and operator stops
still reach the extension; VR, homing and external control cannot own the robot
simultaneously. The extension is trusted native code and must be rebuilt alongside
interface changes. Existing `policy_enable`, `policy_plan` and `policy_status`
service/topic names are preserved when this plugin is loaded.

## Existing data and fine-tuned model

| Artifact | Location | Size |
|---|---|---|
| All 60 raw demonstrations | `~/robot_ai/episodes/right_cube_30hz/episodes/` | See `configs/artifacts.json` |
| Training export | `~/robot_ai/episodes/right_cube_30hz/exports/ccf6d66fb41b4a11a30a459426888864/` | about 122 MiB on disk |
| Downloaded fine-tuned model | `~/robot_ai/models/smolvla_cube_20k/` | about 865 MiB |

The retained export has 60 episodes and 39,229 frames at 30 Hz. Both `dataset` and
`dataset-latest` point to it. Five older export snapshots were removed at the user's
request; `docs/removed-export-snapshots.json` records their identities. Raw episodes
were not deleted. Recording/export still creates new snapshots through the app.

The checkpoint's `model.safetensors` is the fine-tuned model downloaded from the
workstation, not a fresh base-model download. The saved preprocessing/postprocessing
JSON and normalizer tensors next to it are required for inference.

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
scratch. Saved original configurations are `configs/config.json` and
`configs/train_config.json`; workstation paths in them are historical.

- 20,000 updates, batch size 8, seed 42; four data-loading workers.
- 54 training episodes (35,424 frames), six validation episodes:
  11, 16, 20, 23, 38, 54. See `configs/split.json`.
- AdamW, peak learning rate 0.0001, gradient clipping 10; cosine schedule.
  Saved schedule specifies warm-up 1,000 and decay 30,000; the training log reported
  rescaling to warm-up 666 and decay 20,000 for this run.
- Vision encoder frozen; expert-only training and state-projection training enabled.
- Workstation run: `/media/ext4-data/share/rei/smolvla_cube/outputs/cube_20261109_163032/run`.
- Workstation reported PyTorch 2.11.0+cu126, Transformers 5.5.4. Laptop reported
  PyTorch 2.10.0+cu128, Transformers 5.3.0. Those are observations, not a new install recipe.
- Workstation LeRobot: **0.5.2**, commit
  `cec8ee0be6e1006f9ecbd829fea59d051a11c515`, reported after training.
  Git shows no tracked changes and two untracked files: `sitecustomize.py` and
  `test_libero_render.py`. Their contents/runtime involvement have not been checked.
  This identifies the training revision if the installation was unchanged since training.
- The separate laptop LeRobot commit is recorded in `configs/artifacts.json`.

### Adaptations for this robot

- Right arm only: state and action dimensions are both 8 (J1–J7 plus gripper).
  No left-arm observations or actions are included.
- Main and right-wrist camera keys are renamed to `camera1` and `camera2`.
  The inherited configuration also lists `camera3`, but it is absent from batches;
  `empty_cameras=0` means the inspected image preprocessing skips it.
- Saved normalization statistics and preprocessing/postprocessing are reused for
  deployment; joint angles remain radians and gripper positions remain metres.
- Episodes are split before sampling, keeping all six validation episodes outside
  the training set. The workstation's `train_cube.py` corrects filtered row indices
  and preserves episode boundaries for action chunks. Synthetic and dataset checks
  passed before the recorded training run. The original wrapper is now saved as
  `scripts/train_cube.py`. Its self-test currently fails in the laptop environment
  because that older LeRobot sampler does not accept `seed`; keep the original
  workstation version for training reproduction. Policy inference is unaffected.

### Workstation training versus laptop testing

This checkpoint was fine-tuned on the workstation using its existing LeRobot
installation at `/media/ext4-data/share/rei/lerobot` and Conda environment `lerobot`.
Repeat training there using that environment and `scripts/train_cube.py`. The
reported workstation version is LeRobot 0.5.2 at commit
`cec8ee0be6e1006f9ecbd829fea59d051a11c515`. This was captured after training; it
represents the training revision provided the checkout has not changed since then.
The laptop commit in `configs/artifacts.json` is a different installation.
A follow-up Python check from the workstation home directory in Conda `lerobot`
reported `Loaded sitecustomize: None`. It was not loaded in that session; its
contents and whether it loaded during the original training run remain unverified.

The laptop is used for inference and physical robot testing, not retraining.
Its LeRobot version differs: the training wrapper's `seed` argument is unsupported
by its sampler. That sampler is not used during policy inference. The checkpoint
has already passed strict loading, saved preprocessing/postprocessing, prediction
and physical execution checks on this laptop. This establishes compatibility for
the tested setup, not for arbitrary future library versions. Preserve the working
laptop environment and revalidate checkpoint loading and preprocessing after upgrades.

## Build

The app and plugin must use compatible ROS Jazzy/C++ dependencies. This does not
install LeRobot or download weights. Keep using `~/robot_ai/lerobot_env`.

Build the app normally after pulling its external-controller-interface change.
Then build this plugin:

```sh
source /opt/ros/jazzy/setup.zsh
cmake -S ~/Desktop/openarm-smolvla -B ~/Desktop/openarm-smolvla/build \
  -DVR_APP_ROOT="$HOME/Desktop/vive-focus-vision-ros2"
cmake --build ~/Desktop/openarm-smolvla/build -j2
ctest --test-dir ~/Desktop/openarm-smolvla/build --output-on-failure
```

## Run a supervised policy trial

1. Stop any old policy and robot launch. Use the app panel's CAN setup as usual.
2. Start cameras from the app panel; do not record during a policy trial.
3. In a separate terminal, grant scheduling priority if needed, then start hardware
   with the policy plugin. Replace the example IP with the laptop's current Wi-Fi IP.
   Leave this terminal running. Do not also click the panel's Start VR robot button.

```sh
sudo prlimit --pid $$ --rtprio=50:50
bash ~/Desktop/openarm-smolvla/scripts/start_robot.sh 10.23.1.157
```

This starts hardware held at its measured pose; it does not start model motion.
Use the app's existing return-to-zero command if the demonstrated start pose is
needed; clear the direct path first. Leave VR disabled. Then, in another terminal:

```sh
source /opt/ros/jazzy/setup.zsh
source ~/robot_ai/lerobot_env/bin/activate
python ~/Desktop/openarm-smolvla/scripts/openarm_smolvla_run.py --execute --seconds 60
```

Ctrl+C stops policy execution. No collision avoidance or automatic success detector
is implemented. The model may reopen the gripper prematurely. Holding torque is not
guaranteed on driver faults. Use `openarm_smolvla_shadow.py --seconds 60` for predictions
without motion; it needs cameras and feedback but does not acquire control.

## Runtime tuning preserved by this extraction

No new gripping-force changes were made during the split.

- Arm command speed 0.30 rad/s, acceleration 0.6 rad/s².
- Gripper command speed 0.010 m/s, acceleration 0.05 m/s².
- Joint limits enforced. Small model overshoots are clipped; larger ones rejected.
- Gripper raw predictions from −3 mm to 49 mm may be clipped to 0–44 mm commands.
  This never expands the physical commanded range.
- Closing stops advancing at 3 mm of position lead. Rebound up to 6 mm is tolerated;
  opening errors above 4 mm still stop. This is position-based contact tolerance,
  not force sensing and not a guarantee of secure grasp.
- Input freshness 150 ms and alignment 80 ms; inference plans at most 650 ms old.
  Accepted-plan watchdog 750 ms; expired/replayed chunks cannot renew control.
- Warm-up output is discarded before acquiring control and recapturing input.
- Detailed age/shape/following errors are logged; no automatic restart after faults.

See `docs/openarm_smolvla_run.md` and `docs/openarm_smolvla_shadow.md`.

## Tests and evaluation

`tests/policy_guard_check.cpp` covers clipping, rates, contact hold and opening.
`tests/openarm_policy_check.py APP_BINARY PLUGIN_LIBRARY` uses isolated ROS domain
198 with synthetic feedback and no CAN hardware to check ownership, faults,
watchdogs and malformed input. The app keeps its own homing, gripper and recorder tests.
`python scripts/openarm_validate_smolvla.py` evaluates held-out recordings without
robot interfaces. The saved validation report in `docs/` is offline evidence only;
it does not establish real-world task success.
