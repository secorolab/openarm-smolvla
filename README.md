# OpenArm SmolVLA

Right-arm cube-pickup policy, runtime guards and model experiment records.
The task instruction is **Put the cube in the basket**.

## Workflow

1. Install the [VR/ROS app](https://github.com/secorolab/vive-focus-vision-ros2) and set up OpenArm and the cameras.
2. Teleoperate the robot, record successful demonstrations, and export them to LeRobot format.
3. Clone this repository and install the model dependencies.
4. Fine-tune SmolVLA on the exported dataset.
5. Load the resulting checkpoint, check predictions without motion, then run a supervised robot trial.

All steps can run on one computer with suitable hardware. The app handles
teleoperation and recording; this repository handles training and policy execution.

**Example dataset:** download link will be added here when published. It contains
60 right-arm cube-to-basket demonstrations. You can use it instead of recording
your own. Model weights are separate from the dataset.

## Training and adaptation documentation

Read **[SmolVLA training and OpenArm adaptations](docs/reference-experiment.md)**
for how our checkpoint was trained: dataset split, right-arm and gripper inputs,
two-camera mapping, normalization, training settings, the sampling fix and the
separation between the model code and the VR/ROS app.

## Use our trained checkpoint

To try the cube-to-basket policy without recording or training, obtain our complete
fine-tuned checkpoint, install the dependencies below, and follow
[Run the trained policy](#run-the-trained-policy).

| Download | Contents | Availability |
|---|---|---|
| Fine-tuned SmolVLA checkpoint | Weights, model configuration, saved processors and normalization tensors | Download link to be added |
| LeRobot demonstration dataset | 60 episodes with robot states, actions and main/right-wrist camera videos | Download link to be added |

**The checkpoint was fine-tuned using 54 episodes out of 60; six were held out.**
It was not trained on 50 episodes. The task instruction is **Put the cube in the basket**.
The dataset is optional for running the checkpoint; it is needed for training or
offline evaluation. Cloning this repository does not download either artifact.

Extract the complete checkpoint into `~/robot_ai/models/smolvla_cube_20k/`, or pass
its directory with `--model`. Keep the processor JSON files and normalization
tensors alongside `model.safetensors`; the weights alone are not sufficient.

### What was adapted from SmolVLA?

- **One arm:** eight state/action values represent right J1–J7 and its gripper.
  Left-arm observations and actions are excluded; this is not removal of a physical
  arm or of the ROS hardware checks.
- **Two cameras:** main and right-wrist images map to `camera1` and `camera2`.
  The inherited configuration still lists `camera3`, but no third image is supplied;
  `empty_cameras=0` means it is skipped rather than filled with a dummy image.
- **Fine-tuning:** the pretrained SmolVLA base was adapted to the cube task for
  20,000 updates; it was not trained from scratch.
- **Deployment:** saved normalization and postprocessing produce robot joint/gripper
  targets, which pass through the guarded policy plugin before execution.

Details and source configurations:

- [Inputs, actions and camera mapping](docs/reference-experiment.md#inputs-and-actions)
- [Training settings and base model](docs/reference-experiment.md#training-record)
- [One-arm adaptations and sampling fix](docs/reference-experiment.md#adaptations-for-this-robot)
- [Data transformations and repository separation](docs/reference-experiment.md#data-transformations-and-repository-separation)
- [Saved model configuration](https://github.com/secorolab/openarm-smolvla/blob/main/configs/config.json), [training configuration](https://github.com/secorolab/openarm-smolvla/blob/main/configs/train_config.json)
  and [training/validation split](https://github.com/secorolab/openarm-smolvla/blob/main/configs/split.json)
- [Tested dependency versions](#tested-dependency-versions)

## Related repositories and dependencies

- [VR/ROS app](https://github.com/secorolab/vive-focus-vision-ros2): required ROS
  hardware bridge, cameras, teleoperation, UI and recording/export. Follow its setup
  instructions first; this repository does not replace the hardware workspace.
- [OpenArm SmolVLA](https://github.com/secorolab/openarm-smolvla): this policy repository.
- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/): ROS environment used
  by the robot app and the policy bridge.
- [OpenArm ROS 2](https://github.com/enactic/openarm_ros2) and
  [OpenArm CAN](https://github.com/enactic/openarm_can): upstream hardware software
  used in `~/openarm_ws`. Keep the locally tested guarded hardware configuration;
  these links are not instructions to replace it with unmodified upstream drivers.
- [LeRobot source](https://github.com/huggingface/lerobot) and
  [documentation](https://huggingface.co/docs/lerobot): training, dataset and model APIs.
- [SmolVLA base checkpoint](https://huggingface.co/lerobot/smolvla_base): starting
  weights used for fine-tuning. Robot trials use our fine-tuned local checkpoint.

### Source repositories

| Component | Repository |
|---|---|
| VR/ROS application, UI and recording | [secorolab/vive-focus-vision-ros2](https://github.com/secorolab/vive-focus-vision-ros2) |
| Model integration and policy | [secorolab/openarm-smolvla](https://github.com/secorolab/openarm-smolvla) |

The setup below uses paths relative to the current user's home directory; no
particular username or Desktop checkout is required.

### Connection to the robot app

```text
Main + wrist cameras -> app camera server (HTTP :8081) -> model inference
/joint_states + driver health ------------------------> model inference
model -> saved postprocessor -> ROS policy_plan -> policy plugin
      -> app command publishers -> guarded hardware driver -> CAN -> right arm
```

The model runs in `~/robot_ai/lerobot_env`. Its helper uses `/usr/bin/python3` with
ROS sourced, connecting on domain **84**, discovery **LOCALHOST**. Both processes
run on the robot-connected host, together with the camera server and ROS bridge.
The current runtime uses local discovery and same-host timestamps. The camera server
provides images and capture timestamps.
The policy plugin is loaded into the app bridge by `scripts/start_robot.sh`:

- `/openarm_hardware_preview/policy_enable`: acquire/release control (`SetBool`).
- `/openarm_hardware_preview/policy_plan`: timestamped trajectories with a session token.
- `/openarm_hardware_preview/policy_status`: active/stopped status and reasons.

No direct CAN commands are sent by the Python model. The ordinary VR app launch
has no policy plugin. See the build and run instructions below for policy trials.

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

## Setup

Commands below use **Bash on Ubuntu 24.04 with ROS 2 Jazzy**. In Zsh, either start
`bash` first or use the corresponding `setup.zsh` files. These are setup instructions,
not a claim that a fresh installation has been tested on every machine.

### 1. Clone the two repositories

The app belongs inside a ROS workspace; the policy repository is built separately.
You need GitHub access if these repositories or their dependencies are private.

```bash
mkdir -p ~/vive_vr_ws/src
git clone https://github.com/secorolab/vive-focus-vision-ros2.git \
  ~/vive_vr_ws/src/vive-vr-ros2
git clone https://github.com/secorolab/openarm-smolvla.git ~/openarm-smolvla
```

Skip cloning a repository that already exists. Use compatible revisions containing
`external_control.hpp` in the app and the matching policy plugin in this repository.

### 2. Prepare ROS, hardware and cameras

First follow the app's
[installation instructions](https://github.com/secorolab/vive-focus-vision-ros2/blob/HEAD/docs/install.md),
using `~/vive_vr_ws` as the workspace. Import its `dependencies.repos`; it includes
additional repositories accessed over SSH, so GitHub SSH access may also be needed.
Do not omit the project's KDL fork.

For the real robot, complete the app's
[OpenArm setup](https://github.com/secorolab/vive-focus-vision-ros2/blob/HEAD/docs/openarm_modes.md)
and [camera setup](https://github.com/secorolab/vive-focus-vision-ros2/blob/HEAD/docs/openarm_cameras.md).
The supplied launch helper expects the hardware workspace at `~/openarm_ws`, the
app at `~/vive_vr_ws`, and the prepared robot model at `~/vive_vr_ws/models/openarm_v1`.
The current setup uses OpenArm **v1**, `can0` for right and `can1` for left. Verify
adapter assignment and calibration on your own robot before enabling hardware.

Once these dependencies and hardware packages are installed, build the app:

```bash
source /opt/ros/jazzy/setup.bash
source ~/openarm_ws/install/setup.bash
cd ~/vive_vr_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_OPENARM_VR_BRIDGE=ON
source ~/vive_vr_ws/install/setup.bash
```

The hardware bridge needs both guarded drivers and the relevant controllers healthy,
even though the model controls only the right arm. Confirm ordinary app feedback,
cameras and homing work before trying the policy. The headset is needed for VR
recording, not for model inference.

### 3. Install model dependencies

Use the tested **inference commit**, not an unpinned latest LeRobot checkout.
Keep this environment separate from the app's ROS/setup virtual environment. These
commands are for a new environment; do not overwrite an existing working one.

```bash
mkdir -p ~/robot_ai
python3 -m venv ~/robot_ai/lerobot_env
source ~/robot_ai/lerobot_env/bin/activate
python -m pip install --upgrade pip
git clone https://github.com/huggingface/lerobot.git ~/robot_ai/lerobot
git -C ~/robot_ai/lerobot checkout d656da8ccca5989ff0a2207e81fbfa2c2d5bafb1
python -m pip install torch==2.10.0 torchvision==0.25.0 \
  --index-url https://download.pytorch.org/whl/cu128
python -m pip install -e "$HOME/robot_ai/lerobot[smolvla,dataset]" av pillow
python -c "import torch; print(torch.__version__); print('CUDA:', torch.cuda.is_available())"
```

A compatible NVIDIA driver/GPU is required by this runner, which currently selects
CUDA. The ROS helper uses system Python with ROS sourced; do not install a second
ROS distribution into the model environment. First model loading may fetch backbone
configuration/tokenizer assets from Hugging Face even when weights are local.

### 4. Record and export demonstrations

Use the app's [recording controls](https://github.com/secorolab/vive-focus-vision-ros2/blob/HEAD/docs/openarm_recording.md)
with the real robot launch running. For the 30 Hz dataset used here, start one
camera/recorder process with explicit rates:

```bash
source /opt/ros/jazzy/setup.bash
source ~/openarm_ws/install/setup.bash
source ~/vive_vr_ws/install/setup.bash
/usr/bin/python3 ~/vive_vr_ws/src/vive-vr-ros2/scripts/openarm_cameras.py \
  --fps 60 --record-fps 30 \
  --record-root ~/robot_ai/episodes/right_cube_30hz \
  --task "Put the cube in the basket"
```

Record a successful pickup and placement, stop, then save each attempt. Once the
session is finished, use **Export saved episodes to LeRobot** in the app panel.
Saving preserves raw demonstrations; exporting creates the training dataset.
Use the export directory reported by the panel, containing `meta/`, `data/` and
`videos/`. Do not point training at the raw `episodes/` directory.

The model uses main and right-wrist images plus eight measured values (right-arm
J1–J7 in radians and gripper position in metres). Training actions are the eight
controller reference positions. Keep camera placement, units, joint order and
30 Hz sampling consistent for recording and deployment.

If using the example dataset, skip recording and extract the converted dataset
to `~/robot_ai/datasets/openarm_cube/` once its download link is available.

### 5. Fine-tune SmolVLA

Use a separate training environment and checkout to preserve the inference
installation. This is an environment separation, not a requirement for two devices.
The version table below records the tested dependencies.

```bash
python3 -m venv ~/robot_ai/training_env
source ~/robot_ai/training_env/bin/activate
git clone https://github.com/huggingface/lerobot.git ~/robot_ai/lerobot_training
git -C ~/robot_ai/lerobot_training checkout cec8ee0be6e1006f9ecbd829fea59d051a11c515
python -m pip install -e "$HOME/robot_ai/lerobot_training[smolvla,training]" av
python ~/openarm-smolvla/scripts/train_cube.py --self-test
```

Installing the extras resolves dependencies; compare the resulting versions with
the tested table before reproducing the experiment. The wrapper fixes sampling
indices for episode subsets and must pass its self-test before training.

Prepare a configuration from the archived recipe. Set `dataset_root` to your actual
export. This creates a deterministic episode split and a new output directory;
it does not change the dataset or archived configuration.

```bash
python - <<'PYCONFIG'
import json
import random
from datetime import datetime
from pathlib import Path

home = Path.home()
dataset_root = home / "robot_ai/datasets/openarm_cube"
repo = home / "openarm-smolvla"
info = json.loads((dataset_root / "meta/info.json").read_text())
assert info["fps"] == 30, "This recipe expects 30 Hz demonstrations"
episodes = list(range(info["total_episodes"]))
assert len(episodes) >= 2, "Need training and validation episodes"
random.Random(42).shuffle(episodes)
n_validation = max(1, len(episodes) // 10)
run = home / "robot_ai/training" / datetime.now().strftime("cube_%Y%m%d_%H%M%S")
run.mkdir(parents=True, exist_ok=False)
split = {"train": sorted(episodes[n_validation:]), "validation": sorted(episodes[:n_validation])}
config = json.loads((repo / "configs/train_config.json").read_text())
config["dataset"]["root"] = str(dataset_root)
config["dataset"]["episodes"] = split["train"]
config["output_dir"] = str(run / "output")
config["resume"] = False
config["checkpoint_path"] = None
(run / "split.json").write_text(json.dumps(split, indent=2))
(run / "train.json").write_text(json.dumps(config, indent=2))
print("Training configuration:", run / "train.json")
PYCONFIG
```

Run the wrapper with the configuration path printed above:

```bash
python ~/openarm-smolvla/scripts/train_cube.py --config_path=PATH_TO_TRAIN_JSON
```

The recipe starts from `lerobot/smolvla_base` and fine-tunes for 20,000 updates
with batch size 8. It maps the two camera keys and uses the right arm and gripper.
Reduce batch size if GPU memory is insufficient. Checkpoints are written under
`output/checkpoints/` in the new run directory; the final model is under
`output/checkpoints/last/pretrained_model/`. The held-out split is saved separately;
this training command does not automatically evaluate robot task success.

### 6. Select the trained checkpoint

**Git cloning does not download our trained weights or demonstrations.** No public
artifact download URL is configured here. Obtain the complete checkpoint directory
from the experiment owner, or fine-tune your own model with the matching schema.
The base SmolVLA download is not our cube-task checkpoint.

Place the checkpoint at `~/robot_ai/models/smolvla_cube_20k/`, or pass `--model` to
the inference scripts. It must include `model.safetensors`, `config.json`, both
processor JSON files and their normalization tensors. Keep their contents together.

To reproduce offline evaluation, obtain the 60-episode LeRobot export and preserve
its `data/`, `meta/` and `videos/` contents. The evaluation script currently expects
the experiment-specific export path in [the reference experiment](docs/reference-experiment.md). Raw recordings are needed
only for re-exporting/editing demonstrations, not for inference or training from an
already converted LeRobot dataset. Never add large artifacts to ordinary Git.

### 7. Build the policy plugin

```bash
source /opt/ros/jazzy/setup.bash
cmake -S ~/openarm-smolvla -B ~/openarm-smolvla/build \
  -DVR_APP_ROOT="$HOME/vive_vr_ws/src/vive-vr-ros2"
cmake --build ~/openarm-smolvla/build -j2
ctest --test-dir ~/openarm-smolvla/build --output-on-failure
```

The app's `external_control.hpp` and the plugin must be built against compatible
ROS/C++ dependencies. The remaining run instructions below explain camera startup,
loading the plugin, shadow tests and explicit motion trials.

## Run the trained policy

1. Stop any old policy and robot launch. Use the app panel's CAN setup as usual.
2. Start cameras from the app panel, or run this in a separate terminal and leave
   it running. Do not start two camera servers or record during a policy trial.

```bash
/usr/bin/python3 ~/vive_vr_ws/src/vive-vr-ros2/scripts/openarm_cameras.py --fps 60
```

3. In a separate terminal, grant scheduling priority if needed, then start hardware
   with the policy plugin. Replace the example IP with the robot-connected host's network IP.
   Leave this terminal running. Do not also click the panel's Start VR robot button.

```sh
sudo prlimit --pid $$ --rtprio=50:50
bash ~/openarm-smolvla/scripts/start_robot.sh YOUR_HOST_IP
```

This starts hardware held at its measured pose; it does not start model motion.
Use the app's existing return-to-zero command if the demonstrated start pose is
needed; clear the direct path first. Leave VR disabled. First check live predictions
without commanding motion, using your complete checkpoint directory:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ai/lerobot_env/bin/activate
python ~/openarm-smolvla/scripts/openarm_smolvla_shadow.py \
  --model PATH_TO_PRETRAINED_MODEL --seconds 60
```

After checking inputs and predictions, run a supervised trial. The command below
uses `~/robot_ai/models/smolvla_cube_20k/`; add `--model PATH_TO_PRETRAINED_MODEL`
to use the training output directly:

```sh
source /opt/ros/jazzy/setup.bash
source ~/robot_ai/lerobot_env/bin/activate
python ~/openarm-smolvla/scripts/openarm_smolvla_run.py --execute --seconds 60
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

## Tested dependency versions

| Component | Fine-tuning environment | Inference/testing environment |
|---|---|---|
| Python | 3.12.3 (reported during setup) | 3.12.3 |
| LeRobot package | 0.5.2 | 0.5.2 |
| LeRobot commit | `cec8ee0be6e1006f9ecbd829fea59d051a11c515` | `d656da8ccca5989ff0a2207e81fbfa2c2d5bafb1` |
| PyTorch | 2.11.0+cu126 (reported) | 2.10.0+cu128 (tested) |
| Transformers | 5.5.4 (reported) | 5.3.0 |
| ROS | Not required for offline training | ROS 2 Jazzy |

Both LeRobot checkouts report 0.5.2, but their Git commits and sampler APIs differ.
Package version alone is insufficient to reproduce the training environment.
Training-environment values were captured after training; they describe that run
provided the environment was not subsequently changed. Inference package versions
were checked again on 2026-11-25. These are tested configurations, not a requirement
to use separate computers.

## Documentation website

See [Build and publish the documentation](docs/documentation.md) for local preview
and GitHub Pages setup. The website is generated from this README, the guides and
the C++ headers; no robot or model dependencies are needed to build it.
