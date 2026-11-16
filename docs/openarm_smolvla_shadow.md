# SmolVLA live shadow test

`scripts/openarm_smolvla_shadow.py` logs right-arm predictions without publishing
commands, enabling controllers, or calling motion services. It uses the saved
checkpoint's camera mapping and normalization. There is no execution mode.

Start CAN and the normal real hardware launch using the panel. Hardware startup
can enable motor torque; keep the robot supported and its workspace clear. Leave
VR control disarmed. Do not start recording for this test.

Start the camera server in a separate terminal. If it is already running, stop
and restart it to load the image timestamp headers. Only one process should own
the cameras.

```sh
/usr/bin/python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_cameras.py --fps 60
```

In another terminal, source ROS before activating the model environment:

```sh
source /opt/ros/jazzy/setup.zsh
source ~/robot_ai/lerobot_env/bin/activate
python ~/Desktop/openarm-smolvla/scripts/openarm_smolvla_shadow.py --seconds 60
```

The observer subprocess uses system Python and domain 84 with localhost discovery.
It subscribes only to `/joint_states` and `/vive_vr/hardware/right/healthy`.
The model process reads JPEG images from the existing camera HTTP server on
localhost port 8081. Camera timestamps use the same host's monotonic clock.

Inputs must be at most 150 ms old, with at most 80 ms between the selected images
and state. Missing or faulted right-driver health blocks predictions. These are
input checks, not guarantees about predicted motion. Inference adds latency;
each prediction records the oldest input's age at completion.

Logs are created exclusively under unique `/tmp/openarm_shadow_*.jsonl` names.
Each prediction includes the measured eight-value state, full action sequence,
input timestamps, task and prediction duration. Units are radians for J1–J7 and
meters for the gripper. Waiting entries explain missing or stale inputs. No
episodes or datasets are written. Images are read but are not saved in this log.

Keep the original main/right-wrist camera placement and task scene for comparison
with training. Inspect both previews at `http://localhost:8081` before testing.
Ctrl+C stops this observer only; it does not stop the hardware launch or release
motor torque. This test does not establish task success or authorize executing
its predictions.
