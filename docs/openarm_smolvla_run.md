# Bounded SmolVLA policy trials

`scripts/openarm_smolvla_run.py --execute` controls the right arm and gripper through
the real hardware bridge. It does not publish directly to trajectory controllers.
The existing left-arm targets remain held. Both guarded drivers and the existing
controllers must report healthy, and measured feedback must be fresh.

## First trial

Build the app and policy plugin as described in README.md, then use
`bash scripts/start_robot.sh LAPTOP_WIFI_IP` instead of the panel robot-launch button. Leave VR disarmed and stop recording. Return to
the demonstrated starting pose before starting the policy. A right-arm position
outside the model limits is rejected; policy mode does not perform calibration
or startup recovery. Keep the cameras in their demonstrated positions.

Keep the workspace clear and the physical emergency stop accessible. This runner
does not check collisions with the table, objects, people or the other arm.

Run the camera preview without recording, in its own terminal:

```sh
/usr/bin/python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_cameras.py --fps 60
```

In another terminal, this command requests a maximum ten-second trial, including
initial input wait/warm-up. Model loading happens before that time limit begins.
For a supervised three-minute trial, use `--seconds 180`; six minutes uses
`--seconds 360` (the maximum). The default remains
ten seconds; motion bounds and watchdogs are unchanged.
There is no headset requirement. Do not click Enable VR arms/grippers.

```sh
source /opt/ros/jazzy/setup.zsh
source ~/robot_ai/lerobot_env/bin/activate
python ~/Desktop/openarm-smolvla/scripts/openarm_smolvla_run.py --execute --seconds 10
```

Ctrl+C stops the runner and releases its policy session; it does not release motor
torque. Normal completion and faults leave VR disarmed. A fresh explicit command
is required to restart. To stop through ROS from another terminal:

```sh
source /opt/ros/jazzy/setup.zsh
ROS_DOMAIN_ID=84 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
ros2 service call /openarm_hardware_preview/policy_enable std_srvs/srv/SetBool '{data: false}'
```

## Execution and limits

- Explicit policy ownership blocks VR enable, homing and another policy session.
  Session tokens prevent a delayed plan from a previous runner from commanding.
  This arbitration covers the project bridge; do not run external controller/CAN
  command tools alongside it.
- The model's saved pre/postprocessors supply camera renaming and normalization.
  Each plan has 50 absolute right-arm/gripper targets at 30 Hz.
- The first prediction is discarded as warm-up. Control is then acquired before
  capturing fresh inputs, so service discovery/enable delay cannot age that first
  command. No warm-up targets are sent to the bridge.
- Inference runs separately from the bridge's command timer. Plans are timestamped
  at observation time; the bridge skips the elapsed prefix and interpolates the
  remaining trajectory while new predictions are computed. This assumes continued
  motion along the predicted plan and still needs physical evaluation.
- The bridge bounds commanded joint steps to 0.30 rad/s and gripper steps to
  0.010 m/s, using acceleration ramps of 0.6 rad/s² and 0.05 m/s². These bounds
  apply to generated targets, not guarantees of physical motor behavior.
- Model joint/gripper ranges are enforced. Out-of-range predictions within
  0.05 rad (2.86 degrees) are clipped. Gripper predictions up to 3 mm below its
  minimum or 5 mm above its maximum are clipped; larger excursions stop the policy.
  The commanded gripper range remains 0–44 mm, including when a prediction is clipped.
- Joint command lead is bounded to 0.15 rad. Closing gripper commands stop
  advancing at 3 mm of lead, allowing the arm to continue with the held target.
  A closing discrepancy up to 6 mm tolerates rebound; it never adds more squeeze.
  Opening commands can reduce this gap. Opening-direction errors above 4 mm or
  closing errors above 6 mm still stop execution. This uses position feedback,
  not a force sensor: it cannot distinguish object contact from a small jam.
- Camera/state inputs must be fresh (150 ms) and aligned within 80 ms before
  inference. Predictions older than 650 ms are rejected. No fresh accepted plan
  for 750 ms stops policy control; a replay does not refresh that deadline.
- Driver/feedback/controller faults stop policy control through the existing
  bridge. Stops hold the last commanded target and cease the policy stream;
  the existing trajectory-controller timeout and hardware fault behavior remain
  in effect. Holding torque is not guaranteed after a hardware fault.

Logs are new `/tmp/openarm_policy_*.jsonl` files, never recordings or exports.
`submitted_to_guard` means the model submitted a plan, not proof of acceptance or
physical execution. The bridge publishes stop reasons on
`/openarm_hardware_preview/policy_status` and in its launch log.
Timing logs include input age after inference, at submission and after publishing.
Timestamp rejections report age at the receiving bridge; malformed plan shapes
have a separate error. Runner errors are also saved as `stopped` log events.

## Verification

`policy_guard_check` exercises bounds, small-error clipping, invalid predictions,
rate limits, direction reversal and following errors. `openarm_policy_check.py`
uses synthetic feedback and controllers on domain 198, with no CAN drivers, to
test ownership, VR/homing exclusion, timeout/replay, invalid plans, health/feedback
faults and Python handoff/cleanup. Physical pickup success has not been verified.
