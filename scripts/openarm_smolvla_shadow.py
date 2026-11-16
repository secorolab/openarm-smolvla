#!/usr/bin/env python3
"""Read live images and right-arm feedback; log predictions without commanding motors."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import io
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time
from urllib.request import urlopen

JOINTS = [f'openarm_right_joint{i}' for i in range(1, 8)] + ['openarm_right_finger_joint1']


def validate_times(stamps, now):
    if any(not math.isfinite(t) or not 0 <= now-t <= .15 for t in stamps):
        raise ValueError('Camera or joint feedback is older than 150 ms')
    if max(stamps)-min(stamps) > .08:
        raise ValueError('Camera/joint timing differs by more than 80 ms')


def feedback_bridge(control=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('openarm_smolvla_shadow_observer')
    lock = threading.Lock()
    states = deque(maxlen=100)
    health = [0.0, False]
    token = None
    claimed = False
    policy_status = ['']
    if control:
        from std_srvs.srv import SetBool
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        client = node.create_client(SetBool, '/openarm_hardware_preview/policy_enable')
        publisher = node.create_publisher(JointTrajectory, '/openarm_hardware_preview/policy_plan', 1)
        node.create_subscription(String, '/openarm_hardware_preview/policy_status',
                                 lambda msg: policy_status.__setitem__(0, msg.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def enable(value):
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Policy service unavailable: start hardware with openarm-smolvla/scripts/start_robot.sh (policy plugin required)')
        future = client.call_async(SetBool.Request(data=value))
        end = time.monotonic()+3
        while not future.done() and time.monotonic()<end:
            time.sleep(.01)
        if not future.done():
            raise RuntimeError('Policy service timed out')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response.message

    def state(msg):
        age = (node.get_clock().now().nanoseconds - msg.header.stamp.sec*10**9 - msg.header.stamp.nanosec)/1e9
        if not -.05 <= age <= .15 or len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        mapping = dict(zip(msg.name, msg.position))
        if not all(name in mapping and math.isfinite(mapping[name]) for name in JOINTS):
            return
        with lock:
            states.append((time.monotonic()-max(age, 0), [mapping[name] for name in JOINTS]))

    def healthy(msg):
        with lock:
            health[:] = [time.monotonic(), bool(msg.data)]

    node.create_subscription(JointState, '/joint_states', state, qos_profile_sensor_data)
    node.create_subscription(Bool, '/vive_vr/hardware/right/healthy', healthy,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        for line in sys.stdin:
            request = json.loads(line)
            op = request.get('op', 'state')
            if control and op == 'acquire':
                try:
                    if claimed:
                        raise RuntimeError('Policy cannot automatically restart')
                    claimed = True
                    token = enable(True)
                    end = time.monotonic()+.3
                    while policy_status[0] != 'active:'+token and time.monotonic()<end:
                        time.sleep(.005)
                    if policy_status[0] != 'active:'+token:
                        raise RuntimeError('Policy ownership acknowledgement missing')
                    print(json.dumps({'acquired': True}), flush=True)
                except Exception as exc:
                    print(json.dumps({'error': str(exc)}), flush=True)
                    break
                continue
            if control and op == 'plan':
                try:
                    if token is None:
                        raise RuntimeError('Acquire policy before capturing command inputs')
                    elif policy_status[0] != 'active:'+token:
                        raise RuntimeError('Policy ownership lost: '+policy_status[0])
                    age = time.monotonic()-float(request['at'])
                    if not 0 <= age <= .65:
                        raise RuntimeError('Prediction too old to send')
                    msg = JointTrajectory()
                    msg.header.frame_id = token
                    stamp = node.get_clock().now().nanoseconds-int(age*1e9)
                    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                    msg.joint_names = JOINTS
                    for i, pose in enumerate(request['actions']):
                        point = JointTrajectoryPoint()
                        point.positions = pose
                        point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(i*1e9/30), 10**9)
                        msg.points.append(point)
                    publisher.publish(msg)
                    print(json.dumps({'submitted': True, 'input_age_at_submit_s': age,
                                      'input_age_after_publish_s': time.monotonic()-float(request['at'])}), flush=True)
                except Exception as exc:
                    print(json.dumps({'error': str(exc)}), flush=True)
                    break
                continue
            if control and token and policy_status[0] != 'active:'+token:
                print(json.dumps({'fatal': 'Policy stopped: '+policy_status[0]}), flush=True)
                break
            at = float(request['at'])
            with lock:
                if not health[1] or time.monotonic()-health[0] > .5:
                    result = {'error': 'Right driver health missing or faulted; no prediction'}
                elif not states:
                    result = {'error': 'Waiting for measured right-arm joint states'}
                else:
                    stamp, values = min(states, key=lambda item: abs(item[0]-at))
                    result = {'stamp': stamp, 'state': values}
            print(json.dumps(result), flush=True)
    finally:
        if control and claimed:
            try:
                # Only release our own session; never stop a competing owner.
                if token and policy_status[0] == 'active:'+token:
                    enable(False)
            except Exception as exc:
                print(f'Policy stop not confirmed; bridge watchdog remains active: {exc}', file=sys.stderr)
        rclpy.try_shutdown()
        thread.join(timeout=3)
        node.destroy_node()


def fetch_camera(port, name):
    with urlopen(f'http://127.0.0.1:{port}/cameras/{name}.jpg', timeout=2) as response:
        stamp = response.headers.get('X-Frame-Monotonic')
        if stamp is None:
            raise ValueError('Restart the camera server to use the updated timestamp headers')
        return float(stamp), response.read()


def run(args):
    import numpy as np
    import torch
    from PIL import Image
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

    config = PreTrainedConfig.from_pretrained(args.model)
    config.device, config.load_vlm_weights = 'cuda', False
    if tuple(config.input_features['observation.state'].shape) != (8,) or tuple(config.output_features['action'].shape) != (8,):
        raise ValueError('Expected seven right-arm joints and one gripper')
    if config.chunk_size != 50 or config.n_obs_steps != 1:
        raise ValueError('This runner requires 50-action chunks and one observation step')
    torch.manual_seed(42)
    policy = SmolVLAPolicy.from_pretrained(args.model, config=config, strict=True)
    pre, post = make_pre_post_processors(policy.config, pretrained_path=args.model)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    execute = getattr(args, 'execute', False)
    bridge_mode = '--policy-bridge' if execute else '--feedback-bridge'
    bridge = subprocess.Popen(['/usr/bin/python3', __file__, bridge_mode],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1, env=env,
                              start_new_session=True)
    path = Path(args.output).expanduser()
    label = 'EXECUTION: guarded RIGHT arm and gripper; Ctrl+C stops policy' if execute else 'SHADOW ONLY: no command publishers or enable services'
    print(f'{label}. Log: {path}', flush=True)
    def exchange(payload):
        bridge.stdin.write(json.dumps(payload)+'\n')
        bridge.stdin.flush()
        if not select.select([bridge.stdout], [], [], 3)[0]:
            raise RuntimeError('ROS observer is not responding; check sourced ROS environment')
        line = bridge.stdout.readline()
        if not line:
            raise RuntimeError('ROS observer exited; inspect the error above')
        response = json.loads(line)
        if 'fatal' in response:
            raise RuntimeError(response['fatal'])
        return response
    try:
        with path.open('x') as log, ThreadPoolExecutor(max_workers=2) as pool, torch.inference_mode():
            deadline = time.monotonic()+args.seconds
            count = 0
            acquired = False
            while time.monotonic() < deadline:
                try:
                    frames = dict(zip(('main', 'right'), pool.map(lambda name: fetch_camera(args.port, name), ('main', 'right'))))
                    at = sum(frame[0] for frame in frames.values())/2
                    feedback = exchange({'at': at})
                    if 'error' in feedback:
                        raise ValueError(feedback['error'])
                    stamps = [frame[0] for frame in frames.values()] + [feedback['stamp']]
                    validate_times(stamps, time.monotonic())
                    observation = {'observation.state': torch.tensor(feedback['state'], dtype=torch.float32), 'task': args.task}
                    for name, (_, jpeg) in frames.items():
                        pixels = np.array(Image.open(io.BytesIO(jpeg)).convert('RGB'), copy=True)
                        if pixels.shape != (480, 640, 3):
                            raise ValueError(f'{name}: expected 640x480 image')
                        observation[f'observation.images.{name}'] = torch.from_numpy(pixels).permute(2, 0, 1).float()/255
                    batch = pre(observation)
                    validate_times(stamps, time.monotonic())
                    policy.reset()
                    torch.cuda.synchronize()
                    started = time.monotonic()
                    actions = post(policy.predict_action_chunk(batch))[0].cpu()
                    torch.cuda.synchronize()
                    if actions.shape != (config.chunk_size, 8) or not torch.isfinite(actions).all():
                        raise RuntimeError('Invalid prediction output')
                    elapsed = time.monotonic()-started
                    delta = (actions[0, :7]-observation['observation.state'][:7]).abs().max().item()*180/math.pi
                    row = {'event': 'prediction', 'at': time.monotonic(), 'source_monotonic': dict(main=stamps[0], right=stamps[1], state=stamps[2]),
                           'state': feedback['state'], 'actions': actions.tolist(), 'prediction_seconds': elapsed,
                           'oldest_input_age_at_completion_s': time.monotonic()-min(stamps), 'task': args.task}
                    if execute:
                        if not acquired:
                            log.write(json.dumps(dict(row, event='warmup'))+'\n'); log.flush()
                            if time.monotonic() >= deadline:
                                break
                            response = exchange({'op': 'acquire'})
                            if 'error' in response:
                                raise RuntimeError(response['error'])
                            acquired = True
                            print('Warm-up complete. Control acquired; capturing fresh inputs before movement.', flush=True)
                            continue
                        if row['oldest_input_age_at_completion_s'] > .65:
                            raise RuntimeError('Prediction latency exceeded 650 ms; stopping')
                        response = exchange({'op': 'plan', 'at': min(stamps), 'actions': actions.tolist()})
                        if 'error' in response:
                            raise RuntimeError(response['error'])
                        row['submitted_to_guard'] = True
                        row['input_age_at_submit_s'] = response['input_age_at_submit_s']
                        row['input_age_after_publish_s'] = response['input_age_after_publish_s']
                    count += 1
                    outcome = 'Submitted to guarded bridge.' if execute else 'NOT SENT.'
                    print(f'Prediction {count}: {elapsed:.3f}s; first target differs by up to {delta:.2f} deg. {outcome}', flush=True)
                except RuntimeError as exc:
                    log.write(json.dumps({'event': 'stopped', 'at': time.monotonic(), 'reason': str(exc)})+'\n')
                    log.flush()
                    raise
                except (ValueError, OSError) as exc:
                    if execute and acquired:
                        raise RuntimeError(f'Live input failed; stopping policy: {exc}') from exc
                    row = {'event': 'waiting', 'at': time.monotonic(), 'reason': str(exc)}
                    print(f'Waiting: {exc}', flush=True)
                    time.sleep(1)
                log.write(json.dumps(row)+'\n')
                log.flush()
            print(f'Finished: {count} predictions logged. '+('Stopping policy.' if execute else 'No robot commands sent.'), flush=True)
    finally:
        bridge.stdin.close()
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.terminate()
            bridge.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--feedback-bridge', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--policy-bridge', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--model', default=str(Path.home()/'robot_ai/models/smolvla_cube_20k'))
    parser.add_argument('--task', default='Put the cube in the basket')
    parser.add_argument('--domain-id', type=int, default=84)
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', default=f'/tmp/openarm_shadow_{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl')
    args = parser.parse_args()
    if args.feedback_bridge or args.policy_bridge:
        feedback_bridge(control=args.policy_bridge)
    else:
        if not math.isfinite(args.seconds) or args.seconds <= 0:
            parser.error('--seconds must be positive and finite')
        run(args)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Shadow test stopped. No robot commands sent.')
