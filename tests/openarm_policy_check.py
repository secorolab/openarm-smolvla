#!/usr/bin/env python3
"""Synthetic policy ownership/watchdog tests; isolated domain and no CAN drivers."""
import os
os.environ['ROS_DOMAIN_ID'] = '198'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import subprocess
import sys
import time
import json
import select
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.msg import ControllerState

rclpy.init()
node=rclpy.create_node('policy_check')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
feedback=node.create_publisher(JointState,'/joint_states',qos_profile_sensor_data)
health=[node.create_publisher(Bool,f'/vive_vr/hardware/{s}/healthy',qos) for s in ('right','left')]
plans=node.create_publisher(JointTrajectory,'/openarm_hardware_preview/policy_plan',1)
commands=[]; left=[]; fingers=[]; statuses=[]
subscriptions=[node.create_subscription(JointTrajectory,'/right_joint_trajectory_controller/joint_trajectory',commands.append,100),
               node.create_subscription(JointTrajectory,'/left_joint_trajectory_controller/joint_trajectory',left.append,100),
               node.create_subscription(JointTrajectory,'/right_gripper_controller/joint_trajectory',fingers.append,100),
               node.create_subscription(String,'/openarm_hardware_preview/policy_status',lambda msg:statuses.append(msg.data),qos)]
def controllers(req,res):
    res.controller=[ControllerState(name=f'{s}_{k}',state='active') for s in ('right','left') for k in ('joint_trajectory_controller','gripper_controller')]
    return res
service=node.create_service(ListControllers,'/controller_manager/list_controllers',controllers)
clients={name:node.create_client(SetBool,'/openarm_hardware_preview/'+name) for name in ('policy_enable','enable','return_to_zero')}
proc=subprocess.Popen([sys.argv[1],'--ros-args','-p',f'model:={Path(__file__).with_name("openarm_test_arm.xml")}',
                       '-p',f'external_control_library:={sys.argv[2]}','-p','hardware.enable_commands:=true','-p','hardware.control_grippers:=true'],stdout=subprocess.DEVNULL)
helper=None
names=[f'openarm_{s}_joint{i}' for s in ('right','left') for i in range(1,8)]+[f'openarm_{s}_finger_joint1' for s in ('right','left')]
def pump(seconds, healthy=True, feed=True):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if feed:
            msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg()
            msg.name=names; msg.position=[0.0]*14+[.01,.01]; feedback.publish(msg)
        for pub in health: pub.publish(Bool(data=healthy))
        rclpy.spin_once(node,timeout_sec=.003)
        time.sleep(.005)
        assert proc.poll() is None
def call(name,value=True):
    f=clients[name].call_async(SetBool.Request(data=value))
    end=time.monotonic()+2
    while not f.done() and time.monotonic()<end: pump(.02)
    assert f.done(), name
    return f.result()
def plan(token,goal=.1,age=.05):
    msg=JointTrajectory(); msg.header.frame_id=token
    stamp=node.get_clock().now().nanoseconds-int(age*1e9)
    msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,10**9)
    msg.joint_names=[f'openarm_right_joint{i}' for i in range(1,8)]+['openarm_right_finger_joint1']
    for k in range(50):
        p=JointTrajectoryPoint(); p.positions=[goal]*7+[.02]
        p.time_from_start.sec,p.time_from_start.nanosec=divmod(round(k*1e9/30),10**9)
        msg.points.append(p)
    return msg
try:
    pump(2)
    assert not commands and not fingers
    response=call('policy_enable'); assert response.success,response.message
    token=response.message
    assert not call('policy_enable').success
    assert not call('enable').success
    assert not call('return_to_zero').success
    plans.publish(plan('wrong-token')); pump(.1); assert not commands
    valid=plan(token); plans.publish(valid); pump(.25)
    assert commands and fingers and not left
    for a,b in zip(commands,commands[1:]):
        assert max(abs(x-y) for x,y in zip(a.points[0].positions,b.points[0].positions))<=.00600001
    for a,b in zip(fingers,fingers[1:]):
        assert abs(a.points[0].positions[0]-b.points[0].positions[0])<=.00020001
    # Replayed plans must not extend the lease.
    for _ in range(7): plans.publish(valid); pump(.1)
    assert statuses[-1].startswith('stopped:policy prediction watchdog'),statuses
    pump(.1); count=len(commands); plans.publish(plan(token)); pump(.1); assert len(commands)==count
    for fault in ('stale','future','shape','malformed','outlier','health','feedback','stop'):
        pump(.3)
        response=call('policy_enable'); assert response.success,response.message
        token=response.message
        msg=plan(token)
        if fault=='stale': msg=plan(token,age=1)
        if fault=='future': msg=plan(token,age=-1)
        if fault=='shape': msg.points.pop()
        if fault=='malformed': msg.joint_names.reverse()
        if fault=='outlier': msg.points[0].positions[0]=100.
        plans.publish(msg)
        pump(.08)
        if fault=='health': pump(.2,healthy=False)
        elif fault=='feedback': pump(.25,feed=False)
        elif fault=='stop': assert call('policy_enable',False).success
        else: pump(.08)
        pump(.1)
        assert statuses[-1].startswith('stopped:'),(fault,statuses[-1])
        if fault in ('stale','future'): assert 'input age at bridge=' in statuses[-1],statuses[-1]
        if fault=='shape': assert 'expected 8 joints and 50 points' in statuses[-1],statuses[-1]
        count=len(commands); pump(.1); assert len(commands)==count,fault
    assert not left
    helper=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).parents[1]/'scripts/openarm_smolvla_shadow.py'),'--policy-bridge'],
                            stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    pump(1.5)
    def request(payload, allow_wait=False):
        helper.stdin.write(json.dumps(payload)+'\n'); helper.stdin.flush()
        end=time.monotonic()+4
        while not select.select([helper.stdout],[],[],0)[0] and time.monotonic()<end: pump(.01)
        assert select.select([helper.stdout],[],[],0)[0], 'helper response timeout'
        result=json.loads(helper.stdout.readline())
        assert allow_wait or ('error' not in result and 'fatal' not in result),result
        return result
    end=time.monotonic()+6
    observed={}
    while time.monotonic()<end:
        observed=request({'at':time.monotonic()},allow_wait=True)
        if 'state' in observed: break
        pump(.2)
    assert len(observed['state'])==8,observed
    assert request({'op':'acquire'})['acquired']
    pump(.03)
    before=len(commands)
    pump(.03)
    assert len(commands)==before, 'acquire must not replay warm-up targets'
    submitted=request({'op':'plan','at':time.monotonic(),'actions':[[.05]*7+[.015]]*50})
    assert submitted['submitted']
    assert 0 <= submitted['input_age_at_submit_s'] <= submitted['input_age_after_publish_s']
    pump(.1)
    assert statuses[-1].startswith('active:')
    helper.stdin.close()
    end=time.monotonic()+4
    while helper.poll() is None and time.monotonic()<end: pump(.02)
    assert helper.poll()==0
    pump(.1)
    assert statuses[-1].startswith('stopped:policy stopped by operator')
    print('PASS: policy ownership, bounded output, replay/timeout, invalid plans, faults and explicit stop; no CAN')
finally:
    if helper and helper.poll() is None:
        helper.terminate()
        try: helper.wait(timeout=3)
        except subprocess.TimeoutExpired: helper.kill(); helper.wait()
    proc.terminate()
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
    node.destroy_node(); rclpy.shutdown()
