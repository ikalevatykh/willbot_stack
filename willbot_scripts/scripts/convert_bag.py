#!/usr/bin/env python

import lmdb
import numpy as np
import rosbag
import rospy
from io import BytesIO
from pathlib import Path
import pickle as pkl
from pykdl_utils.kdl_kinematics import KDLKinematics
from scipy import interpolate
from scipy import signal
from urdf_parser_py.urdf import URDF
from tqdm import tqdm


path = Path('/root/data/pick')


base_link = 'base_link'
tool_link = 'tool'
urdf = '/root/src/willbot_stack/willbot_description/urdf/willbot.urdf'
urdf = URDF.from_xml_file(urdf)
kdl_kin = KDLKinematics(urdf, base_link, tool_link)
joint_names = kdl_kin.get_joint_names()

def forward_kin(q):
    m = kdl_kin.forward(q)
    return np.squeeze(np.asarray(m[:3, 3]))


db_map_size = 1099511627776  # 1TB
db_path = path / 'database'
if not db_path.exists():
    db_path.mkdir()

db = lmdb.open(str(db_path), db_map_size)


for seed in tqdm(range(0, 1000)):
    bag_name = path / '{}.bag'.format(seed)

    arm_target = []
    arm_state = []
    arm_wrench = []
    tool_velocity = []
    hand_target = []
    hand_state = []
    rgb = []
    depth = []

    try:
        bag = rosbag.Bag(str(bag_name))
    except:
        continue
    T0 = bag.get_start_time()
    T1 = bag.get_end_time()

    for topic, msg, t in bag.read_messages():
        if topic == 'traj_controller_goal':
            arm_target.append(msg)
        if topic == 'traj_controller_state':
            arm_state.append(msg)
        if topic == 'tool_velocity':
            tool_velocity.append(msg)
        if topic == 'wrench':
            arm_wrench.append(msg)
        if topic == 'hand_output':
            hand_target.append(msg)
        if topic == 'hand_input':
            hand_state.append(msg)
        if topic == 'rgb':
            rgb.append(msg)
        if topic == 'depth':
            depth.append(msg)
    bag.close()

    # arm target
    target_q = []
    target_t = []
    for msg in arm_target[1:]:
        t0 = msg.header.stamp.to_sec() - T0
        t = [t0 + p.time_from_start.to_sec() for p in msg.goal.trajectory.points]
        ind = [msg.goal.trajectory.joint_names.index(n) for n in joint_names]
        pos = [np.array(p.positions)[ind] for p in msg.goal.trajectory.points]
        target_q += pos
        target_t += t

    t = np.arange(0, T1 - T0, 0.1).tolist()
    f = interpolate.interp1d(target_t, target_q,
                             axis=0, kind='linear', fill_value="extrapolate")
    target_q = f(t)
    target_dq = np.vstack([np.diff(target_q, axis=0) * 10., 6 * [0]])

    # joint position
    desired_q = []
    actual_q = []
    actual_dq = []
    arm_state_t = []
    for msg in arm_state:
        t = msg.header.stamp.to_sec() - T0
        arm_state_t.append(t)

        ind = [msg.joint_names.index(n) for n in joint_names]
        pos = np.array(msg.desired.positions)[ind]
        desired_q.append(pos)

        pos = np.array(msg.actual.positions)[ind]
        actual_q.append(pos)

        vel = np.array(msg.actual.velocities)[ind]
        actual_dq.append(vel)

    t = np.arange(0, T1 - T0, 0.1).tolist()
    f = interpolate.interp1d(arm_state_t, desired_q,
                             axis=0, kind='linear', fill_value="extrapolate")
    desired_q = f(t)
    desired_dq = np.vstack([np.diff(desired_q, axis=0) * 10., 6 * [0]])

    # tool position
    actual_dx = []
    for msg in tool_velocity:
        v = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
             msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        actual_dx.append(v)

    actual_x = [forward_kin(q) for q in actual_q]
    desired_x = [forward_kin(q) for q in desired_q]
    desired_dx = np.vstack([np.diff(desired_x, axis=0) * 10., 3 * [0]])
    target_x = [forward_kin(q) for q in target_q]
    target_dx = np.vstack([np.diff(target_x, axis=0) * 10., 3 * [0]])

    # arm wrench
    wrench = []
    for msg in arm_wrench:
        w = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
             msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        wrench.append(w)

    # hand target
    target_grip_pos = []
    target_grip_vel = []
    for msg in hand_target:
        pos = [msg.rPRA, msg.rPRA, msg.rPRA]
        target_grip_pos.append(pos)
        target_grip_vel.append(2.0 - 4.0 * (msg.rPRA > 0))

    # hand state
    actual_grip_pos = []
    for msg in hand_state:
        pos = [msg.gPOA, msg.gPOB, msg.gPOC]
        actual_grip_pos.append(pos)

    with db.begin(write=True) as txn:
        for i in range(len(depth)):
            state = dict(
                joint_position=actual_q[i],
                joint_velocity=actual_dq[i],
                tool_position=actual_x[i],
                #linear_velocity=actual_dx[i],
                wrench=wrench[i],
                grip_position=actual_grip_pos[i]
            )

            action = dict(
                #joint_position=desired_q[i],
                #joint_velocity=desired_dq[i],
                #tool_position=desired_x[i],
                linear_velocity=desired_dx[i],
                grip_velocity=target_grip_vel[i]
            )

            img_rgb = rgb[i].data
            img_depth = depth[i].data

            dic_entry = dict(
                state=state,
                action=action,
                rgb=img_rgb,
                depth=img_depth
            )

            dic_buf = BytesIO()
            pkl.dump(dic_entry, dic_buf)
            ind = '{:06}/{:06}'.format(seed, i)
            txn.put(ind.encode('ascii'), dic_buf.getvalue())