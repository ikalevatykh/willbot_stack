#!/usr/bin/env python

from __future__ import print_function
import roslib; roslib.load_manifest('recorder')

import rosbag
import rospy
from threading import Thread, Event

from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import CompressedImage, Image
from robotiq_s_model_control.msg import _SModel_robot_output as SModelOutputMsg
from robotiq_s_model_control.msg import _SModel_robot_input as SModelInputMsg

import PIL.Image
import StringIO
import numpy as np

from recorder.srv import *

#/pos_based_pos_traj_controller/state #  control_msgs/JointTrajectoryControllerState
#/tool_velocity #  geometry_msgs/TwistStamped
#/wrench  # geometry_msgs/WrenchStamped

#/SModelRobotInput # robotiq_s_model_control/SModel_robot_input
#/SModelRobotOutput # robotiq_s_model_control/SModel_robot_output

#/camera/rgb/image_color/compressed #  sensor_msgs/CompressedImage
#/camera/depth_registered/image/compressedDepth  # sensor_msgs/CompressedImage
#/camera/depth_registered/image_raw/compressed #  sensor_msgs/CompressedImage

class SignalCollector(object):
	def __init__(self):	
		self._traj_controller_goal = None
		self._traj_controller_state = None
		self._tool_velocity = None
		self._wrench = None
		
		self._hand_input = None
		self._hand_output = SModelOutputMsg.SModel_robot_output()
		
		self._rgb = None
		self._depth = None
		self._depth_raw = None
		
		rospy.Subscriber("/pos_based_pos_traj_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, 
			self._traj_controller_goal_cb, queue_size=1)		
		rospy.Subscriber("/pos_based_pos_traj_controller/state", JointTrajectoryControllerState, 
			self._traj_controller_state_cb, queue_size=1)
		# rospy.Subscriber("/tool_velocity", TwistStamped, 
		# 	self._tool_velocity_cb, queue_size=1)
		rospy.Subscriber("/wrench", WrenchStamped, 
			self._wrench_cb, queue_size=1)
		
		rospy.Subscriber("/SModelRobotInput", SModelInputMsg.SModel_robot_input, 
			self._hand_input_cb, queue_size=1)
		rospy.Subscriber("/SModelRobotOutput", SModelOutputMsg.SModel_robot_output, 
			self._hand_output_cb, queue_size=1)
			
		# rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, 
		# 	self._rgb_cb, queue_size=1)
		# rospy.Subscriber("/camera/depth_registered/image/compressedDepth", CompressedImage, 
		# 	self._depth_cb, queue_size=1)
		# rospy.Subscriber("/camera/depth_registered/image_raw/compressed", CompressedImage, 
		# 	self._depth_raw_cb, queue_size=1)

		

		rospy.Subscriber("/camera/color/image_rect_color", Image, 
			self._rgb_cb, queue_size=1)
		rospy.Subscriber("/camera/depth/image_rect_raw", Image, 
			self._depth_cb, queue_size=1)
			
		print('Waiting for signals...')
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			signals = [
					self._traj_controller_state,
					#self._tool_velocity,
					self._wrench,
		
					self._hand_input,
					self._hand_output,
		
					self._rgb,
					self._depth,
					#self._depth_raw
			]
			if all(signals):
				break
			print('signals state: ', [s is not None for s in signals])
			r.sleep()			
		print('All signals ready')		
						
	def _traj_controller_goal_cb(self, msg):			
		self._traj_controller_goal = msg						
	def _traj_controller_state_cb(self, msg):			
		self._traj_controller_state = msg		
	def _tool_velocity_cb(self, msg):			
		self._tool_velocity = msg
	def _wrench_cb(self, msg):			
		self._wrench = msg

	def _hand_input_cb(self, msg):			
		self._hand_input = msg
	def _hand_output_cb(self, msg):			
		self._hand_output = msg

	def _rgb_cb(self, msg):	
		# Resize and compress image
		src = PIL.Image.frombytes("RGB", (msg.width, msg.height), msg.data, "raw", "BGR")
		
		src.resize((224, 224), PIL.Image.ANTIALIAS)

		dest = StringIO.StringIO()
		src.save(dest, format="JPEG")
		
		dest_msg = CompressedImage()
		dest_msg.header.seq = msg.header.seq		
		dest_msg.header.stamp.secs = msg.header.stamp.secs
		dest_msg.header.stamp.nsecs = msg.header.stamp.nsecs
		dest_msg.header.frame_id = msg.header.frame_id
		dest_msg.format = "JPEG"
		dest_msg.data = dest.getvalue()
		### End of compression and resizing		

		self._rgb = dest_msg

	def _depth_cb(self, msg):			
		# Resize and compress image
		src = PIL.Image.frombytes("F", (msg.width, msg.height), msg.data, "raw", "F;16").resize((224, 224), PIL.Image.ANTIALIAS)

		arr = np.array(src).reshape(src.size[0], src.size[1])
		arr = np.clip(arr, 0, 2000)
		arr -= np.min(arr)
		arr *= 255 / np.max(arr)

		src = PIL.Image.fromarray(arr.astype("uint8"))

		dest = StringIO.StringIO()
		src.save(dest, format="JPEG")
		
		dest_msg = CompressedImage()
		dest_msg.header.seq = msg.header.seq		
		dest_msg.header.stamp.secs = msg.header.stamp.secs
		dest_msg.header.stamp.nsecs = msg.header.stamp.nsecs
		dest_msg.header.frame_id = msg.header.frame_id
		dest_msg.format = "JPEG"
		dest_msg.data = dest.getvalue()
		### End of compression and resizing

		self._depth = dest_msg

	def _depth_raw_cb(self, msg):			
		self._depth_raw = msg

class Bag(object):
	def __init__(self, signals, file_name, frequency = 10):
		self._signals = signals
		self._file_name = file_name
		self._frequency = frequency		
		
		self._bag = rosbag.Bag(self._file_name, 'w')
		self._run = True
		self._event = Event()
		self._thread = Thread(target = self._loop)
		self._thread.start()
		print('Bag initializing... ', self._file_name)
		while not self._event.is_set() and not rospy.is_shutdown():
			self._event.wait(0.1)
		print('Bag initialized... ', self._file_name)
			
	def _loop(self):
		r = rospy.Rate(self._frequency)
		while self._run and not rospy.is_shutdown():			
			if self._signals._traj_controller_goal is not None:
				self._bag.write('traj_controller_goal', self._signals._traj_controller_goal)
				self._signals._traj_controller_goal = None
			
			self._bag.write('traj_controller_state', self._signals._traj_controller_state)
			#self._bag.write('tool_velocity', self._signals._tool_velocity)
			self._bag.write('wrench', self._signals._wrench)
		
			self._bag.write('hand_input', self._signals._hand_input)
			self._bag.write('hand_output', self._signals._hand_output)
		
			self._bag.write('rgb', self._signals._rgb)
			self._bag.write('depth', self._signals._depth)
			#self._bag.write('depth_raw', self._signals._depth_raw)
			
			self._event.set()
			r.sleep()
		
	def close(self):
		if self._thread is not None:
			self._run = False
			self._thread.join()
			self._thread = None
		if self._bag is not None:
			self._bag.close()
			self._bag = None
		print('Bag finished... ', self._file_name)
			
			
class RecordServer():
	def __init__(self):
		rospy.init_node('recorder')
		self._bag = None
		self._signals = SignalCollector()		
		s = rospy.Service('record_command', Command, self._handle_command)
		rospy.spin()

	def _handle_command(self, req):
		try:
			if self._bag:
				self._bag.close()
				self._bag = None
			if req.enable_record:
				self._bag = Bag(self._signals, req.bag_file_name, 10)		
			return CommandResponse(success=True)
		except Exception as e:
			return CommandResponse(success=False, error_string=str(e))

	
if __name__ == '__main__':
    try:
        server = RecordServer()
    except rospy.ROSInterruptException:
        pass




