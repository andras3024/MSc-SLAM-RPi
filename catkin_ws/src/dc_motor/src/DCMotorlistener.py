#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String,Bool
import time
import geometry_msgs.msg
import os
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from MotorController import MotorController
from MotorController import VxVyWtoMotorSpeeds
from math import sin, cos, pi,degrees,radians
from nav_msgs.msg import Odometry
import numpy as np
from std_srvs.srv import Empty
import datetime

enc11old = 0
enc21old = 0
enc12old = 0
enc22old = 0

v1 = 0
v2 = 0
v3 = 0
v4 = 0

v1old = 0
v2old = 0
v3old = 0
v4old = 0

ENCtoMET = 0.000221657
METtoENC = 4511.4761
D = 2*0.17324
base_th = radians(225) # Robot coordinate system correction

x = 0.0
y = 0.0
th = 0.0
vel_last_time = 0

rospy.init_node('DCMotor', anonymous=True)

odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()

pos_cont_pub = rospy.Publisher("robot/positon_ready", Bool, queue_size=1)

# Covariance
Pcv = np.mat(np.diag([0.0] * 3))
Pcv[0,0] = 0.001
Pcv[1,1] = 0.001
Pcv[2,2] = 0.0003046

address_1 = 0x01
address_2 = 0x03
Motor12 = MotorController(address_1)
Motor34 = MotorController(address_2)

last_time = 0

reset_encoders = False

enc11 = 0
enc12 = 0
enc21 = 0
enc22 = 0

xdes = 0
ydes = 0
newPosMsg = False
newPosMoving = False

publish_tf = rospy.get_param('~publish_tf', False)

v_filter_length = 2
vx_array = np.zeros(v_filter_length)
vy_array = np.zeros(v_filter_length)
vth_array = np.zeros(v_filter_length)
v_index = 0

new_speed = False


def readAllEncoders():
	global enc11old, enc12old, enc21old, enc22old
	global enc11, enc12, enc21, enc22
	#new_time_encoder = time.time()
	enc11 = Motor12.readEncoderCount(1)
	enc21 = Motor34.readEncoderCount(1)
	enc12 = Motor12.readEncoderCount(2)
	enc22 = Motor34.readEncoderCount(2)
	#elapsed_time_encoder = time.time() - new_time_encoder
	#rospy.loginfo("Encoder read time: %s ms", str(elapsed_time_encoder * 1000))
	if abs(enc11 - enc11old) < 10000 and abs(enc21 - enc21old) < 10000 and abs(enc12 - enc12old) < 10000  and abs(enc22 - enc22old) < 10000 :
		denc11i = (enc11 - enc11old)*ENCtoMET
		denc21i = (enc21 - enc21old)*ENCtoMET
		denc12i = (enc12 - enc12old)*ENCtoMET
		denc22i = (enc22 - enc22old)*ENCtoMET
		enc11old = enc11
		enc21old = enc21
		enc12old = enc12
		enc22old = enc22
	else:
		denc11i = 0
		denc21i = 0
		denc12i = 0
		denc22i = 0
		rospy.logwarn("Encoder read error")
	return [denc11i,denc12i,denc21i,denc22i]


def position_control():
	global enc11, enc12, enc21, enc22, METtoENC
	global xdes,ydes
	th_pos_base = base_th + th

	sth = sin(th_pos_base)
	cth = cos(th_pos_base)

	yo = (ydes-(xdes/cth)*sth)/(((sth*sth)/cth)+cth)
	xo = xdes/cth + yo * sth/cth

	cmd_enc11 = int(yo * METtoENC + enc11)
	cmd_enc12 = int(-xo * METtoENC + enc12)
	cmd_enc21 = int(-yo * METtoENC + enc21)
	cmd_enc22 = int(xo * METtoENC + enc22)
	return cmd_enc11,cmd_enc12,cmd_enc21,cmd_enc22


def posupdate():
	global th,x,y,vel_last_time
	global odom_pub, odom_broadcaster
	global Pcv
	global vx_array, vy_array, vth_array, v_index

	[denc11,denc12,denc21,denc22] = readAllEncoders()
	current_time_o = rospy.Time.now()
	# Time difference calculation
	vel_current_time = time.time()
	dt = (vel_current_time - vel_last_time) # [s]
	vel_last_time = vel_current_time
	# Compute odometry of the robot
	d_th_12_22 =  (denc12+denc22)/D # [meter]
	d_th_11_21 =  (denc11+denc21)/D # [meter]
	delta_th = (d_th_12_22+d_th_11_21)/2 # [radian]
	delta_th_div = delta_th/2 # [radian]
	# Theta calculation (rotation around Z-axis)
	th_div = th + delta_th_div
	th += delta_th

	delta_x0 = (denc22-denc12)/2
	delta_y0 = (denc11-denc21)/2
	delta_x = delta_x0*cos(base_th+th_div)-delta_y0*sin(base_th+th_div)
	delta_y = delta_x0*sin(base_th+th_div)+delta_y0*cos(base_th+th_div)
	x += delta_x
	y += delta_y


	delta_x_base = delta_x0*cos(base_th)-delta_y0*sin(base_th)
	delta_y_base = delta_x0*sin(base_th)+delta_y0*cos(base_th)
	vx_array[v_index] = delta_x_base/dt # m/s
	vy_array[v_index] = delta_y_base/dt # m/s
	vth_array[v_index] = delta_th/dt # rad/s
	if v_index < v_filter_length-1:
		v_index += 1
	else:
		v_index = 0

	vx = np.average(vx_array)
	vy = np.average(vy_array)
	vth = np.average(vth_array)

	if th > pi:
		th = th-2*pi
	elif th < -pi:
		th = th+2*pi
	# Odometry is 6DOF we'll need a quaternion created from yaw
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

	# Publish the transform over tf
	if publish_tf:
		odom_broadcaster.sendTransform((x, y, 0.),odom_quat,current_time_o,"base_link","odom")

	# Publish the odometry message over ROS
	odom = Odometry()
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = "odom"
	odom.child_frame_id = "base_link"

	# Odometry pose covariance
	p_cov = np.array([0.0] * 36).reshape(6, 6)
	# position covariance for x and y
	p_cov[0:2, 0:2] = Pcv[0:2, 0:2]
	# orientation covariance for Yaw
	p_cov[5, 5] = Pcv[2, 2]
	odom.pose.covariance = tuple(p_cov.ravel().tolist())

	# Set the position
	odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

	# Set the velocity
	odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
	# Velocity covariance
	if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(vth) < 2e-3:
		odom.twist.covariance[0] = 1e-12 # vx/vx
		odom.twist.covariance[7] = 1e-12  # vy/vy
		odom.twist.covariance[35] = 1e-12  # vth/vth
	else:
		odom.twist.covariance[0] = 0.01 # vx/vx
		odom.twist.covariance[7] = 0.01  # vy/vy
		odom.twist.covariance[35] = 0.02  # vth/vth

	# Publish the message
	odom_pub.publish(odom)

def handle_reset_encoders(self):
	global reset_encoders
	reset_encoders = True
	return []

def callback_vel(data):
	global v1, v2, v3, v4, new_speed
	# Get new motor speeds
	[v1,v2,v3,v4] = VxVyWtoMotorSpeeds(data.linear.x,data.linear.y,data.angular.z,225)
	new_speed = True

def callback_pos(data):
	global xdes, ydes, newPosMsg
	xdes = data.position.x
	ydes = data.position.y
	newPosMsg = True

def listener():
	# Global variables for use in function
	global v1old, v2old, v3old, v4old
	global v1, v2, v3, v4, new_speed
	global last_time
	global reset_encoders
	global th,x,y
	global enc11old, enc12old, enc21old, enc22old
	global newPosMsg, newPosMoving
	# Init motor controllers
	Motor12.openI2Cbus() # Only to be called on one I2C motor controller
	Motor12.controllerEnable()
	Motor34.controllerEnable()
	DCfirm = Motor12.readDCFirmware()
	# Read Battery Voltage
	Bvoltage = Motor12.readBatteryVoltage()
	rospy.loginfo("DC Motor controller initialized")
	rospy.loginfo("DC Motor firmware version: %s",str(DCfirm))
	rospy.loginfo("Battery Voltage: %s V",str(Bvoltage))

	# Reset encoders to zero
	Motor12.resetEncoders()
	Motor34.resetEncoders()

	# Start ROS node
	rospy.Subscriber("/cmd_vel", Twist, callback_vel)
	rospy.Subscriber("/cmd_pos", Pose, callback_pos)
	# Reset encoders service
	rospy.Service('/reset_encoders', Empty, handle_reset_encoders)

	# 40 Hz while loop rate
	r = rospy.Rate(40)
	while not rospy.core.is_shutdown():
		start = datetime.datetime.now()
		if reset_encoders:
			# Reset encoders to zero
			Motor12.resetEncoders()
			Motor34.resetEncoders()
			# Reset odometry pose data to zero
			th = 0
			x = 0
			y = 0
			# Reset encoder variables to zero
			enc11old = 0
			enc21old = 0
			enc12old = 0
			enc22old = 0
			rospy.loginfo("DC Motor encoders reset")
			reset_encoders = False

		if newPosMsg and (v1old+v2old+v3old+v4old) == 0:
			mstods = 1127.87  # m/s to deg/s
			pos_base_speed = int(0.3 * mstods)
			cmd_enc11,cmd_enc12,cmd_enc21,cmd_enc22 = position_control()
			Motor12.setMotorTargets(pos_base_speed,cmd_enc11, pos_base_speed, cmd_enc12)
			Motor34.setMotorTargets(pos_base_speed,cmd_enc21, pos_base_speed, cmd_enc22)
			newPosMsg = False
			newPosMoving = True
		elif newPosMoving:
			Motor_Busy12 = Motor12.readMotorsBusy()
			Motor_Busy34 = Motor34.readMotorsBusy()
			Motor_Busy = Motor_Busy12 or Motor_Busy34
			if not Motor_Busy:
				newPosMoving = False
				#Motor12.stopMotors()
				#Motor34.stopMotors()
				global pos_cont_pub
				pos_cont_pub.publish(Bool(True))
		elif new_speed:
			# Set Motor speeds
			#if not (v1 == 0 or v2==0 or v3==0 or v4==0):
				#print('V1:' + str(v1)  + ' V2:' + str(v2) + ' V3:'+ str(v3) + ' V4:' + str(v4))
			Motor12.setMotorSpeeds(v1, v2)
			Motor34.setMotorSpeeds(v3, v4)
			[v1old, v2old, v3old, v4old] = [v1, v2, v3, v4]
			new_speed = False

		# Update odom position
		time.sleep(0.01)
		posupdate()
		#time.sleep(0.01)

		# Wait for next loop
		r.sleep()
		end = datetime.datetime.now()
		time_diff = end - start
		print_st = str(time_diff.microseconds/1000) + " ms"
		#print(print_st)
		if time_diff.microseconds > 50000:
			print("Over 50 ms: " + print_st)


if __name__ == '__main__':
	listener()
