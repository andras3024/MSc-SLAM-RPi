#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Bool
import time
import geometry_msgs.msg
from geometry_msgs.msg import Twist,Pose
from math import radians,degrees
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pos_ready = True
pos_steps = 0

def get_rotation (msg):
    global roll, pitch, yaw, posx, posy, yaw_deg
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw_deg = degrees(yaw)
    if yaw_deg < 0:
        yaw_deg += 360.0  # Convert negative to positive angles
    posx = msg.pose.pose.position.x
    posy = msg.pose.pose.position.y

def get_pos_ready (msg):
    global pos_ready
    pos_ready = True

def talker():
    # Reset encoders
    resEnc = rospy.ServiceProxy('/reset_encoders', Empty)
    resEnc()
    # Reset path
    resPath = rospy.ServiceProxy('/reset_path', Empty)
    resPath()
    # Bool for print functions
    global pos_bool, pos_ready
    # Ros init
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
    pos_pub = rospy.Publisher('/cmd_pos', Pose, queue_size=3)
    rospy.init_node('DCMotorVarTest', anonymous=True)
    rospy.Subscriber('/odom', Odometry, get_rotation)
    rospy.Subscriber('/robot/positon_ready', Bool, get_pos_ready)
    rospy.wait_for_message('/odom', Odometry) # Delay until next odom message
    rospy.loginfo("X - 0, Y - 0 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy), str(yaw_deg))
    rate = rospy.Rate(10)  # 10hz
    # Velocity message
    vel_msg = Twist()
    vel_x = 0.0
    vel_y = 0.0
    ang_z = 0.0
    rospy.loginfo("Let's move the robot")
    zero_time = time.time()
    sign = 1
    run_number = 0
    max_run_number = 1
    pos_bool= False
    vel_control = True
    while not rospy.is_shutdown():
        # Timed velocity control
        if vel_control:
            time_move = time.time() - zero_time
            if  time_move <  0.5:
                vel_x = time_move*sign*0.5
            elif time_move < 4.0:
                vel_x = 0.25*sign
            elif time_move < 4.5:
                vel_x = abs(time_move - 4.5)*sign*0.5
                pos_bool = True
            elif time_move < 4.8:
                vel_x = 0
            elif time_move < 5:
                if pos_bool:
                    if sign == 1:
                        rospy.loginfo("X - 1, Y - 0 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy),str(yaw_deg))
                    else:
                        rospy.loginfo("X - 0, Y - 1 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy),
                                      str(yaw_deg))
                    pos_bool = False
                vel_x = 0
            elif time_move < 5.5:
                vel_y = abs(time_move - 5)*sign*0.5
            elif time_move < 9:
                vel_y = 0.25*sign
            elif time_move < 9.5:
                vel_y = abs(time_move - 9.5)*sign*0.5
                pos_bool = True
            elif time_move < 10:
                vel_y = 0
            elif time_move < 10.2:
                if pos_bool:
                    if sign == 1:
                        rospy.loginfo("X - 1, Y - 1 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy),str(yaw_deg))
                    else:
                        rospy.loginfo("X - 0, Y - 0 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy),
                                      str(yaw_deg))
                    pos_bool = False
            else:
                vel_x = 0
                vel_y = 0
                if run_number < max_run_number:
                    zero_time = time.time()
                    sign = -sign
                else:
                    rospy.loginfo("Job done!")
                    rospy.signal_shutdown("Job done!")
                run_number += 1

            vel_msg.linear.x = vel_x
            vel_msg.linear.y = vel_y
            vel_msg.angular.z = ang_z
            pub.publish(vel_msg)
        else:
            if pos_ready:
                global pos_steps
                pos_steps += 1
                pos_ready = False
                if pos_steps == 1:
                    pos_msg = Pose()
                    pos_msg.position.x = 1.0
                    pos_msg.position.y = 0.0
                    pos_pub.publish(pos_msg)
                elif pos_steps == 2:
                    rospy.loginfo("X - 1, Y - 0 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy),
                                  str(yaw_deg))
                    pos_msg = Pose()
                    pos_msg.position.x = 0.0
                    pos_msg.position.y = 1.0
                    pos_pub.publish(pos_msg)
                elif pos_steps == 3:
                    rospy.loginfo("X - 1, Y - 1 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy),
                                  str(yaw_deg))
                    pos_msg = Pose()
                    pos_msg.position.x = -1.0
                    pos_msg.position.y = 0.0
                    pos_pub.publish(pos_msg)
                elif pos_steps == 4:
                    rospy.loginfo("X - 0, Y - 1 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy),
                                  str(yaw_deg))
                    pos_msg = Pose()
                    pos_msg.position.x = 0.0
                    pos_msg.position.y = -1.0
                    pos_pub.publish(pos_msg)
                else:
                    rospy.loginfo("X - 0, Y - 0 Real Pos x: %s m, Pos y: %s m, Theta: %s deg", str(posx), str(posy),
                                  str(yaw_deg))
                    rospy.loginfo("Job done!")
                    rospy.signal_shutdown("Job done!")

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass