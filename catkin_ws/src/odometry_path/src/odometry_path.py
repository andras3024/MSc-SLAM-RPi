#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import numpy as np
import time
from std_srvs.srv import Empty


# Node initialization
rospy.init_node('path_plotter')
# Variable initialization
xold = 0.0
yold = 0.0
thetaold = 0.0
cont = 0
# Path message
path = Path()
# Path publisher initialization
path_name = rospy.get_param('~path_name', '/path')
pub = rospy.Publisher(path_name, Path, queue_size=1)

def handle_reset_path(self):
    global path,cont
    del path.poses[:]
    cont = 0
    return []

def callback(data):
    global xold, yold, thetaold
    global cont
    global pub, path

    # Max size of array pose msg from the path
    max_append = 1000
    # Pose message
    pose = PoseStamped()
    # Set a attributes of the msg
    pose.header.frame_id = "odom"
    pose.pose.position.x = float(data.pose.pose.position.x)
    pose.pose.position.y = float(data.pose.pose.position.y)
    pose.pose.orientation.x = float(data.pose.pose.orientation.x)
    pose.pose.orientation.y = float(data.pose.pose.orientation.y)
    pose.pose.orientation.z = float(data.pose.pose.orientation.z)
    pose.pose.orientation.w = float(data.pose.pose.orientation.w)
    # Quaternion to Euler transformation
    quat_arr = np.array([pose.pose.orientation.x, pose.pose.orientation.y,  pose.pose.orientation.z,  pose.pose.orientation.w])
    y = tf.transformations.euler_from_quaternion(quat_arr, 'sxyz')[2]
    # To avoid repeating the values
    if abs(xold - pose.pose.position.x) > 0.01 or abs(yold - pose.pose.position.y) > 0.01 or abs(thetaold - y) > 0.0018:
        #Set further attributes of the msg
        pose.header.seq = path.header.seq + 1
        path.header.frame_id="odom"
        path.header.stamp=rospy.Time.now()
        pose.header.stamp = path.header.stamp
        # Check path array size
        cont = cont + 1
        if cont > max_append:
            path.poses.pop(0)
        # Add new pose message to array
        path.poses.append(pose)
        # Publish the msg
        pub.publish(path)
        # Save the last positions
        xold=pose.pose.position.x
        yold=pose.pose.position.y
        thetaold=y

if __name__ == '__main__':
    # Get parameters values
    topic_name = rospy.get_param('~topic_name', '/odom')
    # Subscription to the Odometry topic
    rospy.Subscriber(topic_name, Odometry, callback)
    # Reset path service
    rospy.Service('/reset_path', Empty, handle_reset_path)

    rate = rospy.Rate(30) # 30hz
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
