#!/usr/bin/env python
#
# Dummy node publishing dummy estimation data
#
#

import rospy
import math
import numpy as np

from testbed_msgs.msg import CustOdometryStamped
from geometry_msgs.msg import PoseStamped

# Function to generate Target Pose message
def genTRGmsg(p, q):
    pose_msg = PoseStamped()

    pose_msg.pose.position.x = p[0] 
    pose_msg.pose.position.y = p[1]
    pose_msg.pose.position.z = p[2]

    pose_msg.pose.orientation.w = q[0]
    pose_msg.pose.orientation.x = q[1]
    pose_msg.pose.orientation.y = q[2]
    pose_msg.pose.orientation.z = q[3]

    return pose_msg

def genObstaclemsg(p, q):
    pose_msg = PoseStamped()

    pose_msg.pose.position.x = p[0] 
    pose_msg.pose.position.y = p[1]
    pose_msg.pose.position.z = p[2]

    pose_msg.pose.orientation.w = q[0]
    pose_msg.pose.orientation.x = q[1]
    pose_msg.pose.orientation.y = q[2]
    pose_msg.pose.orientation.z = q[3]

    return pose_msg

def genDronemsg(p, q):
    pose_msg = PoseStamped()

    pose_msg.pose.position.x = p[0] 
    pose_msg.pose.position.y = p[1]
    pose_msg.pose.position.z = p[2]

    pose_msg.pose.orientation.w = q[0]
    pose_msg.pose.orientation.x = q[1]
    pose_msg.pose.orientation.y = q[2]
    pose_msg.pose.orientation.z = q[3]

    return pose_msg


if __name__ == "__main__":
    rospy.init_node("Dummy_Node")
    
    freq = 30.0
    # Rate of the publication
    rate = rospy.Rate(freq) 
   
    dummy_codom_pub = rospy.Publisher("/cf2/external_codom", 
            CustOdometryStamped, queue_size=10)

    dummy_target_pub = rospy.Publisher("/vrpn_client_node/target/pose", 
            PoseStamped, queue_size=10)

    dummy_obstacle_pub = rospy.Publisher("/nodeB/external_pose", 
            PoseStamped, queue_size=10)

    dummy_drone_pub = rospy.Publisher("/vrpn_client_node/cf2/pose", 
            PoseStamped, queue_size=10)

    msg_vehicle = CustOdometryStamped()
    msg_vehicle.p.x = 1.0
    msg_vehicle.p.y = 0.0
    msg_vehicle.p.z = 0.7
    msg_vehicle.q.w = 1.0 

    # Y up
    tg_pos = np.array([2.0, 0.3, 0.8])
    angle = (math.pi/2)
    tg_q = np.array([math.cos(angle/2), 0.0, 0.0, math.sin(angle/2)]) 
    msg_target = genTRGmsg(tg_pos, tg_q)

    obstacle_p = np.array([1.0, 0.0, 0.0])
    obstacle_q = np.array([1,0, 0,0, 0.0, 0.0])
    msg_obstacle = genObstaclemsg(obstacle_p, obstacle_q) 

    msg_drone = genDronemsg([0,1,0],[1,0,0,0])

    while (not rospy.is_shutdown()):
        pass
        dummy_codom_pub.publish(msg_vehicle)    
        dummy_target_pub.publish(msg_target)
        dummy_obstacle_pub.publish(msg_obstacle)
        dummy_drone_pub.publish(msg_drone)

        rate.sleep()
