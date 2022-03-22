#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import marvelmind_nav.msg

to_publish_marvelmind_pose = False
marvelmind_pose = geometry_msgs.msg.PoseStamped()
aruco_pose_publisher = None


def run_marvelmind_pose_publisher():
    marvelmind_pose_publisher  = rospy.Publisher("marvelmind_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
    global to_publish_marvelmind_pose
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if to_publish_marvelmind_pose:
            marvelmind_pose_publisher.publish(marvelmind_pose)
            to_publish_marvelmind_pose = False
        rate.sleep()

def run_aruco_pose_publisher():
    pass

def hedge_pos_callback(msg):
    global marvelmind_pose
    global to_publish_marvelmind_pose
    marvelmind_pose.pose.position.x = msg.x_m
    marvelmind_pose.pose.position.y = msg.y_m
    marvelmind_pose.pose.position.z = msg.z_m
    marvelmind_pose.pose.orientation.w = 1
    marvelmind_pose.pose.orientation.x = 0
    marvelmind_pose.pose.orientation.y = 0
    marvelmind_pose.pose.orientation.z = 0
    to_publish_marvelmind_pose = True


def aruco_pos_callback(msg):
    pass

if __name__ == "__main__":
    rospy.init_node("drone_pose_publisher")
    # Subscribers
    rospy.Subscriber("hedge_pos", marvelmind_nav.msg.hedge_pos, hedge_pos_callback)
    # Publishers
    run_marvelmind_pose_publisher()
    rospy.spin()