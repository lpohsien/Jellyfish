#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import marvelmind_nav.msg
import tf2_ros

drone_name = "djeye_m210" #Default drone name, can be set through param servers
to_publish_marvelmind_pose = False
marvelmind_pose = geometry_msgs.msg.PoseStamped()

def initialise():
    global drone_name

    drone_name = rospy.get_param("drone_name")

def marvelmind_pose_publisher(pub, r):
    global to_publish_marvelmind_pose

    if to_publish_marvelmind_pose:
        marvelmind_pose.header.stamp = rospy.Time.now()
        pub.publish(marvelmind_pose)
        to_publish_marvelmind_pose = False
    r.sleep()

def aruco_pose_publisher(pub, tf_Buffer, r):
    try:
        trans = tf_Buffer.lookup_transform(f"{drone_name}_base_link", "aruco", rospy.Time())
        pose_msg = geometry_msgs.msg.PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.orientation = trans.transform.rotation
        pose_msg.pose.position = trans.transform.translation
        pub.publish(pose_msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass
    r.sleep()

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


if __name__ == "__main__":
    rospy.init_node("drone_pose_publisher")
    initialise()
    # Subscribers
    rospy.Subscriber("hedge_pos", marvelmind_nav.msg.hedge_pos, hedge_pos_callback)
    # Publishers
    tf_Buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_Buffer)
    marvelmind_pose_pub  = rospy.Publisher("marvelmind_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
    marvelmind_pose_pub_rate = rospy.Rate(5)
    aruco_pose_pub = rospy.Publisher("aruco_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
    aruco_pose_pub_rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        marvelmind_pose_publisher(marvelmind_pose_pub, marvelmind_pose_pub_rate)
        aruco_pose_publisher(aruco_pose_pub, tf_Buffer, aruco_pose_pub_rate)
    rospy.spin()