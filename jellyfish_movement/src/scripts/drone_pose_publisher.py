#!/usr/bin/env python

from decimal import DivisionByZero
import rospy
import geometry_msgs.msg
import marvelmind_nav.msg
import tf2_ros
import tf_conversions
import math

drone_name = "djeye_m210" #Default drone name, can be set through param servers
to_publish_marvelmind_pose = False
hedgehog_1_address = None
hedgehog_2_address = None
hedgehog_coordinates = {}
center_of_hedgehogs_relative_to_base_link = [0, 0, 0]
hedgehog_rotation_relative_to_base_link_q = [0, 0, 0, 0]
pub_marvelmind = True

def initialise():
    global drone_name
    global hedgehog_1_address
    global hedgehog_2_address
    global center_of_hedgehogs_relative_to_base_link
    global pub_marvelmind
    global hedgehog_coordinates

    if rospy.has_param("drone_name"):
        drone_name = rospy.get_param("drone_name")
    else:
        rospy.logerr("Cannot find param drone name on parameter server. Might want to check that... Defaulting to 'dji_m210'")
        drone_name = "dji_m210"
    if (rospy.has_param("hedgehog_1_address") and 
        rospy.has_param("hedgehog_2_address") and 
        rospy.has_param("center_of_hedgehogs_relative_to_base_link")):

        hedgehog_1_address = rospy.get_param("hedgehog_1_address")
        hedgehog_2_address = rospy.get_param("hedgehog_2_address")
        center_of_hedgehogs_relative_to_base_link = rospy.get_param("center_of_hedgehogs_relative_to_base_link")
        hedgehog_coordinates[hedgehog_1_address] = [0, 0, 0]
        hedgehog_coordinates[hedgehog_2_address] = [0, 0, 0]
        if rospy.has_param("center_of_hedgehogs_relative_to_base_link"):
            center_of_hedgehogs_relative_to_base_link = rospy.get_param("center_of_hedgehogs_relative_to_base_link")
        else:
            rospy.logerr("Cannot find param center_of_hedgehogs_relative_to_base_link on parameter server. Might want to check that... Defaulting to [0, 0, 0]")
    else:
        rospy.logerr("Cannot find param hedgehog_positions on parameter server. Might want to check that... We shall run without marvelmind")
        pub_marvelmind = False
        
def marvelmind_pose_publisher(pub, r):
    global to_publish_marvelmind_pose

    if to_publish_marvelmind_pose:
        marvelmind_pose = geometry_msgs.msg.PoseStamped()
        marvelmind_pose.header.stamp = rospy.Time.now()
        marvelmind_pose.header.frame_id = "{}_to_marvelmind".format(drone_name)
        hedgehog_1_coordinates = hedgehog_coordinates[hedgehog_1_address]
        hedgehog_2_coordinates = hedgehog_coordinates[hedgehog_2_address]
        hedgehog_center = [(hedgehog_1_coordinates[i] + hedgehog_2_coordinates[i])/2 for i in range(3)]
        hedgehog_vector = [(hedgehog_2_coordinates[i] - hedgehog_1_coordinates[i]) for i in range(3)]
        try:
            hedgehog_roll =  math.atan(hedgehog_vector[2]/hedgehog_vector[0])
        except (DivisionByZero, ZeroDivisionError):
            hedgehog_roll = math.pi
        try:
            hedgehog_yaw =  math.atan(hedgehog_vector[1]/hedgehog_vector[0])
        except (DivisionByZero, ZeroDivisionError):
            hedgehog_yaw = math.pi
        #pitch is assumed to be 0, since we cannot get that info from the marvelmind beacons (and we dont need it anyway)
        hedgehog_pitch = 0
        hedgehog_q = tf_conversions.transformations.quaternion_from_euler(hedgehog_roll, hedgehog_pitch, hedgehog_yaw)
        marvelmind_pose.pose.position.x = hedgehog_center[0] - center_of_hedgehogs_relative_to_base_link[0]
        marvelmind_pose.pose.position.y = hedgehog_center[1] - center_of_hedgehogs_relative_to_base_link[1]
        marvelmind_pose.pose.position.z = hedgehog_center[2] - center_of_hedgehogs_relative_to_base_link[2]
        marvelmind_pose.pose.orientation.x = hedgehog_q[0]
        marvelmind_pose.pose.orientation.y = hedgehog_q[1]
        marvelmind_pose.pose.orientation.z = hedgehog_q[2]
        marvelmind_pose.pose.orientation.w = hedgehog_q[3]
        pub.publish(marvelmind_pose)
        to_publish_marvelmind_pose = False
    r.sleep()

def aruco_pose_publisher(pub, tf_Buffer, r):
    try:
        trans = tf_Buffer.lookup_transform("{}_base_link".format(drone_name), "{}_aruco_rotated".format(drone_name), rospy.Time())
        pose_msg = geometry_msgs.msg.PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.orientation = trans.transform.rotation
        pose_msg.pose.position = trans.transform.translation
        pub.publish(pose_msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass
    r.sleep()

def hedge_pos_callback(msg):
    global to_publish_marvelmind_pose
    global hedgehog_coordinates

    hedgehog_coordinates[msg.address] = [msg.x_m, msg.y_m, msg.z_m]
    to_publish_marvelmind_pose = True


if __name__ == "__main__":
    rospy.init_node("drone_pose_publisher")
    initialise()
    # Subscribers
    rospy.Subscriber("hedge_pos_a", marvelmind_nav.msg.hedge_pos_a, hedge_pos_callback)
    # Publishers
    tf_Buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_Buffer)
    marvelmind_pose_pub  = rospy.Publisher("marvelmind_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
    rospy.loginfo("Publishing to marvelmind_pose")
    marvelmind_pose_pub_rate = rospy.Rate(5)
    aruco_pose_pub = rospy.Publisher("aruco_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
    aruco_pose_pub_rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if pub_marvelmind:
            marvelmind_pose_publisher(marvelmind_pose_pub, marvelmind_pose_pub_rate)
        aruco_pose_publisher(aruco_pose_pub, tf_Buffer, aruco_pose_pub_rate)
    rospy.spin()