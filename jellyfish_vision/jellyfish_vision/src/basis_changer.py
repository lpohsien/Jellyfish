#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np

new_basis = np.array([
    [0, 1, 0], 
    [1, 0, 0], 
    [0, 0, 1]])

# https://stackoverflow.com/questions/39098154/switch-chirality-of-a-quaternion-in-code Refer to this if the new_basis vector changes
def change_basis(msg, callback_args):
    tf_pub = callback_args[0]
    drone_name = callback_args[1]
    tf_msg = geometry_msgs.msg.TransformStamped()
    tf_msg.header.frame_id = f"{drone_name}_camera"
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.child_frame_id = f"{drone_name}_aruco_rotated"
    pos = msg.pose.position
    tvec = np.array([pos.x, pos.y, pos.z])
    new_pos = new_basis @ tvec
    tf_msg.transform.translation.x = new_pos[0]
    tf_msg.transform.translation.y = new_pos[1]
    tf_msg.transform.translation.z = new_pos[2]
    # The imaginary components are inverted as the chirality of the coordinate system has been changed, and the x and y has been swapped
    # due to the new basis vectors
    tf_msg.transform.rotation.x = -msg.pose.orientation.y
    tf_msg.transform.rotation.y = -msg.pose.orientation.x
    tf_msg.transform.rotation.z = -msg.pose.orientation.z
    tf_msg.transform.rotation.w = msg.pose.orientation.w
    tf_pub.sendTransform(tf_msg)



if __name__ == "__main__":
    # Init things
    rospy.init_node("aruco_basis_changer")
    if rospy.has_param("drone_name"):
        drone_name = rospy.get_param("drone_name")
    else:
        drone_name = "dji_m210"
        rospy.logwarn(f"Could not find drone name, defaulting to {drone_name}")
    tf_pub = tf2_ros.TransformBroadcaster()    
    # Subscribers
    rospy.Subscriber(f"{drone_name}_aruco_node/pose", geometry_msgs.msg.PoseStamped, change_basis, callback_args = (tf_pub, drone_name))
    rospy.spin()
