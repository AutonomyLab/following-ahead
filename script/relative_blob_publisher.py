#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
import message_filters
import tf as transform
import numpy as np
from stage_ros.msg import BlobDetection
from stage_ros.msg import Blobs

tfObj = transform.TransformerROS(True, rospy.Duration(10.0))
relativePosePub = rospy.Publisher('/person_follower/groundtruth_pose', TransformStamped, queue_size=1)

def callback_blob_f(blobs_msg):
    print (len(blobs_msg.detections))
    if (len(blobs_msg.detections) > 0):

    # relativePoseMsg = TransformStamped()
    # relativePoseMsg.header.frame_id = "world"
    # relativePoseMsg.child_frame_id = "person_follower/helmet_relative"
    # relativePoseMsg.transform.translation.x = relativePose[0, 3]
    # relativePoseMsg.transform.translation.y = relativePose[1, 3]
    # relativePoseMsg.transform.translation.z = relativePose[2, 3]
    # relativePoseMsg.transform.rotation.x = relativeQuaternion[0]
    # relativePoseMsg.transform.rotation.y = relativeQuaternion[1]
    # relativePoseMsg.transform.rotation.z = relativeQuaternion[2]
    # relativePoseMsg.transform.rotation.w = relativeQuaternion[3]
    # relativePosePub.publish(relativePoseMsg)

    # br = transform.TransformBroadcaster()
    # br.sendTransform(
    #     (relativePose[0, 3], relativePose[1, 3], relativePose[2, 3]),
    #     relativeQuaternion,
    #     rospy.Time.now(),
    #     relativePoseMsg.child_frame_id,
    #     "world"
    # )      

def callback_blob_b(blobs_msg):
    print (len(blobs_msg.detections))
    if (len(blobs_msg.detections) > 0):

if __name__ == '__main__':
    rospy.init_node('node_name')

    rospy.Subscriber('/robot_1/blob_0', Blobs, callback_blob_f)
    rospy.Subscriber('/robot_1/blob_1', Blobs, callback_blob_b)

    rospy.spin()