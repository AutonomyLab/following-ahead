#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
import message_filters
import tf as transform
import numpy as np

tfObj = transform.TransformerROS(True, rospy.Duration(10.0))
relativePosePub = rospy.Publisher('/person_follower/groundtruth_pose', TransformStamped, queue_size=1)

def callback(helmetMsg, kinectMsg):
    helmetPose = tfObj.fromTranslationRotation(
        (helmetMsg.transform.translation.x, helmetMsg.transform.translation.y, helmetMsg.transform.translation.z),
        (helmetMsg.transform.rotation.x, helmetMsg.transform.rotation.y, helmetMsg.transform.rotation.z, helmetMsg.transform.rotation.w)
    )

    kinectPose = tfObj.fromTranslationRotation(
        (kinectMsg.transform.translation.x, kinectMsg.transform.translation.y, kinectMsg.transform.translation.z),
        (kinectMsg.transform.rotation.x, kinectMsg.transform.rotation.y, kinectMsg.transform.rotation.z, kinectMsg.transform.rotation.w)
    )

    relativePose = np.dot(np.linalg.inv(kinectPose), helmetPose) 
    relativeQuaternion = transform.transformations.quaternion_from_matrix(relativePose)

    relativePoseMsg = TransformStamped()
    relativePoseMsg.header.frame_id = "base_link"
    relativePoseMsg.child_frame_id = "person_follower/helmet_relative"
    relativePoseMsg.transform.translation.x = relativePose[0, 3]
    relativePoseMsg.transform.translation.y = relativePose[1, 3]
    relativePoseMsg.transform.translation.z = relativePose[2, 3]
    relativePoseMsg.transform.rotation.x = relativeQuaternion[0]
    relativePoseMsg.transform.rotation.y = relativeQuaternion[1]
    relativePoseMsg.transform.rotation.z = relativeQuaternion[2]
    relativePoseMsg.transform.rotation.w = relativeQuaternion[3]
    relativePosePub.publish(relativePoseMsg)

    br = transform.TransformBroadcaster()
    br.sendTransform(
        (relativePose[0, 3], relativePose[1, 3], relativePose[2, 3]),
        relativeQuaternion,
        rospy.Time.now(),
        relativePoseMsg.child_frame_id,
        "base_link"
    )

if __name__ == '__main__':
    rospy.init_node('node_name')

    helmetSub = message_filters.Subscriber('/vicon/helmet2/helmet2', TransformStamped)
    kinectSub = message_filters.Subscriber('/vicon/priss_follower/priss_follower', TransformStamped)

    ts = message_filters.TimeSynchronizer([helmetSub, kinectSub], 10)
    ts.registerCallback(callback)
    rospy.spin()