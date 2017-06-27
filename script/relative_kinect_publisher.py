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

    relativePoseMsg = TransformStamped()
    relativePoseMsg.header.frame_id = "world"
    relativePoseMsg.child_frame_id = "person_follower/helmet_relative"
    relativePosePub.publish(relativePoseMsg)

    br = transform.TransformBroadcaster()
    br.sendTransform(
        (relativePose[0, 3], relativePose[1, 3], relativePose[2, 3]),
        transform.transformations.quaternion_from_matrix(relativePose),
        rospy.Time.now(),
        relativePoseMsg.child_frame_id,
        "world"
    )

if __name__ == '__main__':
    rospy.init_node('node_name')

    helmetSub = message_filters.Subscriber('/vicon/helmet/helmet', TransformStamped)
    kinectSub = message_filters.Subscriber('/vicon/kkk/kkk', TransformStamped)

    ts = message_filters.TimeSynchronizer([helmetSub, kinectSub], 10)
    ts.registerCallback(callback)
    rospy.spin()