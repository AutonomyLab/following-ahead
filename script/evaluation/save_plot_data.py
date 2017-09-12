#!/usr/bin/env python

import rospy
import message_filters
from geometry_msgs.msg import TransformStamped, PointStamped
import tf
from math import sqrt, atan2, pi
import numpy as np

class SavePlotData:
    '''
    data saved in the format 
        timestamp, range, robot_x, robot_y, person_x, person_y
        timestamp, bearing
    '''
    def __init__(self):
        self.robot_transform_sub = message_filters.Subscriber("/vicon/priss_follower/priss_follower", TransformStamped)
        self.person_transform_sub = message_filters.Subscriber("/vicon/helmet_follower/helmet_follower", TransformStamped)
        self.vicon_time_synchronizer = message_filters.TimeSynchronizer(
            [self.robot_transform_sub, self.person_transform_sub], 
            10
        )
        self.vicon_time_synchronizer.registerCallback(self.viconCallback)
        self.tf_transformer = tf.TransformerROS(True, rospy.Duration(10.0))
        self.tf_listener = tf.TransformListener()

        self.range_data = []
        self.bearing_data = []
        self.robot_positions = []
        self.person_positions = []
        self.timestamps = []
        self.bearing_timestamps = []
        self.start_time_offset = 0.0

    def getPersonCorrected(self, person_transform):
        person_point = PointStamped()
        ## square
        # person_point.point.x = 0
        # person_point.point.y = 0.5
        # person_point.point.z = 0

        ## eight-1
        person_point.point.x = 0.5
        person_point.point.y = 0
        person_point.point.z = 0

        person_point.header.stamp = person_transform.header.stamp
        person_point.header.frame_id = person_transform.child_frame_id

        person_point_corrected = self.tf_transformer.transformPoint(person_transform.header.frame_id, person_point)
        return (
            person_point_corrected.point.x,
            person_point_corrected.point.y,
            person_point_corrected.point.x
        )

    def viconCallback(self, robot_transform, person_transform):
        ## ------------------------ Absolute positions ------------------------ ##
        self.tf_transformer.setTransform(robot_transform)
        self.tf_transformer.setTransform(person_transform)
        (relative_position, relative_quaternion) = self.tf_transformer.lookupTransform(
            robot_transform.child_frame_id, person_transform.child_frame_id, rospy.Time(0)
        )
        
        current_range = sqrt( relative_position[0]**2 + relative_position[1]**2 )
        
        ## ------------------------ Relative positions ------------------------ ##
        is_bearing = True
        try:
            (relative_position, _) = self.tf_listener.lookupTransform("/base_link", "/relative_pose", rospy.Time(0))
        except:
            rospy.logwarn("CAN'T GET RELATIVE POSE")
            is_bearing = False
        
        current_bearing = atan2(relative_position[1], -relative_position[0]) * 180 / pi

        is_okay = True
        if len(self.range_data):    
            previous_range = self.range_data[-1]
            if abs(current_range - previous_range) > 0.15:
                is_okay = False
            
        if is_okay:
            self.timestamps.append(robot_transform.header.stamp.to_sec() - self.start_time_offset)
            self.range_data.append(current_range)
            self.robot_positions.append(
                (robot_transform.transform.translation.x, robot_transform.transform.translation.y)
            )
            self.person_positions.append(
                self.getPersonCorrected(person_transform) # (person_transform.transform.translation.x, person_transform.transform.translation.y)
            )

            if is_bearing:
                self.bearing_data.append(current_bearing)
                self.bearing_timestamps.append(robot_transform.header.stamp.to_sec() - self.start_time_offset)

    def save(self):
        array = np.zeros((len(self.timestamps), 6))
        for i in range(len(self.timestamps)):
            array[i, 0] = self.timestamps[i]
            array[i, 1] = self.range_data[i]
            array[i, 2] = self.robot_positions[i][0]
            array[i, 3] = self.robot_positions[i][1]
            array[i, 4] = self.person_positions[i][0]
            array[i, 5] = self.person_positions[i][1]
        np.savetxt("trajectory_data.txt", array)

        array = np.zeros((len(self.bearing_timestamps), 2))
        for i in range(len(self.bearing_timestamps)):
            array[i, 0] = self.bearing_timestamps[i]
            array[i, 1] = self.bearing_data[i]
        np.savetxt("bearing_data.txt", array)
if __name__ == "__main__":
    processor = SavePlotData()
    rospy.init_node('save_vicon_data')
    rospy.on_shutdown(processor.save)

    rospy.spin()