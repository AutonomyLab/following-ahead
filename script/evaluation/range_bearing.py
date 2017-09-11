#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, PointStamped 
import tf
from math import sqrt, atan2, pi
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

font = {
    'family' : 'Times',
    'weight' : 'bold',
    'size'   : 50,
}

axes = {
    'unicode_minus' : False
}

matplotlib.rc('font', **font)
matplotlib.rc('axes', **axes)

class RangeBearingProcessor:
    def __init__(self):
        rospy.Subscriber(
            "/vicon/helmet_follower/helmet_follower", # "/person_follower/groundtruth_pose", 
            TransformStamped, self.relativePoseCallback
        )
        
        self.range_data = []
        self.bearing_data = []
        self.timestamps = []
        self.start_time_offset = 0.0

        self.tf_listener = tf.TransformListener()

    def relativePoseCallback(self, relative_pose):
        time = self.tf_listener.getLatestCommonTime("/base_link", "/map")
        
        # T_map_person = PointStamped()
        # T_map_person.header = relative_pose.header
        # T_map_person.header.stamp = time
        # T_map_person.point.x = relative_pose.transform.translation.x
        # T_map_person.point.y = relative_pose.transform.translation.y
        # T_map_person.point.z = relative_pose.transform.translation.z
        
        try:
            # T_robot_person = self.tf_listener.transformPoint("/base_link", T_map_person)
            # position = (T_map_person.point.x, T_map_person.point.y, T_map_person.point.z)
            # quaternion = (0, 0, 0, 1)
            (position, quaternion) = self.tf_listener.lookupTransform("/base_link", "/relative_pose", rospy.Time(0))
        except all:
            return
        
        current_range = sqrt( position[0]**2 + position[1]**2 )
        current_bearing = atan2(position[1], -position[0]) * 180 / pi

        is_okay = True
        if len(self.range_data):    
            previous_range = self.range_data[-1]
            if abs(current_range - previous_range) > 0.15:
                is_okay = False
        else:
            self.start_time_offset = time.to_sec()
            
        if is_okay:
            self.range_data.append(current_range)
            self.bearing_data.append(current_bearing)
            self.timestamps.append(time.to_sec() - self.start_time_offset)

    def plot(self):
        plt.plot(self.timestamps, self.bearing_data)
        plt.yticks(np.arange(-20, 80, 40))
        plt.xlabel("time [sec]")
        plt.ylabel("bearing [degree]")

        # plt.figure()
        # plt.plot(self.timestamps, self.range_data)
        # plt.xlabel("time [sec]")
        # plt.ylabel("range [m]")

        plt.show()


if __name__ == "__main__":
    rospy.init_node('range_bearing', anonymous=True)
    range_bearing_processor = RangeBearingProcessor()
    rospy.on_shutdown(range_bearing_processor.plot)

    rospy.spin()



