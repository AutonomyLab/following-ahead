#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
import tf
from math import sqrt, atan2, pi
import matplotlib.pyplot as plt

class RangeBearingProcessor:
    def __init__(self):
        rospy.Subscriber("/person_follower/groundtruth_pose", TransformStamped, self.relativePoseCallback)
        
        self.range_data = []
        self.bearing_data = []
        self.timestamps = []
        self.start_time_offset = 0.0

        self.tf_listener = tf.TransformListener()

    def relativePoseCallback(self, relative_pose):
        time = self.tf_listener.getLatestCommonTime("/base_link", "/person_raw")
        (position, quaternion) = self.tf_listener.lookupTransform("/base_link", "/person_raw", rospy.Time(0))
        
        current_range = sqrt( position[0]**2 + position[1]**2 )
        current_bearing = atan2(position[1], -position[0]) * 180 / pi

        is_okay = True
        if len(self.range_data):    
            previous_range = self.range_data[-1]
            if abs(current_range - previous_range) > 0.25:
                is_okay = False
        else:
            self.start_time_offset = time.to_sec()
            
        if is_okay:
            self.range_data.append(current_range)
            self.bearing_data.append(current_bearing)
            self.timestamps.append(time.to_sec() - self.start_time_offset)

    def plot(self):
        plt.plot(self.timestamps, self.bearing_data)
        plt.figure()
        plt.plot(self.timestamps, self.range_data)
        plt.show()


if __name__ == "__main__":
    rospy.init_node('range_bearing', anonymous=True)
    range_bearing_processor = RangeBearingProcessor()
    rospy.on_shutdown(range_bearing_processor.plot)

    rospy.spin()



