#!/usr/bin/env python

import rospy
import message_filters
from geometry_msgs.msg import TransformStamped
import tf
from math import sqrt
import matplotlib.pyplot as plt
import cv2
import numpy as np

class Processor:
    def __init__(self):
        self.robot_transform_sub = message_filters.Subscriber("/vicon/priss_follower/priss_follower", TransformStamped)
        self.person_transform_sub = message_filters.Subscriber("/vicon/helmet_follower/helmet_follower", TransformStamped)
        self.vicon_time_synchronizer = message_filters.TimeSynchronizer(
            [self.robot_transform_sub, self.person_transform_sub], 
            10
        )
        self.vicon_time_synchronizer.registerCallback(self.viconCallback)
        self.tf_transformer = tf.TransformerROS(True, rospy.Duration(10.0))

        self.range_data = []
        self.robot_positions = []
        self.person_positions = []
        self.timestamps = []
        self.start_time_offset = 0.0


    def viconCallback(self, robot_transform, person_transform):
        self.tf_transformer.setTransform(robot_transform)
        self.tf_transformer.setTransform(person_transform)
        (relative_position, relative_quaternion) = self.tf_transformer.lookupTransform(
            robot_transform.child_frame_id, person_transform.child_frame_id, rospy.Time(0)
        )
        
        current_range = sqrt( relative_position[0]**2 + relative_position[1]**2 )
        is_okay = True
        if len(self.range_data):    
            previous_range = self.range_data[-1]
            if abs(current_range - previous_range) > 0.15:
                is_okay = False
        else:
            self.start_time_offset = robot_transform.header.stamp.to_sec()
            
        if is_okay:
            self.range_data.append(current_range)
            self.timestamps.append(robot_transform.header.stamp.to_sec() - self.start_time_offset)
            self.robot_positions.append(
                (robot_transform.transform.translation.x, robot_transform.transform.translation.y)
            )
            self.person_positions.append(
                (person_transform.transform.translation.x, person_transform.transform.translation.y)
            )

    def plot(self):
        plt.plot(self.timestamps, self.range_data)
        self.showTrajectory()
        self.showCorrespondingPoint()
        cv2.waitKey(0)
        plt.show()
        

    def showTrajectory(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)

        ax.plot( 
            map(lambda x: x[0], self.robot_positions) , 
            map(lambda x: x[1], self.robot_positions), 
            label="robot trajectory"
        )
        ax.plot( 
            map(lambda x: x[0], self.person_positions) , 
            map(lambda x: x[1], self.person_positions), 
            label="person trajectory"
        )
        
        ax.legend()
        ax.set_xlabel("x [m]")
        ax.set_xlabel("y [m]")

    def showCorrespondingPoint(self):
        skip = 40 # 80
        
        # size of image meters
        # metric_size = 9.0
        # image_size = 600
        # img = np.ones((image_size, image_size, 3), dtype=np.uint8)
        # img *= 255

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.hold(True)
        ax.grid(True)

        for i in range(0, len(self.robot_positions), skip):
            
            ax.plot(
                [self.robot_positions[i][0], self.person_positions[i][0]], [self.robot_positions[i][1], self.person_positions[i][1]], 'c'
            )

            ax.plot([self.robot_positions[i][0]], [self.robot_positions[i][1]], 'ro')
            ax.plot([self.person_positions[i][0]], [self.person_positions[i][1]], 'b^')

            # robot_image_coords = (
            #     int(round( image_size/2.0 + self.robot_positions[i][0] * image_size / metric_size )),
            #     int(round( image_size/2.0 + self.robot_positions[i][1] * image_size / metric_size ))
            # )
            
            # person_image_coords = (
            #     int(round( image_size/2.0 + self.person_positions[i][0] * image_size / metric_size )),
            #     int(round( image_size/2.0 + self.person_positions[i][1] * image_size / metric_size ))
            # )

            # cv2.circle(img, robot_image_coords, 4, (255, 0, 0), -1)
            # cv2.rectangle(
            #     img, 
            #     (person_image_coords[0]-3, person_image_coords[0]-3), 
            #     (person_image_coords[0]+3, person_image_coords[0]+3), 
            #     (255, 255, 0), 
            #     -1
            # )
            # cv2.line(img, robot_image_coords, person_image_coords, (255, 0, 255), 1)
        # cv2.imshow("window", img)

if __name__ == "__main__":
    processor = Processor()
    rospy.init_node('vicon_processor')
    rospy.on_shutdown(processor.plot)

    rospy.spin()
    