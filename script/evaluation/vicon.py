#!/usr/bin/env python

import rospy
import message_filters
from geometry_msgs.msg import TransformStamped, PointStamped
import tf
from math import sqrt
import matplotlib
import matplotlib.pyplot as plt
import cv2
import numpy as np

PLOT_AXIS_FONT = 130
TRAJECTORY_AXIS_FONT = 45

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
                self.getPersonCorrected(person_transform) # (person_transform.transform.translation.x, person_transform.transform.translation.y)
            )

    def plot(self):
        font = {
            'family' : 'Times',
            'weight' : 'bold',
            'size'   : PLOT_AXIS_FONT,
        }

        axes = {
            'unicode_minus' : False
        }

        matplotlib.rc('font', **font)
        matplotlib.rc('axes', **axes)
        plt.plot(self.timestamps, self.range_data)
        plt.yticks(np.arange(.2, 2, .4))
        plt.xlabel("time [sec]")
        plt.ylabel("range [m]")

        font = {
            'family' : 'Times',
            'weight' : 'bold',
            'size'   : TRAJECTORY_AXIS_FONT,
        }

        axes = {
            'unicode_minus' : False
        }

        matplotlib.rc('font', **font)
        matplotlib.rc('axes', **axes)
        # self.showTrajectory()
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
        ax.set_ylabel("y [m]")

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
            
            markersize = 12
            markercolor_robot = 'r'
            markercolor_person = 'b'
            
            if i == 0:
                markersize = 20
                markercolor_robot = (1.0, 0.5, 0)
                markercolor_person = (0, 0.5, 1.0)
            elif (len(self.robot_positions) - i) < skip :
                markersize = 20
                markercolor_robot = (1.0, 1.0, 0)
                markercolor_person = (0, 1.0, 1.0)

            if i == 0:
                ax.plot([self.robot_positions[i][0]], [self.robot_positions[i][1]], color=markercolor_robot, markersize=markersize, marker='o', label="robot trajectory")
                ax.plot([self.person_positions[i][0]], [self.person_positions[i][1]], color=markercolor_person, markersize=markersize, marker='^', label="person trajectory")
            else:
                ax.plot([self.robot_positions[i][0]], [self.robot_positions[i][1]], color=markercolor_robot, markersize=markersize, marker='o', )
                ax.plot([self.person_positions[i][0]], [self.person_positions[i][1]], color=markercolor_person, markersize=markersize, marker='^', )

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
        # ax.legend(bbox_to_anchor=(1.1, 1.1))
        ax.set_aspect('equal', 'datalim')
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

if __name__ == "__main__":
    processor = Processor()
    rospy.init_node('vicon_processor')
    rospy.on_shutdown(processor.plot)

    rospy.spin()
    