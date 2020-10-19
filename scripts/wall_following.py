#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 50
kd = 0.05
ki = 0
servo_offset = 0.0
prev_error = 0.0 
# error = 0.0
integral = 0.0
theta = 40

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/opp_drive' # originally /nav

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1000) #TODO: Publish to drive

    def getRange(self, data, angle):
        if angle > 179.9:
            angle = 179.9

        index = len(data.ranges)*(angle + 45)/ANGLE_RANGE
        dist = data.ranges[int(index)]
        if math.isinf(dist) or math.isnan(dist):
            return 4.0
        return dist

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0

        angle = servo_offset
        # error = 5*error
        control_error = kp*error + kd*(error - prev_error)# + ki*integral
        angle = angle + control_error*np.pi/180

        prev_error = error
        if angle > 30*np.pi/180:
            angle = 30*np.pi/180
        if angle < -30*np.pi/180:
            angle = -30*np.pi/180

        if angle > 0*np.pi/180 or angle < -0*np.pi/180:
            velocity = VELOCITY * 3/4

        if angle > 10*np.pi/180 or angle < -10*np.pi/180:
            velocity = VELOCITY * 2/4

        if angle > 20*np.pi/180 or angle < -20*np.pi/180:
            velocity = VELOCITY * 1/4

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data):
        #Follow left wall as per the algorithm 
        #TODO:implement
        a = self.getRange(data, 180-theta)
        b = self.getRange(data, 179.9)
        swing = math.radians(theta)
        alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))

        curr_dist = b*math.cos(alpha)

        future_dist = curr_dist - CAR_LENGTH*math.sin(alpha)

        error = future_dist - DESIRED_DISTANCE_LEFT
        return error, curr_dist

		
    def followRight(self, data):
        a = self.getRange(data,theta)
        b = self.getRange(data,0)
        swing = math.radians(theta)
        alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))

        curr_dist = b*math.cos(alpha)

        future_dist = curr_dist+CAR_LENGTH*math.sin(alpha)

        error = DESIRED_DISTANCE_RIGHT - future_dist
        return error, curr_dist


    def lidar_callback(self, data):
        """ 
        """

        error_right, curr_dist_right = self.followRight(data)
        error_left, curr_dist_left = self.followLeft(data)
	
        if curr_dist_right >= curr_dist_left:
            error = error_left
        else:
	    # error = followRight(data,final_desired_trajectory)
            error = error_right
	    #print 'Following Right'
	    #print 'Error', error

        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)