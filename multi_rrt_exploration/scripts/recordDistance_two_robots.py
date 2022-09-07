#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Point
from math import sqrt
from copy import copy

class RecordDistance():
    
    def __init__(self):

        # init and name the node
        rospy.init_node('recordDistanceNode', anonymous=False)
        
        self.listener = tf.TransformListener()
       
        rateHz = 1

        rate = rospy.Rate(rateHz)
        
        robots = ['robot_1', 'robot_2']

        distances = [0, 0]
        last  = []
        p = Point()
        p.x = 0
        p.y = 0
        for i in range(len(distances)):
            last.append(copy(p))

        for i in range(len(robots)):
            temp = 1
            while temp != 0:
                try:
                    self.listener.waitForTransform('/'+ robots[i]+'/odom','/'+ robots[i]+'/base_link', rospy.Time(), rospy.Duration(5))
                    temp = 0
                except(tf.Exception):
                    rospy.logerr("can't get the odom frame of robots" + str(i))
                    
                
                # rospy.signal_shutdown('tf Exception')

        while not rospy.is_shutdown():
            dist = 0
            for i in range(len(robots)):
                position = self.get_odom(robots[i])
                distances[i] += sqrt((position.x-last[i].x)**2 + (position.y-last[i].y)**2)
                dist += distances[i]
                last[i].x = position.x
                last[i].y = position.y

                rospy.logwarn('robots'+str(i)+' distance: ' + str(distances[i]))
            rospy.logwarn('Total distance: ' + str(dist))
            rate.sleep()
            # rospy.sleep(5)
        
    def get_odom(self, robot):
        # get the current transform between the odom and base frame
        try:
            (trans, rot) = self.listener.lookupTransform('/'+ robot+'/odom', '/'+robot+'/base_link', rospy.Time(0))
        except (tf.Exception):
            rospy.ERROR('tf Exception')
            return 
        return Point(*trans)

if __name__ == '__main__':
    try:
        RecordDistance()
    except (Exception):
        rospy.loginfo("Terminated.")    
