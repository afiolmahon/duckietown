#!/usr/bin/env python

"""
main.py
Defines node for ROS
"""

import rospy
from std_msgs.msg import String
import duckystate
import graph

NODE_NAME = 'ducky25'

class DuckyNode(object):
    """Node behavior is defined here"""
    def __init__(self, startNode=0, startOrientation=0):
        # Save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        #initialize duckybot objects machine
        self.duckyGraph = graph.DuckyGraph()
        self.duckyState = duckystate.DuckyState(self.duckyGraph, startNode, startOrientation)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        
        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.periodicEvent)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def periodicEvent(self, event):
        self.duckyState.state_machine()

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

def main():
    # Intialize node
    rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.DEBUG)
    # Initialize representation objects
    ducky = DuckyNode()
    # configure shutdown behavior
    rospy.on_shutdown(ducky.on_shutdown())
    # keep node alive
    rospy.spin()


if __name__ == '__main__':
	main()