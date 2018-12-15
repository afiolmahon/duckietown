#!/usr/bin/env python

'''
main.py
Defines node for ROS
'''
from duckietown_msgs.msg import StopLineReading, BoolStamped

import rospy

import ducky_bot
import ducky_graph
import ducky_io
import time

SIMULATOR = True
TIMESTEP = 1.0
NODE_NAME = 'ducky25'

class DuckyNode(object):
    ''' Node behavior is defined here '''

    def __init__(self, ducky_bot, time_step):
        # Save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo('[%s] Initialzing.' %(self.node_name))
        # Initialize duckybot objects machine
        self.ducky_bot = ducky_bot
        # Read parameters
        self.bot_timestep = self.setupParameter('~bot_timestep', time_step)
        
        # create publisher to enable/disable lane control
        self.pub_lane_control = rospy.Publisher("/ducky25/lane_controller_node/enabled", BoolStamped, queue_size=1)

        # create subscriber to read stopline data from filter node
        self.sub_topic_b = rospy.Subscriber("/ducky25/stop_line_filter_node/stop_line_reading", StopLineReading, self.handle_stopline_reading)
        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.bot_timestep), self.periodic_task)
        rospy.loginfo('[%s] Initialzed.' %(self.node_name))

        # State information for writing at beginning of periodic task
        self.is_at_intersection = False
        self.ducky_bot.io.lane_control_func = self.set_lane_control_enable


    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo('[%s] %s = %s ' %(self.node_name,param_name,value))
        return value

    def set_lane_control_enable(self, enabled):
        msg = BoolStamped()
        msg.data = enabled
        self.pub_lane_control.publish(msg)

    def handle_stopline_reading(self, msg):
        # Update DuckyIO with information
        #self.ducky_bot.io.log('stop_line_detected: {}, at_stop_line: {}'.format(msg.stop_line_detected, msg.at_stop_line))
        self.is_at_intersection = msg.at_stop_line

    def periodic_task(self, event):
        self.ducky_bot.io.at_intersection = self.is_at_intersection
        self.ducky_bot.state_machine()

    def on_shutdown(self):
        rospy.loginfo('[%s] Shutting down.' %(self.node_name))


def main():
    # Create State
    robot_io = ducky_io.ROSIO()
    bot = ducky_bot.DuckyBot(ducky_graph.DuckyGraph(), robot_io, 'A', 0)
    # Intialize node
    rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.DEBUG)
    # Initialize representation objects
    ducky = DuckyNode(bot, TIMESTEP)
    # configure shutdown behavior
    rospy.on_shutdown(ducky.on_shutdown)
    # keep node alive
    rospy.spin()

if __name__ == '__main__':
	main()