#!/usr/bin/env python

'''
main.py
Defines node for ROS
'''

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
        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.bot_timestep), self.periodic_task)

        rospy.loginfo('[%s] Initialzed.' %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo('[%s] %s = %s ' %(self.node_name,param_name,value))
        return value

    def periodic_task(self, event):
        self.ducky_bot.state_machine()

    def on_shutdown(self):
        rospy.loginfo('[%s] Shutting down.' %(self.node_name))


def main():
    graph = ducky_graph.DuckyGraph()
    bot_io = ducky_io.ROSIO()
    start_node_id = 'A'
    start_orientation = 0
    bot = ducky_bot.DuckyBot(graph, bot_io, start_node_id, start_orientation)

    # Create State
    bot = ducky_bot.DuckyBot(graph, ducky_io.ROSIO(), start_node_id, start_orientation)
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