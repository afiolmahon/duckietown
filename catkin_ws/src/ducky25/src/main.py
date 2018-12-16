#!/usr/bin/env python

'''
main.py
Defines node for ROS
'''
from duckietown_msgs.msg import StopLineReading, BoolStamped, Twist2DStamped
import rospy

import ducky_bot
import ducky_graph
import ducky_io
import time

SIMULATOR = True
TIMESTEP = 0.5
NODE_NAME = 'ducky25'

STOP_LINE_TOPIC = "/ducky25/stop_line_filter_node/stop_line_reading"
LANE_CONTROL_ENABLED_TOPIC = "/ducky25/lane_controller_node/enabled"
TOPIC_CAR_CMD = "/ducky25/lane_controller_node/car_cmd"

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
        self.pub_lane_control = rospy.Publisher(LANE_CONTROL_ENABLED_TOPIC, BoolStamped, queue_size=1)

        self.pub_car_cmd = rospy.Publisher(TOPIC_CAR_CMD, Twist2DStamped, queue_size=1)

        # create subscriber to read stopline data from filter node
        self.sub_topic_b = rospy.Subscriber(STOP_LINE_TOPIC, StopLineReading, self.handle_stop_line_msg)

        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.bot_timestep), self.periodic_task)
        rospy.loginfo('[%s] Initialzed.' %(self.node_name))

        # State information for writing at beginning of periodic task
        self.initial_calibrate = False
        self.ducky_bot.io.lane_control_func = self.set_lane_control_enable

        self.set_lane_control_enable(False)


    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo('[%s] %s = %s ' %(self.node_name,param_name,value))
        return value

    def set_lane_control_enable(self, enabled):
        msg = BoolStamped()
        msg.data = enabled
        self.pub_lane_control.publish(msg)

    def openLoopTurn(self, direction):
        self.ducky_bot.io.log("beginning open loop turn in dir {}".format(direction))
        if direction == -1:
            pass
        elif direction == 0:
            msg = Twist2DStamped()
            msg.v = 1.0
            msg.omega = 0.0
            self.pub_car_cmd.publish(msg)
            time.sleep(1.5)
        elif direction == 1:
            msg = Twist2DStamped()
            msg.v = 1.0
            msg.omega = -10.0
            self.pub_car_cmd.publish(msg)
            time.sleep(1.3)
        elif direction == 2:
            raise Exception('cant go backward!')
        elif direction == 3:
            msg = Twist2DStamped()
            msg.v = 1.0
            msg.omega = 7.0
            self.pub_car_cmd.publish(msg)
            time.sleep(2)

        # stop robot
        msg2 = Twist2DStamped()
        msg.v = 0.0
        msg.omega = 0.0
        self.pub_car_cmd.publish(msg2)
        self.ducky_bot.io.log("turn complete returning")
        return True

    def handle_stop_line_msg(self, msg):
        # Update DuckyIO with information
        #self.ducky_bot.io.log('stop_line_detected: {}, at_stop_line: {}'.format(msg.stop_line_detected, msg.at_stop_line))
        self.ducky_bot.io.at_intersection = msg.at_stop_line
    
    def periodic_task(self, event):
        # get to intersection to begin state machine
        if not self.initial_calibrate:
            time.sleep(5)
            self.openLoopTurn(3)
            # if self.ducky_bot.io.drive_intersection(-1):
            #     self.initial_calibrate = True
            self.initial_calibrate = True
        else:
            # self.ducky_bot.state_machine()
            pass

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