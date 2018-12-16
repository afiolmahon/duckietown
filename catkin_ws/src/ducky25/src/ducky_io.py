'''
ducky_io.py

define input and output specific to ros so logic can be run/tested without ros installed
'''
import time

# Direction definitons
DIR_SRAIGHT = 0
DIR_RIGHT = 1
DIR_BACK = 2
DIR_LEFT = 3
DIRS = [DIR_SRAIGHT, DIR_RIGHT, DIR_BACK, DIR_LEFT]

TOPIC_CAR_CMD = "/ducky25/lane_controller_node/car_cmd"

class DuckyIO:
    '''Inteface to decouple ROS specific code'''

    def drive_intersection(self, direction):
        ''' return true when intersection is reached, false otherwise '''
        raise Exception('Method drive_intersection unimplemented')

    def log(self, message):
        raise Exception('Method log unimplemented')

try:
    import rospy
    from duckietown_msgs.msg import Twist2DStamped


    class ROSIO(DuckyIO):
        def __init__(self):
            self.drive_state = 2
            self.at_intersection = False
            self.lane_control_func = lambda x: self.log('Fake enable set to {}'.format(x))
            self.pub_car_cmd = rospy.Publisher(TOPIC_CAR_CMD, Twist2DStamped, queue_size=1)

        def openLoopTurn(self, direction):
            if direction == -1:
                pass
            elif direction == 0:
                msg = Twist2DStamped()
                msg.v = 0.5
                msg.omega = 0.0
                self.pub_car_cmd.publish(msg)
                time.sleep(2)
            elif direction == 1:
                msg = Twist2DStamped()
                msg.v = 0.5
                msg.omega = -45.0
                self.pub_car_cmd.publish(msg)
                time.sleep(2)
            elif direction == 2:
                raise Exception('cant go backward!')
            elif direction == 3:
                msg = Twist2DStamped()
                msg.v = 0.5
                msg.omega = 45.0
                self.pub_car_cmd.publish(msg)
                time.sleep(2)
                
            # stop robot
            msg2 = Twist2DStamped()
            msg.v = 0.0
            msg.omega = 0.0
            self.pub_car_cmd.publish(msg2)
            return True
        
        def setLaneControl(self, enabled):
            self.log('Setting lane control: {}'.format(enabled))
            self.lane_control_func(enabled)

        def drive_intersection(self, direction):
            self.log('drive_intersection called! state: {}'.format(self.drive_state))
            if self.drive_state == 0:
                # wait for turn to finish
                if self.openLoopTurn(direction):
                    self.drive_state = 1
            elif self.drive_state == 1:
                self.setLaneControl(True)
                self.at_intersection = False
                self.drive_state = 2
            elif self.drive_state == 2:
                # drive until we are at the intersection
                if self.at_intersection:
                    self.setLaneControl(False)
                    self.drive_state = 0
                    return True
            return False

        def log(self, message):
            message_string = '[ROSInterface]{}'.format(message)
            rospy.loginfo(message_string)

except ImportError as e:
    raise Exception('Cant import ROS!')


class TestIO(DuckyIO):
    def drive_intersection(self, direction):
        print('[ROSInterface]Driving in {} direction'.format(direction))
        return True

    def log(self, message):
        print('[DUCKY IO]{}'.format(message))


