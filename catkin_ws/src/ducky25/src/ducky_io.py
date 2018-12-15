'''
ducky_io.py

define input and output specific to ros so logic can be run/tested without ros installed
'''

# Direction definitons
DIR_SRAIGHT = 0
DIR_RIGHT = 1
DIR_BACK = 2
DIR_LEFT = 3
DIRS = [DIR_SRAIGHT, DIR_RIGHT, DIR_BACK, DIR_LEFT]

class DuckyIO:
    '''Inteface to decouple ROS specific code'''

    def drive_intersection(self, direction, tagid=-1):
        ''' return true when intersection is reached, false otherwise '''
        raise Exception('Method drive_intersection unimplemented')

    def log(self, message):
        raise Exception('Method log unimplemented')

try:
    import rospy
    from duckietown_msgs.msg import  Twist2DStamped, LanePose, BoolStamped

    class ROSIO(DuckyIO):
        def __init__(self):
            self.drive_state = 0
            self.at_intersection = False
            self.pub_lane_control = rospy.Publisher("~enable", BoolStamped, queue_size=1)

        def openLoopTurn(self, direction):
            pass

        def setLaneControl(self, enabled):
            msg = BoolStamped()
            msg.data = enabled
            self.pub_lane_control.publish(msg)

        def drive_intersection(self, direction, tagid=-1):
            if self.drive_state == 0:
                # Open loop turn
                self.openLoopTurn(direction)
                self.drive_state = 1
            elif self.drive_state == 1:
                self.setLaneControl(True)
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
            print(message_string)

except ImportError as e:
    print('error loading rospy')


class TestIO(DuckyIO):
    def drive_intersection(self, direction, tagid=-1):
        print('[ROSInterface]Driving in {} direction, tagid is {}'.format(direction, tagid))
        return True

    def log(self, message):
        print('[DUCKY IO]{}'.format(message))


