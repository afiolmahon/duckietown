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

class DuckyIO:
    '''Inteface to decouple ROS specific code'''

    def drive_intersection(self, direction, tagid=-1):
        ''' return true when intersection is reached, false otherwise '''
        raise Exception('Method drive_intersection unimplemented')

    def log(self, message):
        raise Exception('Method log unimplemented')

try:
    import rospy

    class ROSIO(DuckyIO):
        def __init__(self):
            self.drive_state = 2
            self.at_intersection = False
            self.lane_control_func = lambda x: self.log('Fake enable set to {}'.format(x))
            self.open_turn_func = lambda v, o: self.log('Fake open turn: v {}  o {}'.format(v, o))

        def openLoopTurn(self, direction):
            if direction == -1:
                pass
            else:
                self.log('turn begin!')
                self.open_turn_func(0.5, 90.0)
                time.sleep(0.5)
                self.open_turn_func(0.0, 0.0)
                self.log('turn end!')



        def setLaneControl(self, enabled):
            self.log('Setting lane control: {}'.format(enabled))
            self.lane_control_func(enabled)

        def drive_intersection(self, direction):
            self.log('drive_intersection called! state: {}'.format(self.drive_state))
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

except ImportError as e:
    raise Exception('Cant import ROS!')
    print('error loading rospy')


class TestIO(DuckyIO):
    def drive_intersection(self, direction, tagid=-1):
        print('[ROSInterface]Driving in {} direction, tagid is {}'.format(direction, tagid))
        return True

    def log(self, message):
        print('[DUCKY IO]{}'.format(message))


