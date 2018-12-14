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
        raise Exception('Method drive_intersection unimplemented')

    def log(self, message):
        raise Exception('Method log unimplemented')

try:
    import rospy
    
    class ROSIO(DuckyIO):
        def drive_intersection(self, direction, tagid=-1):
            raise Exception('Method drive_intersection unimplemented')

        def log(self, message):
            message_string = '[ROSInterface]{}'.format(message)
            rospy.loginfo(message_string)
            print(message_string)

except ImportError as e:
    raise Exception('Cannot Import rospy')


class TestIO(DuckyIO):
    def drive_intersection(self, direction, tagid=-1):
        print('[ROSInterface]Driving in {} direction, tagid is {}'.format(direction, tagid))

    def log(self, message):
        print('[ROSInterface]{}'.format(message))


