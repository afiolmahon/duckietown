'''
rosBind.py

'''

try:

    class ROSInterface:
        '''Inteface for driving commands to encapsulate ROS specific code'''

        def __init__(self):
            pass

        def intersectionLeft(self):
            raise Exception('Method unimplemented')

        def intersectionRight(self):
            raise Exception('Method unimplemented')
        
        def intersectionStraight(self):
            raise Exception('Method unimplemented')
        
        def intersectionBack(self):
            raise Exception('Method unimplemented')

        def nextIntersection(self):
            raise Exception('Method unimplemented')

        def driveToTag(self, tagid):
            raise Exception('Method unimplemented')

        def log(self, message):
            rospy.loginfo('[ROSInterface]{}'.format(message))

except Exception as e:
    print(e)


class ROSInterFake(ROSInterface):
    '''Test logic under ideal physical robot/ROS behavior'''

    def intersectionLeft(self):
        print('Left at Intersection...')

    def intersectionRight(self):
        print('Right at Intersection...')
    
    def intersectionStraight(self):
        print('Straight at Intersection...')
    
    def intersectionBack(self):
        print('U Turn at Intersection...')

    def nextIntersection(self):
        print('Line follow to next intersection...')

    def driveToTag(self, tagid):
        print('Driving to artag #{}'.format(tagid))

    def log(self, message):
        print('[ROSInterface]{}'.format(message))