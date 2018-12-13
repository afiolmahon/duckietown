'''
rosdrive.py

'''

class ROSInterface:
    '''Inteface for driving commands to encapsulate ROS specific code'''

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