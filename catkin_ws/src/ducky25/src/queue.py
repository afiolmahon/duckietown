import rospy
from std_msgs.msg import String

NODE_NAME = 'destination_queue'


def add_to_queue(message):
	#TODO implement
	pass

def handle_get_next_destination():
	#TODO implement
	# update queue...
	# return string id for next destination
	return 'test_loc'

def get_next_destination_server():
	rospy.init_node(NODE_NAME)
	s = rospy.Service('get_next_destination', get_next_destination, handle_get_next_destination)
	rospy.spin()

if __name__ == '__main__':
	get_next_destination_server()

'''
def main():
	pub = rospy.Publisher('destination', String, queue_size=10)
	rospy.init_node(NODE_NAME)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		test_str = 'Test Message {}'.format(rospy.get_time()) 
		rospy.loginfo(test_str)
		pub.publish(test_str)
		rate.sleep() # Wait for 1/10 of a second
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
'''
