import rospy
from std_msgs.msg import String

NODE_NAME='navigator'

def main():
	pub = rospy.Publisher('destination', String, queue_size=10)
	rospy.init_node(NODE_NAME)
	rate = rospy.Rate(10) # hz
	while not rospy.is_shutdown():
		test_str = 'Test Message {}'.format(rospy.get_time())
		rospy.loginfo(test_str)
		pub.publish(test_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
