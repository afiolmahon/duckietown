import rospy
from std_msgs.msg import String
from queue import DestinationQueue
import graph

NODE_NAME='navigator'
NO_DESTINATION = -1

graph = graph.DuckyGraph()

class DuckyState:
	def __init__(self, start_loc, start_orientation):
		self.queue = DestinationQueue()
		self.location = start_loc
		self.orientation = start_orientation

	def drive(self, turn):
		#TODO implement driving code
		if command == graph.CMD_LEFT:
			pass
		elif command == graph.CMD_RIGHT:
			pass
		elif command == graph.CMD_STRAIGHT:
			pass
		elif command == graph.CMD_BACK:
			pass
		elif command == graph.CMD_PASS:
			pass
		elif command == graph.CMD_NOPATH:
			print("ERROR: No path to objective")

	def execute(self):
		next_destination = self.queue.pop_next_destination() # get next destination
		path = graph.get_path(self.location, next_destination)
		for node in path:
			command = node.get_direction_to_next(self.orientation): # switch from next node to current node
			self.drive(command)
			self.orientation = graph.get_node().get_orientation_from_origin(node)
			# TODO update orientation when arriving at next node
		# get next destination
		# get path to next destination
		# traverse nodes to destination

		pass


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
