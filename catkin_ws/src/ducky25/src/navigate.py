"""
navigate.py

"""

from queue import DestinationQueue
import graph
import rosBind

NODE_NAME = 'navigator'
NO_DESTINATION = -1


class DuckyState:
	def __init__(self, ducky_graph, start_node_id, start_orientation):
		self.queue = DestinationQueue()
		self.current_node_id = start_node_id
		self.orientation = start_orientation
		self.duckyBot = rosBind.ROSInterface()
		self.duckyGraph = ducky_graph

	def drive(self, command, artag=-1):
		""" Drive to next node, will drive to intersection if artag=-1 otherwise will drive to specified tag"""
		if command == graph.CMD_LEFT:
			self.duckyBot.intersectionLeft()
		elif command == graph.CMD_RIGHT:
			self.duckyBot.intersectionRight()
		elif command == graph.CMD_STRAIGHT:
			self.duckyBot.intersectionStraight()
		elif command == graph.CMD_BACK:
			self.duckyBot.intersectionBack()
		elif command == graph.CMD_PASS:
			return # do nothing if already at target intersection
		
		if artag == -1:
			self.duckyBot.nextIntersection()
		else:
			self.duckyBot.driveToTag(artag)
		

	def drive_to_target(self):
		# Get next destination and create a path to it
		destination_node_id = self.queue.pop_next_destination()
		self.duckyBot.log('[drive_to_target()] Next destination: {}, Current location: {}'.format(destination_node_id, self.current_node_id))
		path = self.duckyGraph.get_path(self.current_node_id, destination_node_id)
		self.duckyBot.log('[drive_to_target()] Path planner has generated path: {}'.format(path))

		# Follow the path until the destination node is reached
		for i in range(len(path)):
			next_node_id = path[i]
			# Get command to get from current node to next node
			command = self.duckyGraph.get_node(self.current_node_id).get_direction_to_next(self.orientation, next_node_id)
			self.drive(command)

			# Update orientation and position
			self.orientation = self.duckyGraph.get_node(next_node_id).get_orientation_from_previous(self.current_node_id)
			self.current_node_id = next_node_id
			self.duckyBot.log('[drive_to_target()]POST MOVE {}: orientation: {}, node_id: {}'.format(i, self.orientation, self.current_node_id))

def main():
	duckyGraph = graph.DuckyGraph()
	ducky = DuckyState(duckyGraph, 0, graph.NORTH)

if __name__ == '__main__':
	main()