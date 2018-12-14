'''
ducky_bot.py

Coordinates the robot state/
'''

from ducky_queue import DestinationQueue
import ducky_graph


class DuckyBot:
	def __init__(self, ducky_graph, ducky_io, start_node_id, start_orientation):
		self.queue = DestinationQueue()
		self.current_node_id = start_node_id
		self.orientation = start_orientation
		self.io = ducky_io
		self.duckyGraph = ducky_graph

		# State machine variables
		self.path_position = 0
		self.path_position_stop = 0
		self.path = None

	def drive(self, command, artag=-1):
		''' Drive to next node, will drive to intersection if artag=-1 otherwise will drive to specified tag '''
		if command == ducky_graph.CMD_PASS:	
			return
		self.io.drive_intersection(command, artag)	

	def state_machine(self):

		self.io.log('statemachine called')
		# Follow the path until the destination node is reached
		if self.path_position < self.path_position_stop:
			next_node_id = self.path[self.path_position]
			# Get command to get from current node to next node
			command = self.duckyGraph.get_node(self.current_node_id).get_direction_to_next(self.orientation, next_node_id)
			self.drive(command)

			# Update orientation and position
			self.orientation = self.duckyGraph.get_node(next_node_id).get_orientation_from_previous(self.current_node_id)
			self.current_node_id = next_node_id
			self.io.log('[drive_to_target()]POST MOVE {}: orientation: {}, node_id: {}'.format(self.path_position, self.orientation, self.current_node_id))

			self.path_position = self.path_position + 1
		else:
			# Get next destination and create a path to it
			destination_node_id = self.queue.pop_next_destination()
			self.io.log('[state_machine()] Next destination: {}, Current location: {}'.format(destination_node_id, self.current_node_id))
			self.path = self.duckyGraph.get_path(self.current_node_id, destination_node_id)
			self.io.log('[state_machine()] Path planner has generated path: {}'.format(self.path))

			self.path_position_stop = len(self.path) - 1
			self.path_position = 0

			self.io.log('[state_machine()] Resetting to 0 state')

	def onShutdown(self):
		pass
