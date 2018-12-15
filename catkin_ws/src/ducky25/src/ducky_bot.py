'''
ducky_bot.py

Coordinates the robot state/
'''

from ducky_queue import DestinationQueue
import ducky_graph

CMD_PASS = 'PASS'


class DuckyBot:
	def __init__(self, ducky_graph, ducky_io, start_node_id, start_orientation):
		self.queue = DestinationQueue()
		self.queue.add_to_queue('C')
		self.current_node_id = start_node_id
		self.orientation = start_orientation
		self.io = ducky_io
		self.duckyGraph = ducky_graph

		# State machine variables
		self.state = 0
		self.path_position = 0
		self.path_position_stop = 0
		self.path = None
		self.command = CMD_PASS
		self.next_node_id = None

	def drive(self, command, artag=-1):
		''' Drive to next node, will drive to intersection if artag=-1 otherwise will drive to specified tag '''
		return True if command == CMD_PASS else self.io.drive_intersection(command, artag)	

	def state_machine(self):
		if self.state == 0:
			''' calculate path and drive '''
			# Get next destination and create a path to it
			destination_node_id = self.queue.pop_next_destination()
			if destination_node_id == -1:
				self.io.log('[state_machine()] Waiting for destination from queue...')
			else:	
				self.io.log('[state_machine()] Next destination: {}, Current location: {}'.format(destination_node_id, self.current_node_id))
				self.path = self.duckyGraph.get_path(self.current_node_id, destination_node_id)
				self.io.log('[state_machine()] Path planner has generated path: {}'.format(self.path))

				self.path_position_stop = len(self.path)
				self.path_position = 0
				self.io.log('[state_machine()] Transitioning to move state')
				# State transition
				self.state = 1
		elif self.state == 1:
			''' calculate position and wait for next state '''
			if self.path_position < self.path_position_stop:
				# Drive to next node along path
				self.next_node_id = self.path[self.path_position]
				# Get command to get from current node to next node
				self.command = self.duckyGraph.get_node(self.current_node_id).get_direction_to_next(self.orientation, self.next_node_id)
				self.io.log('[state_machine()]PRE MOVE {}: orientation: {}, node_id: {}, next_node_id: {}'.format(self.path_position, self.orientation, self.current_node_id, self.next_node_id))
				self.state = 2
			else:
				# finished navigating path, rest state to queue
				self.state = 0
		elif self.state == 2:
			''' waiting for drive to complete '''
			if self.drive(self.command): # when we have reached the destination
				# Update orientation and position
				if self.next_node_id != self.current_node_id:
					self.orientation = self.duckyGraph.get_node(self.next_node_id).get_orientation_from_previous(self.current_node_id)
					self.current_node_id = self.next_node_id
				self.io.log('[state_machine()]POST MOVE {}: orientation: {}, node_id: {}'.format(self.path_position, self.orientation, self.current_node_id))

				self.path_position = self.path_position + 1
				self.state = 1
			
			