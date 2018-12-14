"""
queue.py
location queue implementation for ducky25, decides next location to travel to
"""

class DestinationQueue:
	def __init__(self):
		self.queue = []
		self.position = 0

	def add_to_queue(self, location):
		self.queue.append(location)

	def pop_next_destination(self):
		if len(self.queue) > self.position:
			result = self.queue[self.position]
			self.position = self.position + 1
		else:
			result = '-1'
		return result
