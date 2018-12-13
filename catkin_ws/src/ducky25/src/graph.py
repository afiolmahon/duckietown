"""
graph.py
Path finding on duckietown
"""

# Command constants
CMD_LEFT = 'LEFT' # Left turn at intersection
CMD_RIGHT = 'RIGHT' # right turn at intersection
CMD_STRAIGHT = 'STRAIGHT' # go straight at intersection
CMD_BACK = 'BACK' # go backwards at intersection, shouldnt be needed
CMD_PASS = 'PASS' # apready at destination, continue without action
CMD_NOPATH = 'NOPATH' # raise an error

# Compass direction constants
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

# Map direction coming into node to a direction leaving the node
DIRS = [CMD_STRAIGHT, CMD_RIGHT, CMD_BACK, CMD_LEFT]

def get_direction_to_orientation(current_dir, next_dir):
    ''' Calculate direction to leave node in specified NESW based on current orientation (Ex. If facing east and want to go south take a right)'''
    return DIRS[(4-current_dir+next_dir)%4]
    

class DuckyNode:
    def __init__(self, node_id, north_id, east_id, south_id, west_id):
        self.id = node_id
        self.adjacent_node_ids = [north_id, east_id, south_id, west_id] # indices correspond to compass directional constants

    def get_edges(self):
        """Get all edges that connect to this node"""
        edges = []
        for node_id in self.adjacent_node_ids:
            edges.append((self.id, node_id))
            edges.append((node_id, self.id))
        return edges

    def get_orientation_from_previous(self, previous_node_id):
        for i in range(len(self.adjacent_node_ids)):
            if previous_node_id == self.adjacent_node_ids[i]:
                return i
        raise Exception('get_orientation_from_previous: Invalid previous_node_id {}'.format(previous_node_id))

    def get_direction_to_next(self, orientation, next_node_id):
        for i in range(len(self.adjacent_node_ids)):
            if next_node_id == self.adjacent_node_ids[i]:
                return get_direction_to_orientation(orientation, i)
        return CMD_PASS if (next_node_id == self.id) else CMD_NOPATH # Return no path if no adjacent were found or next node is not self

class DuckyGraph:
    def __init__(self):
        self.nodes = dict()

    def add_node(self, node):
        self.nodes[node.id] = node

    def get_node(self, node_id):
        return self.nodes[node_id]

    def get_path_weight(self, n1, n2):
        # TODO implement
        """ use dijkstras algorithm to find path between 2 nodes"""
        return 10

    def get_path(self, n1, n2):
        # TODO implement
        """ use pathfinding algorithm to return a series of nodes from start to end path"""
        return ['p1', 'p2', 'p3']

