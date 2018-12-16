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
    def __init__(self, node_id, north_id, east_id, south_id, west_id, north_weight, east_weight, south_weight, west_weight):
        self.id = node_id
        self.adjacent_node_ids = [north_id, east_id, south_id, west_id] # indices correspond to compass directional constants
        self.weights = [north_weight, east_weight, south_weight, west_weight]

    def get_edges(self):
        """Get all edges that connect to this node"""
        edges = []
        for node_id in self.adjacent_node_ids:
            edges.append((self.id, node_id))
        return edges

    def get_orientation_from_previous(self, previous_node_id):
        for i in range(len(self.adjacent_node_ids)):
            if previous_node_id == self.adjacent_node_ids[i]:
                return (i + 2) % 4
        raise Exception('get_orientation_from_previous: Invalid previous_node_id {}'.format(previous_node_id))

    def get_direction_to_next(self, orientation, next_node_id):
        for i in range(len(self.adjacent_node_ids)):
            if next_node_id == self.adjacent_node_ids[i]:
                return get_direction_to_orientation(orientation, i)
        if (next_node_id == self.id):
            return CMD_PASS
        else:
            raise Exception('[get_direction_to_next] No Path found, node {} does not connect to {}'.format(self.id, next_node_id))

class DuckyGraph:
    def __init__(self):
        self.nodes = dict()
        self.add_node(DuckyNode('A', None, 'D', 'C', 'B', None, 8, 1, 4))
        self.add_node(DuckyNode('B', 'A', 'C', 'E', None, 4, 1, 4, None))
        self.add_node(DuckyNode('C', 'A', 'D', 'E', 'B', 1, 2, 1, 2))
        self.add_node(DuckyNode('D', 'C', 'A', 'E', None, 2, 8, 2, 0))
        self.add_node(DuckyNode('E', 'C', 'D', None, 'B', 1, 2, None, 4))

    def add_node(self, node):
        self.nodes[node.id] = node

    def get_node(self, node_id):
        return self.nodes[node_id]

    def get_edges(self):
        edges = []
        for i in self.nodes.keys():
            for j in self.nodes[i].get_edges():
                edges.append(j)
        return edges

    def get_path_weight(self, n1, n2):
        # TODO implement
        """ use dijkstras algorithm to find path between 2 nodes"""
        return 7

    def get_path(self, n1, n2):
        # TODO implement
        """ use pathfinding algorithm to return a series of nodes from start to end path"""
        return ['A', 'D', 'C']

