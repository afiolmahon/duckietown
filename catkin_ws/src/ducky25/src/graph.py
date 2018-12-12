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

# direction constants
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

# Map direction coming into node to a direction leaving the node
ORIENTATION_DIRECTION_MAPPER = [[CMD_STRAIGHT, CMD_RIGHT, CMD_BACK, CMD_LEFT],
                                [CMD_LEFT, CMD_STRAIGHT, CMD_RIGHT, CMD_BACK],
                                [CMD_BACK, CMD_LEFT, CMD_STRAIGHT, CMD_RIGHT],
                                [CMD_RIGHT, CMD_BACK, CMD_LEFT, CMD_STRAIGHT]]

def get_direction_to_orientation(current_dir, next_dir):
    return ORIENTATION_DIRECTION_MAPPER[current_dir][next_dir]
    

class DuckyNode:
    def __init__(self, node_id, north_id, east_id, south_id, west_id):
        self.id = node_id
        self.north = north_id
        self.east = east_id
        self.south = south_id
        self.west = west_id

    def get_edges(self):
        """Get all edges that connect to this node"""
        return [
            (self.id, self.north), (self.north, self.id),
            (self.id, self.east), (self.east, self.id),
            (self.id, self.south), (self.south, self.id),
            (self.id, self.west), (self.west, self.id)
        ]

    def get_direction_to_next(self, orientation, next_node):
        if next_node == self.north:
            return get_direction_to_orientation(orientation, NORTH)
        elif next_node == self.east:
            return get_direction_to_orientation(orientation, EAST)
        elif next_node == self.south:
            return get_direction_to_orientation(orientation, SOUTH)
        elif next_node == self.west:
            return get_direction_to_orientation(orientation, WEST)
        elif next_node == self.id:
            return CMD_PASS
        else:
            return CMD_NOPATH


class DuckyGraph:
    def __init__(self):
        self.nodes = dict()

    def add_node(self, node):
        self.nodes[node.id] = node

    def get_node(self, node_id):
        return self.nodes[node_id]

    def get_path_weight(self, n1, n2):
        """ use dijkstras algorithm to find path between 2 nodes"""
        return 10

    def get_path(self, n1, n2):
        """ use pathfinding algorithm to return a series of nodes from start to end path"""
        return ['p1', 'p2', 'p3']

