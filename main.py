"""

My implementation of the A* Pathfinding algorithm.

"""

import math, random, colorama, time

colorama.init()

class Node:
    def __init__(self, x: int, y: int, type: str) -> None:
        self.x = x
        self.y = y
        self.type = type

    def __str__(self):
        return f"{self.type}Node({self.x}, {self.y})"
    
    def __eq__(self, value: object) -> bool:
        return value.x == self.x and value.y == self.y

    def get_node_as_emoji(self):
        if self.type == "Wall":
            return "⬛️"
        
        if self.type == "Start":
            return "🟩"
        
        if self.type == "Destination":
            return "🟥"

        return "⬜️"

    def is_destination_node(self):
        if self.type == "Destination":
            return True
        return False

    def get_coords(self):
        return (self.x, self.y)

    # gonna use the manhattan distance to calculate the cost of the node.
    # not sure but euclidian distance is basically the pythagorem theorem thing. we can avoid using sqrt because it would be a waste of cpu cycles (real)
    def calculate_cost_heurisitc(self, destinationNode):
        # added a wall type but im too lazy to generate obstacles for a*
        if self.type == "Wall":
            return math.inf

        return abs(self.x - destinationNode.x) + abs(self.y - destinationNode.y)

def info(msg, prefix="A*"):
    print(f"{colorama.Fore.CYAN}[{prefix}]{colorama.Fore.RESET}: {msg}")

def display_map(map: list[Node], size = (5, 5), path = None):
    mapStrings = [[] for y in range(size[1])]
    finalString = ""

    if path is None:
        for y in range(size[1]):
            for x in range(size[0]):
                mapStrings[y].append(map[y][x].get_node_as_emoji())
        
        for column in mapStrings:
            finalString += "".join(column) + "\n"
    else:
        for y in range(size[1]):
            for x in range(size[0]):
                node_data = map[y][x]
                emoji = node_data.get_node_as_emoji()

                if node_data.type != "Start" and node_data.type != "Destination" and path[y][x] == True:
                    emoji = "🟦"

                mapStrings[y].append(emoji)
        
        for column in mapStrings:
            finalString += "".join(column) + "\n"


    print(finalString)

def generate_pathfind_map(size = (5, 5)):
    sizeX = size[0]
    sizeY = size[1]

    path_to_pathfind = [[] for y in range(sizeY)]

    destination_coordinates = (random.randint(1, sizeX-1), random.randint(1, sizeY-1))

    for y in range(0, sizeY):
        for x in range(0, sizeX):
            NodeType = "Default"

            if x == 0 and y == 0:
                NodeType = "Start"
            elif x == destination_coordinates[0] and y == destination_coordinates[1]:
                NodeType = "Destination"
            elif random.randint(1, 5) == random.randint(1, 5):
                NodeType = "Wall"

            path_to_pathfind[y].append(Node(x, y, NodeType))
    
    return path_to_pathfind, destination_coordinates

def main():
    map_size = (10, 5)

    map_data, destination = generate_pathfind_map(map_size)
    display_map(map_data, map_size)

    info(f"Put destination node coords at ({destination[0]}, {destination[1]})", "A* Map Generation")
    info("Starting to pathfind...")
    
    # A* Pathfinding, get nodes that has the lowest cost repededtly

    # visited nodes as we dont want to revisit our old nodes
    # this is a matrix of the size of the map filled with booleans, false for not visited and true for visited
    visited_nodes = [[False for x in range(map_size[0])] for y in range(map_size[1])]
    
    def get_lowest_cost_node_neighbour(node: Node, destinationNode: Node, blockedNodes=None) -> Node:
        def is_pos_out_of_bounds(position):
            x = position[0]
            y = position[1]

            if x < 0 or y < 0:
                return True
            
            if x > (map_size[0] - 1) or y > (map_size[1] - 1):
                return True

            if math.isinf(get_node_cost_from_pos(position)):
                return True

            if blockedNodes and blockedNodes[node[1]][node[0]] == True:
                return True

            return False
        
        def get_node_cost_from_pos(pos):
            try:
                return map_data[pos[1]][pos[0]].calculate_cost_heurisitc(destinationNode)
            except Exception as e:
                info(f"error while accessing node cost at ({pos[0], pos[1]})")
                return math.inf
            

        lowest_cost_data = {
            "Position": (0, 0),
            "Cost": math.inf,
            "Node": Node(0, 0, "_internal_path_node")
        }

        # we should get the neighbouring nodes,
        neighbouring_nodes = [(node.x + 1, node.y), (node.x, node.y + 1), (node.x - 1, node.y), (node.x, node.y - 1)]
        for node in neighbouring_nodes:
            if is_pos_out_of_bounds(node):
                continue
            
            try:
                if visited_nodes[node[1]][node[0]] == True:
                    continue
            except:
                continue

            cost = get_node_cost_from_pos(node)

            if not math.isinf(cost):
                is_destination_node = (destinationNode.x == node[0] and destinationNode.y == node[1])
                if is_destination_node or lowest_cost_data["Cost"] > cost:
                    lowest_cost_data = {
                        "Position": node,
                        "Cost": cost,
                        "Node": map_data[node[1]][node[0]]
                    }
                    
                    if is_destination_node:
                        break
            
            visited_nodes[node[1]][node[0]] = True
        
        return lowest_cost_data["Node"]

    pathfindingStatus = "Success"
    
    path = [[False for x in range(map_size[0])] for y in range(map_size[1])]
    blockedNodes = [[False for x in range(map_size[0])] for y in range(map_size[1])]
    
    lastNode = map_data[0][0]
    destinationNode = map_data[destination[1]][destination[0]]

    while not lastNode.is_destination_node():
        lowestCostNode = get_lowest_cost_node_neighbour(lastNode, destinationNode, blockedNodes)

        if lowestCostNode.type == "_internal_path_node":
            for y in visited_nodes:
                y.clear()
            
            blockedNodes[lowestCostNode.y][lowestCostNode.x] = True
            continue

        lastNode = lowestCostNode
        info(f"Found node {lastNode}")
        path[lastNode.y][lastNode.x] = True
        time.sleep(0.05)
    
    info(f"Finished pathfinding algo with A* algorithm!\n{colorama.Fore.CYAN}[A* Info - Status]{colorama.Fore.RESET} {pathfindingStatus == "Success" and colorama.Fore.GREEN or colorama.Fore.RED}{pathfindingStatus}{colorama.Fore.RESET}\n")

    display_map(map_data, map_size, path)

if __name__ == "__main__":
    main()
