import math
import asyncio
import os
import getpass
import json
import websockets

class Map:
    def __init__(self, grid: str):
        self.grid = grid
        self.grid_size = int(math.sqrt(len(self.grid)))

    def __repr__(self):
        return self.grid
    
    def move(self, vehicle: str, vehicle_type: str, direction: str, index: int):
        """
        Move vehicle in given direction
        If the vehicle can be moved, return the new map
        If the vehicle can't be moved, return None
        """
        # Horizontal car
        if vehicle_type == "HC":
            if direction == 'd':
                # Check if the vehicle can be moved to the right
                if ((index + 1) % self.grid_size == (self.grid_size - 1)) or (self.grid[index + 2] != 'o'):
                    return None
                # Move the vehicle to the right
                return self.grid[:index] + 'o' + vehicle * 2 + self.grid[index + 3:]
            elif direction == 'a':
                # Check if the vehicle can be moved to the left
                if (index % self.grid_size == 0) or (self.grid[index - 1] != 'o'):
                    return None
                # Move the vehicle to the left
                return self.grid[:index - 1] + vehicle * 2 + 'o' + self.grid[index + 2:]
        # Horizontal truck
        elif vehicle_type == "HT":
            if direction == 'd':
                # Check if the vehicle can be moved to the right
                if ((index + 2) % self.grid_size == (self.grid_size - 1)) or (self.grid[index + 3] != 'o'):
                    return None
                # Move the vehicle to the right
                return self.grid[:index] + 'o' + vehicle * 3 + self.grid[index + 4:]
            elif direction == 'a':
                # Check if the vehicle can be moved to the left
                if (index % self.grid_size == 0) or (self.grid[index - 1] != 'o'):
                    return None
                # Move the vehicle to the left
                return self.grid[:index - 1] + vehicle * 3 + 'o' + self.grid[index + 3:]
        # Vertical car
        elif vehicle_type == "VC":
            if direction == 's':
                # Check if the vehicle can be moved to the bottom
                if (((index + self.grid_size) // self.grid_size) == (self.grid_size - 1)) or (self.grid[index + self.grid_size * 2] != 'o'):
                    return None
                # Move the vehicle to the bottom
                return self.grid[:index] + 'o' + self.grid[index + 1:index + self.grid_size*2] + vehicle + self.grid[index + self.grid_size * 2 + 1:]
            elif direction == 'w':
                # Check if the vehicle can be moved to the top
                if ((index // self.grid_size) == 0) or (self.grid[index -  self.grid_size] != 'o'):
                    return None
                # Move the vehicle to the top
                return self.grid[:index - self.grid_size] + vehicle + self.grid[index - self.grid_size + 1 :index + self.grid_size] + 'o' + self.grid[index + self.grid_size + 1:]
        # Vertical truck
        elif vehicle_type == "VT":
            if direction == 's':
                # Check if the vehicle can be moved to the bottom
                if (((index + self.grid_size * 2) // self.grid_size) == (self.grid_size - 1)) or (self.grid[index + self.grid_size * 3] != 'o'):
                    return None
                # Move the vehicle to the bottom
                return self.grid[:index] + 'o' + self.grid[index + 1:index + self.grid_size*3] + vehicle + self.grid[index + self.grid_size * 3 + 1:]
            elif direction == 'w':
                # Check if the vehicle can be moved to the top
                if ((index // self.grid_size) == 0) or (self.grid[index -  self.grid_size] != 'o'):
                    return None
                # Move the vehicle to the top
                return self.grid[:index - self.grid_size] + vehicle + self.grid[index - self.grid_size + 1 :index + self.grid_size * 2] + 'o' + self.grid[index + self.grid_size * 2 + 1:]
        return None

    def test_win(self):
        """Test if red car has crossed the left most column"""
        if self.grid_size == 6 and len(self.grid[self.grid.index('A') + 2 : 18]) == 0:
            return True
        elif self.grid_size == 8 and len(self.grid[self.grid.index('A') + 2 : 40]) == 0:
            return True
        elif self.grid_size == 4 and len(self.grid[self.grid.index('A') + 2 : 12]) == 0:
            return True
        return False

async def reconstruct_path(came_from: dict, current: str):
    """Reconstruct the path"""
    # Path
    path = [came_from[current]['move']]
    # Loop
    while current in came_from:
        path.append(came_from[current]['move'])
        current = came_from[current]['node']
    # Return the path
    return path

async def astar(grid: str, heuristic):
    """A* Pathfinder algorithm"""
    # The set of discovered nodes that may need to be (re-)expanded
    # Initially, only the start node is known
    # This is usually implemented as a min-heap or priority queue rather than a hash-set
    open_set = {grid}
    # For node n, came_from[n] is the node immediately preceding it on the cheapest path from start
    came_from = {}
    # For node n, gscore[n] is the cost of the cheapest path from start to n currently known
    gscore = {grid: 0}
    # For node n, fscore[n] := gscore[n] + h(n). fscore[n] represents our current best guess as to
    # how cheap a path could be from start to finish if it goes through n
    fscore = {grid: await heuristic(grid)}
    # Loop
    while open_set:
        # This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
        current = min(open_set, key=lambda x: fscore[x])
        # Map
        map = Map(current)
        # Check if the current state is the goal
        if map.test_win():
            return await reconstruct_path(came_from, current)
        # Remove the current state from the open
        open_set.remove(current)
        # Generate new states
        neighbors = await generate_nodes(map, current, came_from)
        # If there are no neighbors, continue
        if len(neighbors) == 0:
            continue
        # Loop
        for neighbor in neighbors:
            # tentative_gscore is the distance from start to the neighbor through current
            tentative_gscore = gscore[current] + 1
            if neighbor not in gscore or tentative_gscore < gscore[neighbor]:
                # If this path to neighbor is better than any previous one, record it
                came_from[neighbor] = {'node': current, 'move': neighbors[neighbor]}
                gscore[neighbor] = tentative_gscore
                fscore[neighbor] = tentative_gscore + await heuristic(neighbor)
                if neighbor not in open_set:
                    open_set.add(neighbor)
    # Open set is empty but goal was never reached
    return None

async def generate_nodes(map: Map, grid: str, came_from: dict):
    """Generate nodes"""
    # Vehicles
    vehicles = set(grid)
    vehicles.discard('o')
    vehicles.discard('x')
    # Nodes
    nodes = {}
    # Loop
    for vehicle in vehicles:
        # Get the index of the vehicle
        index = grid.index(vehicle)
        # Vehicle type
        vt = await vehicle_type(map, grid, vehicle, index)
        # Try to move the vehicle
        if vt == "HC" or vt == "HT":
            # Move the vehicle to the right
            right = map.move(vehicle, vt, 'd', index)
            # Check if the move is valid
            if right is not None:
                # Check if the state is not in the came_from
                if right not in came_from:
                    nodes[right] = vehicle + 'd'
                    # Since the horizontal cars do not affect the fscore of each node and 
                    # the algorithm always chooses the first node with the lowest fscore, 
                    # we can go to the next vehicle
                    continue
            # Move the vehicle to the left
            left = map.move(vehicle, vt, 'a', index)
            # Check if the move is valid
            if left is not None:
                # Check if the state is not in the came_from
                if left not in came_from:
                    nodes[left] = vehicle + 'a'
        else:
            # Move the vehicle to the top
            top = map.move(vehicle, vt, 'w', index)
            # Check if the move is valid
            if top is not None:
                # Check if the state is not in the came_from
                if top not in came_from:
                    nodes[top] = vehicle + 'w'
            # Move the vehicle to the bottom
            bottom = map.move(vehicle, vt, 's', index)
            # Check if the move is valid
            if bottom is not None:
                # Check if the state is not in the came_from
                if bottom not in came_from:
                    nodes[bottom] = vehicle + 's'
    # Return the nodes
    return nodes

async def vehicle_type(map: Map, grid: str, vehicle: str, index: int):
    """
    Return the vehicle type
    Type of vehicles:
    HC = Horizontal Car
    HT = Horizontal Truck
    VC = Vertical Car
    VT = Vertical Truck
    """
    # Horizontal vehicle
    if grid[index + 1] == vehicle:
        if index + 2 >= len(grid) or grid[index + 2] != vehicle:
            return "HC"
        return "HT"
    # Vertical vehicle
    elif index + map.grid_size * 2 >= len(grid) or grid[index + map.grid_size * 2] != vehicle:
        return "VC"
    return "VT"

async def heuristic(grid: str):
    """Heuristic based on the number of blocking vehicles plus the distance to the goal"""
    # Grid size
    grid_size = len(grid)
    # Goal row
    if grid_size == 36:
        goal_row = grid[grid.index('A') + 2 : 18]
    elif grid_size == 64:
        goal_row = grid[grid.index('A') + 2 : 40]
    else:
        goal_row = grid[grid.index('A') + 2 : 12]
    # Vehicles
    vehicles = set(goal_row)
    vehicles.discard('o')
    # Heuristic
    if vehicles:
        return len(vehicles) + len(goal_row)
    return len(goal_row)

async def can_move_vehicle(map: Map, vehicle: str, vehicle_type: str, direction: str, index: int):
    """Checks if the vehicle can be moved in the direction of the intention"""
    if map.move(vehicle, vehicle_type, direction, index) is not None:
        return True
    return False

async def go_to_vehicle(cursor: list, vehicle_coords: tuple, websocket: websockets):
    """Moves the cursor to the vehicle"""
    # Move the cursor to the right
    if cursor[0] < vehicle_coords[0]:
        await websocket.send(json.dumps({"cmd": "key", "key": 'd'}))
    # Move the cursor to the left
    elif cursor[0] > vehicle_coords[0]:
        await websocket.send(json.dumps({"cmd": "key", "key": 'a'}))
    # Move the cursor down
    elif cursor[1] < vehicle_coords[1]:
        await websocket.send(json.dumps({"cmd": "key", "key": 's'}))
    # Move the cursor up
    elif cursor[1] > vehicle_coords[1]:
        await websocket.send(json.dumps({"cmd": "key", "key": 'w'}))
    # Select the vehicle
    elif cursor[0] == vehicle_coords[0] and cursor[1] == vehicle_coords[1]:
        await websocket.send(json.dumps({"cmd": "key", "key": ' '}))
        return
    # New state
    new_state = json.loads(await websocket.recv())
    # Recursive call
    await go_to_vehicle(new_state["cursor"], vehicle_coords, websocket)

async def agent_loop(server_address="localhost:8000", agent_name="student"):
    """Agent loop"""
    async with websockets.connect(f"ws://{server_address}/player") as websocket:
        # Initialize variables
        level = 0
        path = []
        # Receive information about static game properties
        await websocket.send(json.dumps({"cmd": "join", "name": agent_name}))
        # Loop
        while True:
            try:
                # Receive game update, this must be called timely or the game will get out of sync with the server
                state = json.loads(await websocket.recv())
                # Grid
                grid = state["grid"].split(' ')[1]
                # Map
                map = Map(grid)
                # Check if we are in a new level
                if state["level"] != level:
                    level = state["level"]
                    path = await astar(grid, heuristic)
                    if path is None:
                        continue
                # Check if the path is None
                if path is None or path == []:
                    path = await astar(grid, heuristic)
                    if path is None:
                        continue
                # New state
                movement = path.pop()
                vehicle = movement[0]
                direction = movement[1]
                # Check if the vehicle to move is selected, if not go to vehicle and select it
                if state["selected"] == vehicle:
                    # Get the index of the vehicle
                    index = grid.index(vehicle)
                    # Check if vehicle can be moved
                    if await can_move_vehicle(map, vehicle, await vehicle_type(map, grid, vehicle, index), direction, index):    
                        await websocket.send(json.dumps({"cmd": "key", "key": direction}))
                    # If not that means the crazy vehicle is blocking the path
                    else:
                        await websocket.send(json.dumps({"cmd": "key", "key": ' '}))
                        path = await astar(grid, heuristic)
                        if path is None:
                            continue
                # If the vehicle is not selected, go to vehicle and select it
                elif state["selected"] == '':
                    # Get cursor
                    cursor = state["cursor"]
                    # Get the index of the vehicle
                    index = grid.index(vehicle)
                    # Get the vehicle type
                    vt = await vehicle_type(map, grid, vehicle, index)
                    # Get the coordinates of the vehicle
                    if vt == "HC":
                        vehicle_coords = [(index % map.grid_size, index // map.grid_size), ((index + 1) % map.grid_size, index // map.grid_size)]
                    elif vt == "HT":
                        vehicle_coords = [(index % map.grid_size, index // map.grid_size), ((index + 1) % map.grid_size, index // map.grid_size), ((index + 2) % map.grid_size, index // map.grid_size)]
                    elif vt == "VC":
                        vehicle_coords = [(index % map.grid_size, index // map.grid_size), (index % map.grid_size, index // map.grid_size + 1)]
                    elif vt == "VT":
                        vehicle_coords = [(index % map.grid_size, index // map.grid_size), (index % map.grid_size, index // map.grid_size + 1), (index % map.grid_size, index // map.grid_size + 2)]
                    # Get the closest vehicle coordinates                    
                    distances = []
                    for vehicle_coord in vehicle_coords:
                        distances.append(abs(cursor[0] - vehicle_coord[0]) + abs(cursor[1] - vehicle_coord[1]))
                    vehicle_coords = vehicle_coords[distances.index(min(distances))]
                    # Go to vehicle and select it
                    await go_to_vehicle(cursor, vehicle_coords, websocket)
                    # State was popped from the list but the action wasn't executed
                    # Need to append state because we need to move the vehicle
                    path.append(movement)
                # If the vehicle is not selected and there is another vehicle selected, deselect it
                elif state["selected"] != '':
                    await websocket.send(json.dumps({"cmd": "key", "key": ' '}))
                    path.append(movement)
            except Exception:	
                return

# The default values can be changed using the command line, example:
# $ NAME='agent' python3 student.py
loop = asyncio.get_event_loop()
SERVER = os.environ.get("SERVER", "localhost")
PORT = os.environ.get("PORT", "8000")
NAME = os.environ.get("NAME", getpass.getuser())
loop.run_until_complete(agent_loop(f"{SERVER}:{PORT}", NAME))
