import random
import heapq
import time

counter = 0
OPEN, CLOSED = set(), set()

def generate_grid(size):
    grid = {}
    for i in range(size):
        for j in range(size):
            blocked = random.random() <= 0.3
            grid[(i, j)] = {'visited': False, 'blocked': blocked, 'agent': False, 'target': False, 'path': False}
    return grid

def print_board(grid, size):
    for i in range(size):
        for j in range(size):
            if grid[(i, j)]['agent']:
                print('A', end=' ')
            elif grid[(i, j)]['target']:
                print('T', end =' ')
            elif grid[(i, j)]['path']:
                print('*', end=' ')
            elif grid[(i, j)]['blocked']:
                print('X', end=' ')
            else:
                print('.', end=' ')
        print()

def get_agent(grid, size):
    point_x = []
    point_y = []
    for x in range(size):
        for y in range(size):
            if not grid[(x, y)]['blocked']:
                point_x.append(x)
                point_y.append(y)
    random_agent_choice = random.randint(0, len(point_x) - 1)
    agent_x = point_x[random_agent_choice]
    agent_y = point_y[random_agent_choice]
    grid[(agent_x, agent_y)]['agent'] = True
    agent = (agent_x, agent_y)
    return agent

def get_target(grid, size):
    point_x = []
    point_y = []
    for x in range(size):
        for y in range(size):
            if not grid[(x, y)]['blocked'] and not grid[(x, y)]['agent']:
                point_x.append(x)
                point_y.append(y)
    random_target_choice = random.randint(0, len(point_x) - 1)
    target_x = point_x[random_target_choice]
    target_y = point_y[random_target_choice]
    grid[(target_x, target_y)]['target'] = True
    target = (target_x, target_y)
    return target

def print_path(path, grid):
    for pos in path:
        grid[pos]['path'] = True
        print(pos)

def distance(state, goal):
    return abs(state[0] - goal[0]) + abs(state[1] - goal[1])

def h(state, goal):
    return distance(state, goal)

def c(state, action):
    return 1

def move(state, action, direction):
    x, y = state
    if direction == 'forward':
        if action == 'up':
            return x - 1, y
        elif action == 'down':
            return x + 1, y
        elif action == 'left':
            return x, y - 1
        elif action == 'right':
            return x, y + 1
    elif direction == 'backward':
        if action == 'up':
            return x + 1, y
        elif action == 'down':
            return x - 1, y
        elif action == 'left':
            return x, y + 1
        elif action == 'right':
            return x, y - 1

def A(state):
    return ['up', 'down', 'left', 'right']

def astar(agent, target, grid, direction):
    OPEN = []
    CLOSED = set()
    heapq.heappush(OPEN, (h(agent, target), agent))
    g = {agent: 0}
    tree = {agent: None}

    while OPEN:
        _, current = heapq.heappop(OPEN)
        if current == target:
            path = []
            while current is not None:
                path.append(current)
                current = tree[current]
            return path[::-1]

        CLOSED.add(current)
        for action in A(current):
            next_state = move(current, action, direction)
            if grid.get(next_state) and not grid[next_state]['blocked'] and next_state not in CLOSED:
                tentative_g = g[current] + c(current, action)
                if next_state not in g or tentative_g < g[next_state]:
                    g[next_state] = tentative_g
                    f = tentative_g + h(next_state, target)
                    heapq.heappush(OPEN, (f, next_state))
                    tree[next_state] = current
    return None

def repeated_astar(agent, target, grid, direction, max_iterations):
    for _ in range(max_iterations):
        path = astar(agent, target, grid, direction)
        if path:
            return path
    return None

#Generation
maze_size = 10
grid = generate_grid(maze_size)
agent = get_agent(grid, maze_size)
target = get_target(grid, maze_size)

#Forward A*
print("Repeated Forward A*:")
start_time = time.time()
forward_path = repeated_astar(agent, target, grid, 'forward', 100)
end_time = time.time()
forward_runtime = end_time - start_time

if forward_path:
    print("Shortest Path from agent to target:")
    print_path(forward_path, grid)
else:
    print("No path found.")

#Backward A*
print("\nRepeated Backward A*:")
start_time = time.time()
backward_path = repeated_astar(agent, target, grid, 'backward', 100)
end_time = time.time()
backward_runtime = end_time - start_time

if backward_path:
    print("Shortest Path from agent to target:")
    print_path(backward_path, grid)
else:
    print("No path found.")


print_board(grid, maze_size)

print("\nRepeated Forward A* Runtime:", forward_runtime)
print("Repeated Backward A* Runtime:", backward_runtime)
