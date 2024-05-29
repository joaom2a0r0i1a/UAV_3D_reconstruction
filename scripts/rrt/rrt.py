import numpy as np
import random
import matplotlib.pyplot as plt

class Node:
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None
        self.cost = 0

def rrt(start, goal, obstacles, dim_x=10, dim_y=10, max_iter=1000, step_size=0.1, radius=0.5, tolerance=0.5):
    tree = [Node(start)]
    goal_reached_nodes = []  # Keep track of nodes that reach the goal
    path = None
    
    for _ in range(max_iter):
        rand_point = sample_space(dim_x,dim_y)
        nearest_node = find_nearest(tree, rand_point)
        new_node = steer(nearest_node, rand_point, step_size)
        
        if not collides(new_node.point, obstacles):
            new_node.parent = nearest_node
            tree.append(new_node)
            
            if np.linalg.norm(new_node.point - goal) < tolerance:
                print("Goal reached!")

                # Backtrack with the node having the smallest cost among those that reached the goal
                goal_reached_nodes.append(new_node)
                min_cost_node = min(goal_reached_nodes, key=lambda node: node.cost)
                path = backtrack(min_cost_node)
    
    return tree, path

def sample_space(dim_x,dim_y):
    rand_x = random.random() * dim_x #* 2 * dim_x - dim_x
    rand_y = random.random() * dim_y #* 2 * dim_y - dim_y
    return np.array([rand_x,rand_y])

def find_nearest(tree, point):
    distances = [np.linalg.norm(node.point - point) for node in tree]
    nearest_index = np.argmin(distances)
    return tree[nearest_index]

def steer(from_node, to_point, step_size):
    direction = (to_point - from_node.point) / np.linalg.norm(to_point - from_node.point)
    new_point = from_node.point + step_size * direction
    return Node(new_point)

def collides(point, obstacles): 
    for obstacle in obstacles:
        if np.linalg.norm(point - obstacle['point']) < obstacle['radius'] + 0.1:
            return True
    return False

def backtrack(node):
    path = []
    while node:
        path.append(node.point)
        node = node.parent
    path.reverse()
    return path

def plot_tree(tree, start, goal, obstacles, path=None):
    fig, ax = plt.subplots()
    # Plot tree edges
    for node in tree:
        if node.parent:
            ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'k-', linewidth=0.5)
    # Plot obstacles
    for obstacle in obstacles:
        circle = plt.Circle(obstacle['point'], obstacle['radius'], color='r')
        ax.add_patch(circle)
    # Plot start and goal points
    ax.scatter(start[0], start[1], color='g', marker='o', label='Start')
    ax.scatter(goal[0], goal[1], color='b', marker='o', label='Goal')
    # Plot final path (if provided)
    if path:
        ax.plot([p[0] for p in path], [p[1] for p in path], 'b-', linewidth=2, label='Path')
    # Labels and formatting
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_xlim(0, 11) 
    ax.set_ylim(0, 11)
    ax.set_title('RRT')
    ax.legend(loc='lower right')
    ax.set_aspect('equal', 'box')
    plt.show()

# Example usage
start = np.array([1, 1])
goal = np.array([10, 10])
dim_x = 11
dim_y = 11
obstacles = [{'point': np.array([5, 5]), 'radius': 1}, {'point': np.array([7, 3]), 'radius': 0.5}, {'point': np.array([2, 8]), 'radius': 0.6}]
tree, path = rrt(start, goal, obstacles, dim_x=dim_x, dim_y=dim_y, step_size=0.5, radius=2)
plot_tree(tree, start, goal, obstacles, path)
