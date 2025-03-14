import numpy as np
import random
import matplotlib.pyplot as plt

class Node:
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None
        self.cost = 0

def rrt_star(start, goal, obstacles, dim_x=10, dim_y=10, max_iter=1000, step_size=0.1, radius=2, tolerance=0.2):
    tree = [Node(start)]
    goal_reached_nodes = []  # Keep track of nodes that reach the goal
    path = None
    
    for _ in range(max_iter):
        rand_point = sample_space(dim_x,dim_y)
        nearest_node = find_nearest(tree, rand_point)
        new_node = steer(nearest_node, rand_point, step_size)
        
        if not collides(new_node.point, obstacles):  # Pass point instead of Node object
            nearby_nodes = find_nearby(tree, new_node, radius)
            new_node = choose_parent(new_node, nearby_nodes)
            tree.append(new_node)
            rewire(tree, new_node, nearby_nodes, radius)
            
            if np.linalg.norm(new_node.point - goal) <= tolerance:  # Adjusted condition
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

def collides(point, obstacles):  # Only point is passed
    for obstacle in obstacles:
        if np.linalg.norm(point - obstacle['point']) < obstacle['radius'] + 0.5:
            return True
    return False

def find_nearby(tree, point, radius):
    tree_points = np.array([node.point for node in tree])
    distances = np.linalg.norm(tree_points - point.point, axis=1)  # Vectorized distance computation
    nearby_nodes = [node for node, dist in zip(tree, distances) if dist < radius]
    return nearby_nodes

def choose_parent(point, nearby_nodes):
    min_cost = float('inf')
    parent = None
    for node in nearby_nodes:
        cost = node.cost + np.linalg.norm(node.point - point.point)
        if cost < min_cost:
            min_cost = cost
            parent = node
    point.parent = parent
    point.cost = min_cost
    return point

def rewire(tree, new_node, nearby_nodes, radius):
    for node in nearby_nodes:
        if node.cost > new_node.cost + np.linalg.norm(node.point - new_node.point):
            node.parent = new_node
            node.cost = new_node.cost + np.linalg.norm(node.point - new_node.point)

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
    ax.set_title('RRT*')
    ax.legend(loc='lower right')
    ax.set_aspect('equal', 'box')
    plt.show()


# Example usage
start = np.array([1, 1])
goal = np.array([10, 10])
dim_x = 11
dim_y = 11
obstacles = [{'point': np.array([5, 5]), 'radius': 1}, {'point': np.array([7, 3]), 'radius': 0.5}, {'point': np.array([2, 8]), 'radius': 0.6}]
tree, path = rrt_star(start, goal, obstacles, dim_x=dim_x, dim_y=dim_y, step_size=0.3, radius=1.5)
plot_tree(tree, start, goal, obstacles, path)
