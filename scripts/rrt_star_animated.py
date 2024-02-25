import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class Node:
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None
        self.cost = 0

def rrt_star(start, goal, obstacles, max_iter=1000, step_size=0.1, radius=0.5):
    tree = [Node(start)]
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('RRT* in 3D')
    ax.legend()
    
    def update(frame):
        nonlocal tree
        rand_point = sample_space()
        nearest_node = find_nearest(tree, rand_point)
        new_node = steer(nearest_node, rand_point, step_size)
        
        if not collides(new_node.point, obstacles):  # Pass point instead of Node object
            nearby_nodes = find_nearby(tree, new_node, radius)
            new_node = choose_parent(new_node, nearby_nodes)
            tree.append(new_node)
            rewire(tree, new_node, nearby_nodes, radius)
            ax.clear()
            plot_tree_3d(ax, tree, start, goal, obstacles)
            
            if np.linalg.norm(new_node.point - goal) <= step_size:  # Adjusted condition
                print("Goal reached!")

                # Backtrack with the node having the smallest cost among those that reached the goal
                goal_reached_nodes.append(new_node)
                min_cost_node = min(goal_reached_nodes, key=lambda node: node.cost)
                path = backtrack(min_cost_node)

                plot_tree(ax, tree, start, goal, obstacles, path)  # Plot path
                return tree, path
    
    ani = FuncAnimation(fig, update, frames=1, blit=False)
    plt.show()

def sample_space():
    return np.random.rand(3) * 10  # Random point in a 3D space

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
        if np.linalg.norm(point - obstacle['point']) < obstacle['radius'] + 0.1:
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

def plot_tree_3d(ax, tree, start, goal, obstacles, path=None):
    for node in tree:
        if node.parent:
            ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], [node.point[2], node.parent.point[2]], 'k-', linewidth=0.5)
    for obstacle in obstacles:
        ax.scatter(obstacle['point'][0], obstacle['point'][1], obstacle['point'][2], color='r', marker='o')
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = obstacle['radius'] * np.outer(np.cos(u), np.sin(v)) + obstacle['point'][0]
        y = obstacle['radius'] * np.outer(np.sin(u), np.sin(v)) + obstacle['point'][1]
        z = obstacle['radius'] * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle['point'][2]
        ax.plot_wireframe(x, y, z, color="r", alpha=0.2)
    ax.scatter(start[0], start[1], start[2], color='g', marker='o', label='Start')
    ax.scatter(goal[0], goal[1], goal[2], color='b', marker='o', label='Goal')
    if path:
        path = np.array(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b-', linewidth=2, label='Path')

# Example usage
start = np.array([1, 1, 1])
goal = np.array([6, 4, 2])
obstacles = [{'point': np.array([5, 5, 5]), 'radius': 1}]
rrt_star(start, goal, obstacles, step_size=0.5, radius=2)
