import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Node:
    def __init__(self, point):
        self.point = np.array(point)  # Including yaw as the fourth dimension
        self.parent = None
        self.cost = 0

def rrt_star(start, goal, obstacles, dim_x=10, dim_y=10, dim_z=10, max_iter=1000, step_size=0.1, radius=2, tolerance=0.3):
    tree = [Node(start)]
    goal_reached_nodes = []  # Keep track of nodes that reach the goal
    path = None
    
    for _ in range(max_iter):
        rand_point = sample_space(dim_x, dim_y, dim_z)
        nearest_node = find_nearest(tree, rand_point)
        new_node = steer(nearest_node, rand_point, step_size)
        
        if not collides(new_node.point[:3], obstacles):  # Pass only x, y, and z components
            nearby_nodes = find_nearby(tree, new_node, radius)
            new_node = choose_parent(new_node, nearby_nodes)
            tree.append(new_node)
            rewire(tree, new_node, nearby_nodes, radius)
            
            if np.linalg.norm(new_node.point[:3] - goal[:3]) <= tolerance:  # Adjusted condition
                print("Goal reached!")

                # Backtrack with the node having the smallest cost among those that reached the goal
                goal_reached_nodes.append(new_node)
                min_cost_node = min(goal_reached_nodes, key=lambda node: node.cost)
                path = backtrack(min_cost_node)
                
                # Calculate yaw angles for each node based on the path
                for i in range(len(path)-1):
                    path[i][3] = calculate_yaw_angle(Node(path[i]), Node(path[i+1]))
                    if i == len(path)-1:
                        path[i+1][3] = goal[3:]  # Preserve yaw angle of the goal
    
    return tree, path


def sample_space(dim_x, dim_y, dim_z):
    rand_x = random.random() * dim_x #* 2 * dim_x - dim_x
    rand_y = random.random() * dim_y #* 2 * dim_y - dim_y
    rand_z = random.random() * dim_z #* 2 * dim_z - dim_z
    return np.array([rand_x, rand_y, rand_z])

def find_nearest(tree, point):
    distances = [np.linalg.norm(node.point[:3] - point[:3]) for node in tree]  # Consider only x, y, and z components
    nearest_index = np.argmin(distances)
    return tree[nearest_index]

def steer(from_node, to_point, step_size):
    direction = (to_point - from_node.point[:3]) / np.linalg.norm(to_point - from_node.point[:3])
    new_point = from_node.point[:3] + step_size * direction
    return Node(np.concatenate([new_point, from_node.point[3:]]))  # Preserve yaw angle

def collides(point, obstacles):
    for obstacle in obstacles:
        if np.linalg.norm(point[:3] - obstacle['point']) < obstacle['radius'] + 0.1:
            return True
    return False

def find_nearby(tree, point, radius):
    tree_points = np.array([node.point for node in tree])
    distances = np.linalg.norm(tree_points[:, :3] - point.point[:3], axis=1)  # Consider only x, y, and z components
    nearby_nodes = [node for node, dist in zip(tree, distances) if dist < radius]
    return nearby_nodes

def choose_parent(point, nearby_nodes):
    min_cost = float('inf')
    parent = None
    for node in nearby_nodes:
        cost = node.cost + np.linalg.norm(node.point[:3] - point.point[:3])
        if cost < min_cost:
            min_cost = cost
            parent = node
    point.parent = parent
    point.cost = min_cost
    return point

def rewire(tree, new_node, nearby_nodes, radius):
    for node in nearby_nodes:
        if node.cost > new_node.cost + np.linalg.norm(node.point[:3] - new_node.point[:3]):
            node.parent = new_node
            node.cost = new_node.cost + np.linalg.norm(node.point[:3] - new_node.point[:3])

def backtrack(node):
    path = []
    while node:
        path.append(node.point)
        node = node.parent
    path.reverse()
    return path

def calculate_yaw_angle(node1, node2):
    dx = node2.point[0] - node1.point[0]
    dy = node2.point[1] - node1.point[1]
    return np.arctan2(dy, dx)

def plot_tree_3d(tree, start, goal, obstacles, path=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
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
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('RRT* in 3D')
    ax.legend()
    plt.show()

'''
# Example usage
start = np.array([1, 1, 1, 0])
goal = np.array([4, 3, 2, 0])
dim_x = 10
dim_y = 10
dim_z = 10
obstacles = [{'point': np.array([5, 5, 5]), 'radius': 1}]
tree, path = rrt_star(start, goal, obstacles, dim_x=dim_x, dim_y=dim_y, dim_z=dim_z, step_size=0.5, radius=2)
plot_tree_3d(tree, start, goal, obstacles, path)
'''
