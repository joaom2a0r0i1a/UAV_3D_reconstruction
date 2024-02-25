import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None
        self.cost = 0

def rrt_star(start, goal, obstacles, max_iter=1000, step_size=0.1, radius=0.5):
    tree = [Node(start)]
    
    for _ in range(max_iter):
        rand_point = sample_space()
        nearest_node = find_nearest(tree, rand_point)
        new_node = steer(nearest_node, rand_point, step_size)
        
        if not collides(new_node, obstacles):
            new_node.parent = nearest_node
            tree.append(new_node)
            
            if np.linalg.norm(new_node.point - goal) < step_size:
                print("Goal reached!")
                break
    
    return tree

def sample_space():
    return np.random.rand(2) * 10  # Random point in a 2D space

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
        if np.linalg.norm(point.point - obstacle['point']) < obstacle['radius']:
            return True
    return False

def plot_tree(tree, start, goal, obstacles):
    plt.figure()
    for node in tree:
        if node.parent:
            plt.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'k-', linewidth=0.5)
    for obstacle in obstacles:
        circle = plt.Circle(obstacle['point'], obstacle['radius'], color='r')
        plt.gca().add_patch(circle)
    plt.scatter(start[0], start[1], color='g', marker='o', label='Start')
    plt.scatter(goal[0], goal[1], color='b', marker='o', label='Goal')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('RRT*')
    plt.legend()
    plt.axis('equal')
    plt.show()

# Example usage
start = np.array([1, 1])
goal = np.array([9, 9])
obstacles = [{'point': np.array([5, 5]), 'radius': 1}]
tree = rrt_star(start, goal, obstacles)
plot_tree(tree, start, goal, obstacles)
