import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None
        self.cost = 0

def rrt_star(start, goal, obstacles, max_iter=1000, step_size=0.1, radius=0.5):
    tree = [Node(start)]
    tree_rewired = [Node(start)]
    
    for _ in range(max_iter):
        rand_point = sample_space()
        nearest_node = find_nearest(tree, rand_point)
        new_node = steer(nearest_node, rand_point, step_size)
        nearest_node_rewired = find_nearest(tree_rewired, rand_point)
        new_node_rewired = steer(nearest_node_rewired, rand_point, step_size)
        
        if not collides(new_node.point, obstacles):
            nearby_nodes = find_nearby(tree, new_node, radius)
            new_node = choose_parent(new_node, nearby_nodes)
            tree.append(new_node)
            
            if np.linalg.norm(new_node.point - goal) < step_size:
                print("Goal reached!")
                #break

        if not collides(new_node_rewired.point, obstacles):

            nearby_nodes_rewired = find_nearby(tree_rewired, new_node_rewired, radius)
            new_node_rewired = choose_parent(new_node_rewired, nearby_nodes_rewired)
            tree_rewired.append(new_node_rewired)
            rewire(tree_rewired, new_node_rewired, nearby_nodes_rewired, radius)
            
            if np.linalg.norm(new_node_rewired.point - goal) < step_size:
                print("Goal reached!")
                #break
    
    return tree, tree_rewired

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
        if np.linalg.norm(point - obstacle['point']) < obstacle['radius']:
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
    #return tree

def plot_trees(tree, tree_rewired, start, goal, obstacles):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    titles = ['Original RRT*', 'Rewired RRT*']
    
    for idx, tree in enumerate([tree, tree_rewired]):
        ax = axes[idx]
        for node in tree:
            if node.parent:
                ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'k-', linewidth=0.5)
        for obstacle in obstacles:
            circle = plt.Circle(obstacle['point'], obstacle['radius'], color='r')
            ax.add_patch(circle)
        ax.scatter(start[0], start[1], color='g', marker='o', label='Start')
        ax.scatter(goal[0], goal[1], color='b', marker='o', label='Goal')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title(titles[idx])
        ax.legend()
        ax.axis('equal')

    plt.show()

# Example usage
start = np.array([1, 1])
goal = np.array([3, 3])
obstacles = [{'point': np.array([5, 5]), 'radius': 1}]
tree, tree_rewired = rrt_star(start, goal, obstacles)
plot_trees(tree, tree_rewired, start, goal, obstacles)
