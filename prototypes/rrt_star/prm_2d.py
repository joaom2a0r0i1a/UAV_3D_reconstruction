import numpy as np
import random
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

class PRM:
    def __init__(self, num_samples, num_neighbors, robot_radius, world_bounds, obstacles, start, goal):
        self.num_samples = num_samples
        self.num_neighbors = num_neighbors
        self.robot_radius = robot_radius
        self.world_bounds = world_bounds
        self.obstacles = obstacles
        self.start = start
        self.goal = goal
        self.samples = self.generate_samples()
        self.graph = self.build_graph()

    def generate_samples(self):
        samples = [self.start, self.goal]  # Include start and goal in samples
        while len(samples) < self.num_samples:
            sample = sample_space(self.world_bounds)
            if not collides(sample, self.obstacles):
                samples.append(sample)
        return np.array(samples)

    def build_graph(self):
        kdtree = KDTree(self.samples)
        graph = {}
        for i, sample in enumerate(self.samples):
            _, indices = kdtree.query(sample, k=self.num_neighbors+1)
            neighbors = []
            for idx in indices[1:]:  # Skip the first index (it's the point itself)
                neighbor = self.samples[idx]
                if not collides_line(sample, neighbor, self.obstacles):
                    neighbors.append(idx)
            graph[i] = neighbors
        return graph

    def plot(self, path=None):
        plt.figure()
        for obstacle in self.obstacles:
            circle = plt.Circle(obstacle['point'], obstacle['radius'], color='r')
            plt.gca().add_patch(circle)
        for node, neighbors in self.graph.items():
            for neighbor in neighbors:
                plt.plot([self.samples[node][0], self.samples[neighbor][0]],
                         [self.samples[node][1], self.samples[neighbor][1]], 'k-', lw=0.5)
        plt.plot(self.samples[:, 0], self.samples[:, 1], 'bo')
        #plt.plot(self.start[0], self.start[1], 'go', label='Start')
        #plt.plot(self.goal[0], self.goal[1], 'ro', label='Goal')
        if path:
            plt.plot([p[0] for p in path], [p[1] for p in path], 'b-', linewidth=2, label='Path')
        plt.xlim(self.world_bounds[0][0], self.world_bounds[1][0])
        plt.ylim(self.world_bounds[0][1], self.world_bounds[1][1])
        plt.gca().set_aspect('equal', adjustable='box')
        plt.title('Probabilistic Roadmap')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        #plt.legend()
        plt.show()

def sample_space(bounds):
    return np.random.uniform(bounds[0], bounds[1])

def collides(point, obstacles):
    for obstacle in obstacles:
        if np.linalg.norm(point - obstacle['point']) < obstacle['radius'] + 0.5:
            return True
    return False

def collides_line(point1, point2, obstacles):
    for obstacle in obstacles:
        if np.linalg.norm(obstacle['point'] - point1) < obstacle['radius'] + 0.5:
            return True
        if np.linalg.norm(obstacle['point'] - point2) < obstacle['radius'] + 0.5:
            return True
    return False

# Example usage
num_samples = 100
num_neighbors = 5
robot_radius = 1.5
world_bounds = ((0, 0), (11, 11))
obstacles = [{'point': np.array([5, 5]), 'radius': 1}, 
             {'point': np.array([7, 3]), 'radius': 0.5}, 
             {'point': np.array([2, 8]), 'radius': 0.6}]
start = np.array([1, 1])
goal = np.array([10, 10])

prm = PRM(num_samples, num_neighbors, robot_radius, world_bounds, obstacles, start, goal)
prm.plot()
