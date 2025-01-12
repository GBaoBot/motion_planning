#!/usr/bin/env python3
import numpy as np
import pylab as pl
import sys
import utils.environment_2d as environment_2d

class Map():
    def __init__(self, random_seed=4, robot_radius=0.005):
        if random_seed is not None:
            np.random.seed(random_seed)
        self.env = None
        self.start = None
        self.goal = None
        self.size_x = 0
        self.size_y = 0
        self.n_obs = 0
        self.robot_radius = robot_radius
        
    def generate_2D_map_with_obstacles(self, size_x, size_y, n_obs):
        self.size_x = size_x
        self.size_y = size_y
        self.n_obs = n_obs
        self.env = environment_2d.Environment(size_x, size_y, n_obs)
        pl.clf()
        self.env.plot()
         
    def generate_start_and_goal(self):
        q = self.env.random_query()
        if q is not None:
            x_start, y_start, x_goal, y_goal = q
            self.env.plot_query(x_start, y_start, x_goal, y_goal)
        self.start = (x_start, y_start)
        self.goal = (x_goal, y_goal)
        return self.start, self.goal
    
    def show_map(self, block=True):
        pl.show(block=block)
        
    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal
    
    def get_distance_if_valid(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        dist = np.sqrt(dx**2 + dy**2)
        
        # if (self.max_edge_length is not None and dist > self.max_edge_length):
        #     return -1
        
        if (dist < self.robot_radius):
            return -1
        num_steps = int(dist / self.robot_radius)
        for i in range(num_steps + 1):
            x = x1 + dx * i / num_steps
            y = y1 + dy * i / num_steps
            if self.env.check_collision(x, y):
                return -1
            
        return dist
    
    def check_collision(self, p):
        return self.env.check_collision(*p)
    
    def get_random_point(self):
        return (np.random.uniform(0, self.size_x), np.random.uniform(0, self.size_y))
        
    def plot_samples_on_map(self, samples):
        sample_x, sample_y = zip(*samples)
        pl.scatter(sample_x, sample_y, color='green', s=10, label='Sampled Points')
    
    def plot_graph_on_map(self, graph):
        for node, neighbors in graph.items():
            for neighbor, w in neighbors:
                x_values = [node[0], neighbor[0]]
                y_values = [node[1], neighbor[1]]
                pl.plot(x_values, y_values, color='blue', linewidth=0.5)
    
    def plot_path_on_map(self, path):
        if path is None:
            print("No path found")
            return
        
        for node in path:
            pl.plot(node[0], node[1], 'bo')
        
        pl.plot(self.start[0], self.start[1], 'go', markersize=10)
        pl.plot(self.goal[0], self.goal[1], 'ro', markersize=10)
        
        for i in range(len(path) - 1):
            pl.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], 'b-')
        