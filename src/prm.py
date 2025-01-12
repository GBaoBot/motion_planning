#!/usr/bin/env python3
import numpy as np
import pylab as pl
import utils.environment_2d as environment_2d
from utils.map import Map
from utils.search_algorithm import SearchAlgo


class PRM:
    def __init__(self, random_seed=4, num_samples=100, max_edge_length=2):
        if random_seed is not None:
            np.random.seed(random_seed)

        self.num_samples = num_samples
        self.max_edge_length = max_edge_length
        self.graph = {}

    def sample_free_space(self, map: Map):
        samples = []
        cnt = 0
        while (cnt <= self.num_samples):
            p = map.get_random_point()
            if not map.check_collision(p):
                samples.append(p)
                cnt += 1
        return samples

    def generate_road_map(self, map, samples):
        for i, sample1 in enumerate(samples):
            self.graph[sample1] = []
            for sample2 in samples:
                dist = map.get_distance_if_valid(sample1, sample2)
                if dist != -1 and dist < self.max_edge_length:
                    self.graph[sample1].append((sample2, dist))

        return self.graph


if __name__ == "__main__":
    map = Map(robot_radius=0.05)
    prm = PRM(num_samples=100, max_edge_length=2)
    search = SearchAlgo()

    map.generate_2D_map_with_obstacles(10, 6, 4)
    map.generate_start_and_goal()

    samples = prm.sample_free_space(map)
    graph = prm.generate_road_map(map, samples)  # Adjacent list

    path, _ = search.dijkstra(graph, map.get_start(), map.get_goal())
    map.plot_path_on_map(path)

    map.show_map()
