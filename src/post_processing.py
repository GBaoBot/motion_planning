#!/usr/bin/env python3
import numpy as np
import pylab as pl
from utils.map import Map
from utils.search_algorithm import SearchAlgo
from prm import PRM
from rrt import BidirectionalRRT


class PostProcessing:
    def __init__(self):
        pass

    def path_short_cutting(self, map: Map, path: list, max_rep: int) -> list:
        """
        Shortcuts the path by attempting to connect random pairs of nodes directly.

        Args:
            map (Map): The map object to check distances and collisions.
            path (list): The original path to be shortened.
            max_rep (int): The maximum number of shortcut attempts.

        Returns:
            list: The shortened path.
        """
        if (path is not None and len(path) <= 2):
            print("Nothing to shortcut")
            return path
        for _ in range(max_rep):
            random_indices = np.random.choice(len(path), size=2, replace=False)
            random_indices.sort()
            idx1, idx2 = random_indices
            node1, node2 = path[idx1], path[idx2]

            if map.get_distance_if_valid(node1, node2) != -1:
                path = path[:random_indices[0]+1] + \
                    [node2] + path[random_indices[1]+1:]
        return path

    def trajectory_short_cutting(self, map: Map, path: list, max_rep: int):
        pass

if __name__ == "__main__":
    map = Map()
    # prm = PRM(num_samples=500, max_edge_length=2)
    rrt = BidirectionalRRT(1)
    search = SearchAlgo()
    post_processing = PostProcessing()

    map.generate_2D_map_with_obstacles(10, 6, 5)
    map.generate_start_and_goal()

    path = rrt.run(map, map.get_start(), map.get_goal(), 500)
    path = post_processing.path_short_cutting(map, path, 100)

    map.plot_path_on_map(path)

    map.show_map()
