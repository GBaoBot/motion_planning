#!/usr/bin/env python3
import numpy as np
import pylab as pl
import sys
import utils.environment_2d as environment_2d
from utils.map import Map
from utils.search_algorithm import SearchAlgo
from prm import PRM
from rrt import BidirectionalRRT
from post_processing import PostProcessing

import rospy


class MotionPlanning:
    def __init__(self):
        rospy.init_node('motion_planning', anonymous=True)
        self.algorithm = rospy.get_param("algorithm", "rrt")

        # Map
        self.map_config = rospy.get_param("map_config", {})
        self.random_seed = self.map_config.get(
            "random_seed", None)  # Default to None if not present
        self.size_x = self.map_config.get("size_x", 10)
        self.size_y = self.map_config.get("size_y", 6)
        self.n_obs = self.map_config.get("n_obs", 4)
        self.robot_radius = self.map_config.get("robot_radius", 0.05)

        # Post Processing
        self.post_processing_method = rospy.get_param("method", "")
        self.path_shortcut = rospy.get_param("path_shortcut", {})
        self.max_rep_path_shortcut = self.path_shortcut.get("max_rep")

    def run(self):
        rospy.loginfo("Motion Planning started!")

        if self.algorithm.lower() == 'prm':
            rospy.loginfo("Running PRM algorithm")
            self.run_prm()

        elif self.algorithm.lower() == 'rrt':
            rospy.loginfo("Running RRT algorithm")
            self.run_rrt()

    def run_prm(self):
        num_samples = rospy.get_param("num_samples", 100)
        max_edge_length = rospy.get_param("max_edge_length", 1)

        map = Map(robot_radius=self.robot_radius)
        prm = PRM(num_samples=num_samples, max_edge_length=max_edge_length)
        search = SearchAlgo()
        pp = PostProcessing()

        # Initialize Map
        map.generate_2D_map_with_obstacles(
            self.size_x, self.size_y, self.n_obs)
        map.generate_start_and_goal()

        # Generate PRM
        samples = prm.sample_free_space(map)
        graph = prm.generate_road_map(map, samples)  # Adjacent list

        # Find Path - Dijkstra's Algorithm
        path, _ = search.dijkstra(graph, map.get_start(), map.get_goal())
        if (len(path) == 0):
            rospy.logerr("Cannot find path!")
            return

        # Post processing - Path Shortcut
        path = pp.path_short_cutting(map, path, self.max_rep_path_shortcut)

        # Draw on map
        map.plot_samples_on_map(samples)
        map.plot_path_on_map(path)
        map.show_map()

    def run_rrt(self):
        max_rep = rospy.get_param("max_rep", 100)
        radius = rospy.get_param("radius", 1)

        map = Map()
        rrt = BidirectionalRRT(radius=radius)
        pp = PostProcessing()

        # Initialize Map
        map.generate_2D_map_with_obstacles(
            self.size_x, self.size_y, self.n_obs)
        map.generate_start_and_goal()

        # Find Path - Run Bidirectional RRT
        path = rrt.run(map, map.get_start(), map.get_goal(), max_rep)
        if (len(path) == 0):
            rospy.logerr("Cannot find path!")
            return

        # Post processing - Path Shortcut
        path = pp.path_short_cutting(map, path, self.max_rep_path_shortcut)

        # Draw on map
        map.plot_path_on_map(path)
        map.show_map()


if __name__ == "__main__":
    motion_planning = MotionPlanning()
    motion_planning.run()
