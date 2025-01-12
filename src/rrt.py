#!/usr/bin/env python3
import numpy as np
import pylab as pl
import sys
import utils.environment_2d as environment_2d
from utils.map import Map


class BidirectionalRRT:
    def __init__(self, radius: float):
        self.r = radius
        self.start_tree = []
        self.goal_tree = []
        self.path = []
        self.parent = {}

    def run(self, map: Map, start: tuple, goal: tuple, max_rep: int) -> list:
        """
        Runs the Bidirectional RRT algorithm to find a path from start to goal.

        Args:
            map (Map): The map object to navigate.
            start (tuple): The starting point coordinates.
            goal (tuple): The goal point coordinates.
            max_rep (int): The maximum number of iterations.

        Returns:
            list: The path from start to goal if found, otherwise an empty list.
        """
        # Initialize 2 trees
        self.start_tree.append(start)
        self.goal_tree.append(goal)
        self.parent[start] = None
        self.parent[goal] = None

        for _ in range(max_rep):
            # Build start_tree
            rand_point = map.get_random_point()

            new_start_node = self.extend(map, self.start_tree, rand_point)
            if new_start_node:
                node_from_other_tree = self.connect_tree(
                    map, new_start_node, self.goal_tree)
                if node_from_other_tree:
                    self.path = self.merge_tree(
                        new_start_node, node_from_other_tree)
                    return self.path

            new_goal_node = self.extend(map, self.goal_tree, rand_point)
            if new_goal_node:
                node_from_other_tree = self.connect_tree(
                    map, new_goal_node, self.start_tree)
                if node_from_other_tree:
                    self.path = self.merge_tree(
                        node_from_other_tree, new_goal_node)
                    return self.path

        return []

    def connect_tree(self, map: Map, node: tuple, tree: list):
        '''
        To check if the input node can be connected to the input tree:
        1. From the input tree, take the node which is nearest to the input node
        2. Check:
            - If it returns a node (meaning this is a valid path)
            - If the distance between 2 nodes is less than r
            
        Args:
            map (Map): The map object to check distances and collisions.
            node (tuple): The node to connect.
            tree (list): The tree to connect to.

        Returns:
            tuple: The nearest node from the tree if a valid connection is found, otherwise None.
        
        '''
        nearest_node = self.get_nearest_node(tree, node)
        if nearest_node and map.get_distance_if_valid(nearest_node, node) != -1 and np.linalg.norm(np.array(nearest_node) - np.array(node)) < self.r:
            return nearest_node
        return None

    def extend(self, map: Map, tree: list, rand_point: tuple) -> tuple:
        """
        Extends the given tree in the direction of the new random point.

        Args:
            map (Map): The map object to check distances and collisions.
            tree (list): The tree to extend.
            rand_point (tuple): The random point to extend towards.

        Returns:
            tuple: The new node if extension is successful, otherwise None.
        """
        nearest_node = self.get_nearest_node(tree, rand_point)
        new_node = self.steer(map, nearest_node, rand_point)

        if new_node:
            tree.append(new_node)
            self.parent[new_node] = nearest_node
        return new_node

    def get_nearest_node(self, tree: list, node: tuple) -> tuple:
        """
        Returns the node from the given tree that is nearest to the input node.

        Args:
            tree (list): The tree to search.
            node (tuple): The node to find the nearest neighbor for.

        Returns:
            tuple: The nearest node from the tree.
        """
        return min(tree, key=lambda node_from_tree: np.linalg.norm(np.array(node) - np.array(node_from_tree)))

    def steer(self, map: Map, start_node: tuple, goal_node: tuple) -> tuple:
        """
        Steers from the start node towards the goal node.

        Args:
            map (Map): The map object to check distances and collisions.
            start_node (tuple): The starting node coordinates.
            goal_node (tuple): The goal node coordinates.

        Returns:
            tuple: The new node coordinates if the path is valid, otherwise None.
        """
        x1, y1 = start_node
        x2, y2 = goal_node
        dx = x2 - x1
        dy = y2 - y1

        dist = np.sqrt(dx**2 + dy**2)

        if (dist > self.r):
            ratio = self.r / dist
            x2 = x1 + ratio * dx
            y2 = y1 + ratio * dy

        p1 = (x1, y1)
        p2 = (x2, y2)
        if (map.get_distance_if_valid(p1, p2) != -1):
            return (x2, y2)
        return None

    def merge_tree(self, connection_node_start: tuple, connection_node_goal: tuple) -> list:
        """
        Merges the two trees at the connection nodes.

        Args:
            connection_node_start (tuple): The connection node from the start tree.
            connection_node_goal (tuple): The connection node from the goal tree.

        Returns:
            list: The merged path from start to goal.
        """
        path_from_start = self.extract_path(connection_node_start)
        path_from_goal = self.extract_path(connection_node_goal)

        path_from_start.reverse()

        return path_from_start + path_from_goal

    def extract_path(self, leaf: tuple) -> list:
        """
        Extracts the path from the leaf node to the root.

        Args:
            leaf (tuple): The leaf node to start extraction from.

        Returns:
            list: The path from the leaf to the root.
        """
        p = []
        current = leaf
        while current is not None:
            p.append(current)
            current = self.parent[current]
        return p

    def plot_path_on_map(self, map: Map, path: list):
        """
        Plots the path on the map.

        Args:
            map (Map): The map object to plot on.
            path (list): The path to plot.
        """
        if path is None:
            print("No path found")
            return

        for node in path:
            pl.plot(node[0], node[1], 'bo')

        pl.plot(map.start[0], map.start[1], 'go', markersize=10)
        pl.plot(map.goal[0], map.goal[1], 'ro', markersize=10)

        for i in range(len(path) - 1):
            pl.plot([path[i][0], path[i+1][0]],
                    [path[i][1], path[i+1][1]], 'b-')

        # map.plot()


if __name__ == "__main__":
    map = Map()
    rrt = BidirectionalRRT(radius=1)
    map.generate_2D_map_with_obstacles(20, 10, 8)
    map.generate_start_and_goal()

    path = rrt.run(map, map.get_start(), map.get_goal(), 1000)

    map.plot_path_on_map(path)
    map.show_map()
