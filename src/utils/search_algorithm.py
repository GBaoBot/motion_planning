#!/usr/bin/env python3
import numpy as np
import pylab as pl
import utils.environment_2d as environment_2d
from utils.map import Map


class SearchAlgo:
    def __init__(self):
        self.path = []

    def preprocess(self, graph, starting_point, goal_point):

        def find_nearest_node(node):
            return min(graph.keys(), key=lambda node_from_graph: np.linalg.norm(np.array(node) - np.array(node_from_graph)))

        starting_node = find_nearest_node(starting_point)
        goal_node = find_nearest_node(goal_point)

        return starting_node, goal_node

    def dijkstra(self, graph, starting_point, goal_point):
        import heapq

        starting_node, goal_node = self.preprocess(
            graph, starting_point, goal_point)
        pq = []
        distances = {node: float('inf') for node in graph}
        previous = {node: None for node in graph}

        # Start node setup
        distances[starting_node] = 0
        heapq.heappush(pq, (0, starting_node))

        while pq:
            current_distance, current_node = heapq.heappop(pq)
            
            if current_node == goal_node:
                break
            
            for neighbor, weight in graph.get(current_node, []):
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))

        # Reconstruct the path
        start_node_to_goal_node = self._reconstruct_path(previous, goal_node)
        if previous[goal_node] is None:
            print("No path found")
            return [], float('inf')
        self.path = [starting_point] + start_node_to_goal_node + [goal_point]
        return self.path, distances[goal_node]

    def _reconstruct_path(self, previous, goal):
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = previous[current]
        return path[::-1]  # Reverse the path to get start -> goal
