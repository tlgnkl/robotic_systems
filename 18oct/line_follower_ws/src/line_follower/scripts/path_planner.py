#!/usr/bin/env python3

import rospy
import heapq
from collections import deque


class PathPlanner:
    def __init__(self, graph):
        self.graph = graph

    def dijkstra(self, start, goal):
        """Алгоритм Дейкстры для поиска кратчайшего пути"""
        if start not in self.graph or goal not in self.graph:
            rospy.logwarn("Стартовая или целевая точка не найдена в графе")
            return None

        distances = {node: float('inf') for node in self.graph}
        distances[start] = 0
        previous = {node: None for node in self.graph}
        queue = [(0, start)]

        while queue:
            current_distance, current_node = heapq.heappop(queue)

            if current_node == goal:
                break

            if current_distance > distances[current_node]:
                continue

            for neighbor in self.graph[current_node]:
                distance = current_distance + 1

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(queue, (distance, neighbor))

        path = []
        current = goal

        while current is not None:
            path.append(current)
            current = previous[current]

        path.reverse()

        if path and path[0] == start:
            rospy.loginfo(f"Путь найден: {len(path)} точек")
            return path

        rospy.logwarn("Путь не найден")
        return None

    def find_nearest_node(self, x, y):
        """Поиск ближайшего узла графа к заданным координатам"""
        nearest_node = None
        min_distance = float('inf')

        target_coords = (x, y)

        for node in self.graph:
            node_x, node_y = map(int, node.split('_'))
            distance = abs(node_x - x) + abs(node_y - y)

            if distance < min_distance:
                min_distance = distance
                nearest_node = node

        return nearest_node
