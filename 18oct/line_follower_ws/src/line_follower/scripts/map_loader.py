#!/usr/bin/env python3

import rospy
import yaml
import numpy as np


class MapLoader:
    def __init__(self):
        self.map_data = None
        self.map_info = None
        self.graph = None

    def load_map(self, map_yaml_path):
        """Загрузка карты из YAML файла"""
        try:
            with open(map_yaml_path, 'r') as file:
                self.map_info = yaml.safe_load(file)

            map_image_path = map_yaml_path.replace('.yaml', '.pgm')
            self.map_data = self._load_pgm(map_image_path)

            rospy.loginfo("Карта успешно загружена")
            self._build_graph_from_map()
            return True

        except Exception as e:
            rospy.logerr(f"Ошибка загрузки карты: {e}")
            return False

    def _load_pgm(self, image_path):
        """Чтение PGM (P2) без использования Pillow"""
        with open(image_path, 'r') as pgm_file:
            # Магическое число
            header = pgm_file.readline().strip()
            if header != 'P2':
                raise ValueError(f"Unsupported PGM format: {header}")

            # Пропускаем комментарии
            line = pgm_file.readline().strip()
            while line.startswith('#'):
                line = pgm_file.readline().strip()

            width, height = map(int, line.split())
            max_value = int(pgm_file.readline().strip())
            if max_value <= 0:
                raise ValueError("Invalid max pixel value in PGM")

            pixels = np.loadtxt(pgm_file, dtype=np.int32)
            if pixels.size != width * height:
                raise ValueError("Pixel count does not match image dimensions")

            return pixels.reshape((height, width))

    def _build_graph_from_map(self):
        """Построение графа из карты линий"""
        height, width = self.map_data.shape
        self.graph = {}

        line_threshold = 128

        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] < line_threshold:
                    node_id = f"{x}_{y}"
                    neighbors = []

                    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                        nx, ny = x + dx, y + dy
                        if (
                            0 <= nx < width
                            and 0 <= ny < height
                            and self.map_data[ny, nx] < line_threshold
                        ):
                            neighbors.append(f"{nx}_{ny}")

                    self.graph[node_id] = neighbors

        rospy.loginfo(f"Граф построен: {len(self.graph)} узлов")

    def get_graph(self):
        return self.graph

    def get_map_info(self):
        return self.map_info

    def pixel_to_world(self, pixel_x, pixel_y):
        """Преобразование координат пикселя в мировые координаты"""
        if not self.map_info:
            return pixel_x, pixel_y

        resolution = self.map_info['resolution']
        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]

        world_x = origin_x + (pixel_x * resolution)
        world_y = origin_y + ((self.map_data.shape[0] - pixel_y) * resolution)

        return world_x, world_y
