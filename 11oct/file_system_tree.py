"""
Дерево файловой системы
"""

import os


class FileSystemNode:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.children = []
        self.is_file = os.path.isfile(path)
        self.size = os.path.getsize(path) if self.is_file else 0
    
    def build_tree(self, max_depth=3, current_depth=0):
        """Рекурсивное построение дерева файловой системы"""
        if current_depth >= max_depth or self.is_file:
            return
        
        try:
            for item in os.listdir(self.path):
                item_path = os.path.join(self.path, item)
                child = FileSystemNode(item_path)
                self.children.append(child)
                child.build_tree(max_depth, current_depth + 1)
        except PermissionError:
            pass
    
    def display(self, level=0):
        """Отображение структуры"""
        indent = "  " * level
        node_type = "📄" if self.is_file else "📁"
        size_info = f" ({self.size} bytes)" if self.is_file else ""
        print(f"{indent}{node_type} {self.name}{size_info}")
        
        for child in self.children:
            child.display(level + 1)
    
    def total_size(self):
        """Общий размер директории"""
        if self.is_file:
            return self.size
        
        total = 0
        for child in self.children:
            total += child.total_size()
        return total


if __name__ == "__main__":
    # Пример использования - сканирование текущей директории
    current_dir = os.path.dirname(os.path.abspath(__file__))
    root = FileSystemNode(current_dir)
    root.build_tree(max_depth=2)
    
    print("Структура директории:")
    root.display()
    print(f"\nОбщий размер: {root.total_size()} bytes")
