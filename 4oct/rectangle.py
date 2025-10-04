class Rectangle:
    """Класс для работы с прямоугольниками."""

    def __init__(self, width: float, height: float):
        if width <= 0 or height <= 0:
            raise ValueError("Ширина и высота должны быть положительными числами")
        self.width = width
        self.height = height

    def area(self) -> float:
        """Вычисляет площадь прямоугольника."""
        return self.width * self.height

    def perimeter(self) -> float:
        """Вычисляет периметр прямоугольника."""
        return 2 * (self.width + self.height)

    def __str__(self) -> str:
        return f"Прямоугольник {self.width}x{self.height}"


if __name__ == "__main__":
    rect = Rectangle(5, 3)
    print(rect)
    print(f"Площадь: {rect.area()}")
    print(f"Периметр: {rect.perimeter()}")
