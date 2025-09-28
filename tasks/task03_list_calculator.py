"""List calculator for Exercise 3 from `home_work_python.md`.

Performs several operations on a predefined list of numbers and prints the
results.
"""

def list_calculator() -> None:
    """Calculate and display various metrics for a preset list of numbers."""
    numbers = [12, 45, 23, 67, 34, 89, 56]

    total = sum(numbers)
    average = total / len(numbers)
    squares = [num ** 2 for num in numbers]
    filtered = [num for num in numbers if num > 30]
    sorted_desc = sorted(numbers, reverse=True)

    print("Исходный список:", numbers)
    print(f"Сумма всех элементов: {total}")
    print(f"Среднее значение: {average:.2f}")
    print("Список квадратов:", squares)
    print("Числа больше 30:", filtered)
    print("Список по убыванию:", sorted_desc)

if __name__ == "__main__":
    list_calculator()
