"""Number sequence analyzer for Exercise 9 from `home_work_python.md`.

Collects numbers until the user types "stop", then prints descriptive statistics,
checks for duplicates, sorts values, finds the median, and renders a simple bar
chart using block characters.
"""

from __future__ import annotations

import math
from collections import Counter

PROMPT = "Введите число (или 'stop' для завершения): "

def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2

def _build_graph(values: list[float]) -> list[str]:
    max_abs = max(abs(v) for v in values)
    if math.isclose(max_abs, 0.0):
        max_abs = 1.0

    graph_lines = []
    for idx, value in enumerate(values, start=1):
        bar_length = max(1, int(round(abs(value) / max_abs * 10)))
        bar = "█" * bar_length
        sign = "-" if value < 0 else ""
        graph_lines.append(f"{idx:>2}. {sign}{bar} ({value:.2f})")
    return graph_lines

def number_sequence_analyzer() -> None:
    """Interactively analyze a user-provided sequence of numbers."""
    numbers: list[float] = []

    while True:
        raw = input(PROMPT)
        if raw.strip().lower() == "stop":
            break
        try:
            value = float(raw.replace(",", "."))
        except ValueError:
            print("Ошибка: введите число или 'stop'.")
            continue
        numbers.append(value)

    if not numbers:
        print("Данные не введены.")
        return

    count = len(numbers)
    total = sum(numbers)
    average = total / count
    maximum = max(numbers)
    minimum = min(numbers)
    median_value = _median(numbers)

    counter = Counter(numbers)
    duplicates = sorted({value for value, freq in counter.items() if freq > 1})

    sorted_numbers = sorted(numbers)
    graph = _build_graph(numbers)

    print("\n=== АНАЛИЗ ПОСЛЕДОВАТЕЛЬНОСТИ ===")
    print(f"Чисел введено: {count}")
    print(f"Максимум: {maximum:.2f}")
    print(f"Минимум: {minimum:.2f}")
    print(f"Среднее: {average:.2f}")
    print(f"Медиана: {median_value:.2f}")
    if duplicates:
        dup_str = ", ".join(f"{value:.2f}" for value in duplicates)
        print(f"Повторяющиеся значения: {dup_str}")
    else:
        print("Повторяющихся значений нет")

    print(f"Отсортированный список: {', '.join(f'{num:.2f}' for num in sorted_numbers)}")

    print("\nГрафик:")
    for line in graph:
        print(line)

if __name__ == "__main__":
    number_sequence_analyzer()
