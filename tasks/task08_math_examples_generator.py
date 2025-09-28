"""Math examples generator for Exercise 8 from `home_work_python.md`.

Generates ten arithmetic problems with mixed operations, checks user answers, and
reports statistics.
"""

from __future__ import annotations

import operator
import random
from typing import Callable, Tuple

Operation = Tuple[str, Callable[[int, int], float]]

OPERATIONS: tuple[Operation, ...] = (
    ("+", lambda a, b: a + b),
    ("-", lambda a, b: a - b),
    ("*", lambda a, b: a * b),
    ("/", lambda a, b: round(a / b, 2)),
)

QUESTION_COUNT = 10


def _generate_operands(op_symbol: str) -> tuple[int, int]:
    if op_symbol == "/":
        a = random.randint(2, 20)
        b = random.randint(1, 10)
        return a, b
    return random.randint(1, 20), random.randint(1, 20)


def math_examples_generator() -> None:
    """Interactively run ten arithmetic challenges and show summary."""
    correct = 0
    results: list[str] = []

    for i in range(1, QUESTION_COUNT + 1):
        op_symbol, func = random.choice(OPERATIONS)
        a, b = _generate_operands(op_symbol)
        correct_result = func(a, b)

        try:
            answer_raw = input(f"{i}. {a} {op_symbol} {b} = ? ")
            user_answer = float(answer_raw.replace(",", "."))
        except ValueError:
            results.append(f"{i}. Неверный формат ответа ({answer_raw!r})")
            continue

        if op_symbol == "/":
            is_correct = abs(user_answer - correct_result) <= 0.01
        else:
            is_correct = user_answer == correct_result

        if is_correct:
            correct += 1
            results.append(f"{i}. Верно ✅ ({user_answer})")
        else:
            results.append(
                f"{i}. Неверно ❌ (ваш ответ: {user_answer}, верный: {correct_result})"
            )

    score_percent = correct / QUESTION_COUNT * 100

    print("\n=== РЕЗУЛЬТАТЫ ===")
    for line in results:
        print(line)
    print(f"\nПравильных ответов: {correct}/{QUESTION_COUNT} ({score_percent:.0f}%)")

if __name__ == "__main__":
    math_examples_generator()
