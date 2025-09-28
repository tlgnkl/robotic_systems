"""Financial calculator for Exercise 4 from `home_work_python.md`.

Prompts the user for deposit parameters, calculates compound interest, checks a
user-defined goal, and determines when the investment doubles.
"""

from __future__ import annotations

import math

def _prompt_positive_float(message: str) -> float:
    raw = input(message)
    try:
        value = float(raw.replace(",", "."))
    except ValueError as exc:  # pragma: no cover - handled by mapping
        raise ValueError("Введите числовое значение") from exc
    if value <= 0:
        raise ValueError("Значение должно быть положительным")
    return value

def financial_calculator() -> None:
    """Interactively compute compound interest metrics."""
    try:
        principal = _prompt_positive_float("Начальная сумма (руб.): ")
        annual_rate_percent = _prompt_positive_float("Годовая ставка (%): ")
        years = _prompt_positive_float("Срок вклада (лет): ")
        target_amount = _prompt_positive_float(
            "Целевая сумма (руб.) для проверки: "
        )
    except ValueError as error:
        print(f"Ошибка: {error}")
        return

    annual_rate = annual_rate_percent / 100
    final_amount = principal * (1 + annual_rate) ** years

    exceeds_target = final_amount >= target_amount

    if annual_rate == 0:
        years_to_double = math.inf
    else:
        years_to_double = math.log(2) / math.log(1 + annual_rate)

    print("\n=== ФИНАНСОВЫЙ ОТЧЕТ ===")
    print(f"Начальная сумма: {principal:,.2f} руб.".replace(",", " "))
    print(f"Ставка: {annual_rate_percent:.2f}% годовых")
    print(f"Срок: {years:.2f} лет")
    print(f"Конечная сумма: {final_amount:,.2f} руб.".replace(",", " "))
    print(f"Цель {target_amount:,.2f} руб. достигнута: {exceeds_target}".replace(",", " "))

    if math.isinf(years_to_double):
        print("Ставка 0%, сумма никогда не удвоится.")
    else:
        print(f"Сумма удвоится через {years_to_double:.2f} лет")

if __name__ == "__main__":
    financial_calculator()
