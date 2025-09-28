"""Tax calculator for Exercise 6 from `home_work_python.md`.

Calculates progressive income tax and prints a breakdown with net income.
"""

from __future__ import annotations

BRACKETS = [
    (15_000, 0.00),
    (50_000, 0.15),
    (100_000, 0.25),
]
TOP_RATE = 0.30

def _calculate_tax(income: float) -> tuple[float, list[tuple[str, float]]]:
    """Return total tax and list of (description, tax_for_bracket)."""
    remaining = income
    lower_limit = 0.0
    breakdown: list[tuple[str, float]] = []
    total_tax = 0.0

    for upper_limit, rate in BRACKETS:
        taxable = max(0.0, min(upper_limit, remaining) - lower_limit)
        if taxable <= 0:
            lower_limit = upper_limit
            continue
        tax = taxable * rate
        breakdown.append((f"{int(lower_limit)} - {int(upper_limit)}", tax))
        total_tax += tax
        lower_limit = upper_limit

    if income > BRACKETS[-1][0]:
        taxable_top = income - BRACKETS[-1][0]
        tax_top = taxable_top * TOP_RATE
        breakdown.append((f"> {int(BRACKETS[-1][0])}", tax_top))
        total_tax += tax_top

    return total_tax, breakdown

def tax_calculator() -> None:
    """Interactively compute progressive tax and net income."""
    raw_income = input("Введите годовой доход (руб.): ")
    try:
        income = float(raw_income.replace(",", "."))
    except ValueError:
        print("Ошибка: требуется числовое значение.")
        return

    if income < 0:
        print("Ошибка: доход не может быть отрицательным.")
        return

    total_tax, breakdown = _calculate_tax(income)
    net_income = income - total_tax

    print("\n=== НАЛОГОВЫЙ ОТЧЕТ ===")
    print(f"Доход: {income:,.2f} руб.".replace(",", " "))
    print(f"Налог к уплате: {total_tax:,.2f} руб.".replace(",", " "))
    print(f"Чистый доход: {net_income:,.2f} руб.".replace(",", " "))

    if breakdown:
        print("\nДетализация по диапазонам:")
        for label, tax in breakdown:
            print(f"- {label}: {tax:,.2f} руб.".replace(",", " "))
    else:
        print("\nНалог не начисляется.")

if __name__ == "__main__":
    tax_calculator()
