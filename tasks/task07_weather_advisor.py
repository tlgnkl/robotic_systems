"""Weather advisor for Exercise 7 from `home_work_python.md`.

Determines season based on month number and suggests clothing according to
temperature, including warnings for extreme conditions.
"""

from __future__ import annotations

SEASONS = {
    "winter": {"months": {12, 1, 2}, "name": "зима"},
    "spring": {"months": {3, 4, 5}, "name": "весна"},
    "summer": {"months": {6, 7, 8}, "name": "лето"},
    "autumn": {"months": {9, 10, 11}, "name": "осень"},
}

CLOTHING_TIPS = [
    (-float("inf"), -20, "экстремально холодно: теплая куртка, шапка, перчатки, шарф"),
    (-20, -10, "очень холодно: пуховик, шапка, варежки"),
    (-10, 0, "холодно: тёплая куртка, шапка"),
    (0, 10, "прохладно: лёгкая куртка или свитер"),
    (10, 20, "умеренно: лёгкая одежда, возможно ветровка"),
    (20, 28, "тепло: футболка, лёгкие брюки/шорты"),
    (28, float("inf"), "жарко: лёгкая одежда, шляпа, вода"),
]

EXTREME_WARNINGS = [
    (-float("inf"), -15, "Осторожно! Возможен сильный мороз и гололед."),
    (32, float("inf"), "Осторожно! Риск теплового удара, избегайте солнца."),
]

def _determine_season(month: int) -> str:
    for season in SEASONS.values():
        if month in season["months"]:
            return season["name"]
    raise ValueError("Некорректный месяц")

def _clothing_recommendation(temp: float) -> str:
    for low, high, tip in CLOTHING_TIPS:
        if low < temp <= high:
            return tip
    return "Подберите одежду по погоде."

def _warnings(temp: float) -> list[str]:
    notes = []
    for low, high, message in EXTREME_WARNINGS:
        if low < temp <= high:
            notes.append(message)
    return notes

def weather_advisor() -> None:
    """Prompt user for month and temperature, then print advice."""
    try:
        month = int(input("Введите номер месяца (1-12): "))
    except ValueError:
        print("Ошибка: месяц должен быть целым числом.")
        return

    if not 1 <= month <= 12:
        print("Ошибка: месяц должен быть в диапазоне 1-12.")
        return

    temp_input = input("Введите текущую температуру (°C): ")
    try:
        temperature = float(temp_input.replace(",", "."))
    except ValueError:
        print("Ошибка: температура должна быть числом.")
        return

    season = _determine_season(month)
    tip = _clothing_recommendation(temperature)
    warnings = _warnings(temperature)

    print("\n=== СОВЕТ ПО ПОГОДЕ ===")
    print(f"Сейчас {season}, температура {temperature:.1f}°C")
    print(f"Рекомендация: {tip}")

    if warnings:
        print("\nПредупреждения:")
        for item in warnings:
            print(f"- {item}")

if __name__ == "__main__":
    weather_advisor()
