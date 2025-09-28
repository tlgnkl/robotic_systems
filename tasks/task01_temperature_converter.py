"""Temperature converter between Celsius, Fahrenheit, and Kelvin.

Each exercise from `home_work_python.md` is implemented in its own file inside
`tasks/`. This module contains the solution for Exercise 1.
"""

def _normalize_scale(raw_scale: str) -> str:
    """Return the canonical one-letter scale name or an empty string if invalid."""
    scale_map = {
        "c": "C",
        "celsius": "C",
        "ц": "C",
        "цельсий": "C",
        "f": "F",
        "fahrenheit": "F",
        "ф": "F",
        "фаренгейт": "F",
        "k": "K",
        "kelvin": "K",
        "к": "K",
        "кельвин": "K",
    }
    return scale_map.get(raw_scale.strip().lower(), "")

def _celsius_to_fahrenheit(celsius: float) -> float:
    return celsius * 9 / 5 + 32

def _celsius_to_kelvin(celsius: float) -> float:
    return celsius + 273.15

def _fahrenheit_to_celsius(fahrenheit: float) -> float:
    return (fahrenheit - 32) * 5 / 9

def _kelvin_to_celsius(kelvin: float) -> float:
    return kelvin - 273.15

def temperature_converter() -> None:
    """Interactively convert a temperature value between C, F, and K."""
    input_value = input("Введите температуру (например, 23.5): ")
    try:
        temperature = float(input_value.replace(",", "."))
    except ValueError:
        print("Ошибка: не удалось преобразовать температуру в число.")
        return

    raw_scale = input("Введите исходную шкалу (C/F/K или слово): ")
    scale = _normalize_scale(raw_scale)
    if not scale:
        print("Ошибка: неизвестная шкала температуры. Используйте C, F или K.")
        return

    if scale == "K" and temperature < 0:
        print("Ошибка: температура в Кельвинах не может быть отрицательной.")
        return

    if scale == "C":
        celsius = temperature
    elif scale == "F":
        celsius = _fahrenheit_to_celsius(temperature)
    else:  # scale == "K"
        celsius = _kelvin_to_celsius(temperature)

    kelvin = _celsius_to_kelvin(celsius)
    if kelvin < 0:
        print("Ошибка: получено некорректное значение Кельвина. Проверьте ввод.")
        return

    fahrenheit = _celsius_to_fahrenheit(celsius)

    print("\n=== РЕЗУЛЬТАТ ПРЕОБРАЗОВАНИЯ ===")
    print(f"Цельсий: {celsius:.2f}°C")
    print(f"Фаренгейт: {fahrenheit:.2f}°F")
    print(f"Кельвин: {kelvin:.2f}K")

if __name__ == "__main__":
    temperature_converter()
