"""Bulls and Cows game for Exercise 10 from `home_work_python.md`.

Implements a 4-digit game without repeating digits. Limits attempts to 10 and
reports bulls (correct digit & position) and cows (correct digit, wrong position).
"""

from __future__ import annotations

import random

def _generate_secret() -> str:
    digits = list("0123456789")
    random.shuffle(digits)
    # ensure first digit not zero
    if digits[0] == "0":
        for i in range(1, len(digits)):
            if digits[i] != "0":
                digits[0], digits[i] = digits[i], digits[0]
                break
    return "".join(digits[:4])

def _evaluate_guess(secret: str, guess: str) -> tuple[int, int]:
    bulls = sum(s == g for s, g in zip(secret, guess))
    cows = sum(min(secret.count(d), guess.count(d)) for d in set(guess)) - bulls
    return bulls, cows

def bulls_and_cows() -> None:
    """Play Bulls and Cows with the user."""
    secret = _generate_secret()
    attempts = 0
    max_attempts = 10

    print("=== ИГРА 'БЫКИ И КОРОВЫ' ===")
    print("Компьютер загадал 4-значное число без повторов. У вас 10 попыток.")

    while attempts < max_attempts:
        guess = input(f"Попытка {attempts + 1}/{max_attempts}. Ваш ответ: ")
        if len(guess) != 4 or not guess.isdigit() or len(set(guess)) != 4:
            print("Ошибка: введите 4-значное число без повторяющихся цифр.")
            continue

        attempts += 1
        bulls, cows = _evaluate_guess(secret, guess)
        print(f"Быки: {bulls}, Коровы: {cows}")

        if bulls == 4:
            print(f"Поздравляем! Вы угадали число {secret} за {attempts} попыток!")
            break
    else:
        print(f"Игра окончена. Загаданное число было: {secret}")

if __name__ == "__main__":
    bulls_and_cows()
