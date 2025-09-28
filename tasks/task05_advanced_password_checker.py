"""Advanced password checker for Exercise 5 from `home_work_python.md`.

Evaluates password strength against multiple criteria and provides feedback.
"""

from __future__ import annotations

import re

COMMON_PASSWORDS = {
    "123456",
    "password",
    "qwerty",
    "letmein",
    "111111",
    "12345678",
    "abc123",
    "iloveyou",
    "7777777",
    "123123",
    "qwerty123",
    "password1",
}

SPECIAL_CHARS = set("!@#$%^&*()_+-=[]{}|;:,.<>?/\\")

def advanced_password_checker() -> None:
    """Ask the user for a password, evaluate it, and print recommendations."""
    password = input("Введите пароль для проверки: ")

    length = len(password)
    has_upper = any(ch.isupper() for ch in password)
    has_lower = any(ch.islower() for ch in password)
    has_digit = any(ch.isdigit() for ch in password)
    has_special = any(ch in SPECIAL_CHARS for ch in password)
    not_common = password.lower() not in COMMON_PASSWORDS

    score = 0
    score += 1 if length >= 12 else 0
    score += 1 if has_upper else 0
    score += 1 if has_lower else 0
    score += 1 if has_digit else 0
    score += 1 if has_special else 0
    score += 1 if not_common else 0

    # Normalize score to 5-point scale (cap at 5)
    normalized_score = min(score, 5)

    if normalized_score >= 5:
        verdict = "Отличная защита"
    elif normalized_score >= 4:
        verdict = "Хорошая защита"
    elif normalized_score >= 3:
        verdict = "Средняя защита"
    else:
        verdict = "Слабая защита"

    print("\n=== ПРОВЕРКА ПАРОЛЯ ===")
    print(f"Длина >= 12: {length >= 12} (факт: {length})")
    print(f"Заглавные буквы: {has_upper}")
    print(f"Строчные буквы: {has_lower}")
    print(f"Цифры: {has_digit}")
    print(f"Спецсимволы: {has_special}")
    print(f"Не из списка популярных: {not_common}")
    print(f"Оценка безопасности: {normalized_score}/5 ({verdict})")

    recommendations = []
    if length < 12:
        recommendations.append("увеличьте длину до 12+ символов")
    if not has_upper:
        recommendations.append("добавьте заглавные буквы")
    if not has_lower:
        recommendations.append("добавьте строчные буквы")
    if not has_digit:
        recommendations.append("добавьте цифры")
    if not has_special:
        recommendations.append("добавьте специальные символы")
    if not not_common:
        recommendations.append("используйте менее популярный пароль")

    if recommendations:
        print("Рекомендации:")
        for tip in recommendations:
            print(f"- {tip}")
    else:
        print("Пароль соответствует всем требованиям!")

if __name__ == "__main__":
    advanced_password_checker()
