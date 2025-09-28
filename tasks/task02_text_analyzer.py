"""Text analyzer utility for Exercise 2 from `home_work_python.md`.

Reads user input and prints statistics about the text, including counts of
characters, words, sentences, uppercase/lowercase distribution, digits, and the
longest word.
"""

from __future__ import annotations

import re
from collections import Counter

def _split_sentences(text: str) -> list[str]:
    """Split text into sentences based on punctuation."""
    raw_sentences = re.split(r"[.!?]+", text)
    return [sentence.strip() for sentence in raw_sentences if sentence.strip()]

def _extract_words(text: str) -> list[str]:
    """Return a list of words, handling Cyrillic and Latin characters."""
    return re.findall(r"[A-Za-zА-Яа-яЁё0-9]+", text)

def text_analyzer() -> None:
    """Interactively analyze text provided by the user."""
    text = input("Введите текст для анализа: ")

    if not text.strip():
        print("Ошибка: пустой ввод. Нечего анализировать.")
        return

    characters_count = len(text)
    words = _extract_words(text)
    words_count = len(words)

    sentences = _split_sentences(text)
    sentences_count = len(sentences)

    longest_word = max(words, key=len) if words else ""

    letter_counts = Counter(ch for ch in text if ch.isalpha())
    total_letters = sum(letter_counts.values())
    uppercase_letters = sum(count for ch, count in letter_counts.items() if ch.isupper())
    lowercase_letters = sum(count for ch, count in letter_counts.items() if ch.islower())

    uppercase_percentage = (uppercase_letters / total_letters * 100) if total_letters else 0.0
    lowercase_percentage = (lowercase_letters / total_letters * 100) if total_letters else 0.0

    digits_count = sum(ch.isdigit() for ch in text)

    print("\n=== РЕЗУЛЬТАТ АНАЛИЗА ===")
    print(f"Символов: {characters_count}")
    print(f"Слов: {words_count}")
    print(f"Предложений: {sentences_count}")

    if longest_word:
        print(f"Самое длинное слово: '{longest_word}' ({len(longest_word)} символов)")
    else:
        print("Самое длинное слово: отсутствует")

    print("\nРегистры букв:")
    print(f"Заглавные: {uppercase_letters} ({uppercase_percentage:.2f}%)")
    print(f"Строчные: {lowercase_letters} ({lowercase_percentage:.2f}%)")

    print(f"\nЦифр в тексте: {digits_count}")

if __name__ == "__main__":
    text_analyzer()
