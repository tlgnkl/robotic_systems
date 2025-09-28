"""Student management system for Exercise 11 from `home_work_python.md`.

Provides a console menu to manage a list of students with their ages and grades.
Supports adding students, listing them in tabular format, searching by name, and
computing group statistics.
"""

from __future__ import annotations

from statistics import mean

Student = dict[str, object]

def _format_student(student: Student) -> str:
    grades = student["grades"]
    avg = mean(grades) if grades else 0
    return (
        f"{student['name']:<20} | {student['age']:>3} | "
        f"{', '.join(map(str, grades)): <20} | {avg:>5.2f}"
    )

def _print_table(students: list[Student]) -> None:
    if not students:
        print("Список студентов пуст.")
        return

    header = "Имя                 | Возр | Оценки              | Средн"
    print(header)
    print("-" * len(header))
    for student in students:
        print(_format_student(student))

def _add_student(students: list[Student]) -> None:
    name = input("Введите имя студента: ").strip()
    if not name:
        print("Ошибка: имя не может быть пустым.")
        return

    try:
        age = int(input("Введите возраст: "))
        if age <= 0:
            raise ValueError
    except ValueError:
        print("Ошибка: возраст должен быть положительным целым числом.")
        return

    grades_raw = input("Введите оценки через пробел: ")
    try:
        grades = [int(g) for g in grades_raw.split() if g]
    except ValueError:
        print("Ошибка: оценки должны быть целыми числами.")
        return

    student = {"name": name, "age": age, "grades": grades}
    students.append(student)
    print(f"Студент {name} добавлен.")

def _find_student(students: list[Student]) -> None:
    query = input("Введите имя для поиска: ").strip().lower()
    matches = [s for s in students if query in s["name"].lower()]

    if not matches:
        print("Студент не найден.")
        return

    print("Найденные студенты:")
    _print_table(matches)

def _calculate_statistics(students: list[Student]) -> None:
    if not students:
        print("Статистика недоступна: нет студентов.")
        return

    total_students = len(students)
    average_age = mean(student["age"] for student in students)
    all_grades = [grade for student in students for grade in student["grades"]]
    average_grade = mean(all_grades) if all_grades else 0

    print("\n=== СТАТИСТИКА ПО ГРУППЕ ===")
    print(f"Количество студентов: {total_students}")
    print(f"Средний возраст: {average_age:.2f}")
    if all_grades:
        print(f"Средний балл по группе: {average_grade:.2f}")
    else:
        print("Оценки отсутствуют, средний балл не рассчитывается.")

def student_management_system() -> None:
    """Main loop of the student management system."""
    students: list[Student] = []

    while True:
        print("\n=== СИСТЕМА УПРАВЛЕНИЯ СТУДЕНТАМИ ===")
        print("1. Добавить студента")
        print("2. Показать всех студентов")
        print("3. Найти студента по имени")
        print("4. Рассчитать статистику")
        print("5. Выход")

        choice = input("Выберите действие: ").strip()

        if choice == "1":
            _add_student(students)
        elif choice == "2":
            _print_table(students)
        elif choice == "3":
            _find_student(students)
        elif choice == "4":
            _calculate_statistics(students)
        elif choice == "5":
            print("Выход из программы.")
            break
        else:
            print("Неверный выбор. Попробуйте снова.")

if __name__ == "__main__":
    student_management_system()
