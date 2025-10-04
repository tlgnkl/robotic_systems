"""Пример из `basics.md`: класс `Student` и расчёт среднего балла."""


class Student:
    university = "Python University"

    def __init__(self, name, student_id):
        self.name = name
        self.student_id = student_id
        self.grades = []

    def add_grade(self, grade):
        if 0 <= grade <= 100:
            self.grades.append(grade)
            return f"Оценка {grade} добавлена"
        return "Оценка должна быть от 0 до 100"

    def get_average(self):
        if self.grades:
            return sum(self.grades) / len(self.grades)
        return 0

    def get_info(self):
        return f"Студент: {self.name}, ID: {self.student_id}, Средний балл: {self.get_average():.2f}"


if __name__ == "__main__":
    student1 = Student("Анна", "S001")
    student2 = Student("Михаил", "S002")

    print("=== Пример Student ===")
    print(student1.add_grade(85))
    print(student1.add_grade(92))
    print(student1.add_grade(78))
    print(student2.add_grade(90))
    print(student2.add_grade(88))
    print(student1.get_info())
    print(student2.get_info())
