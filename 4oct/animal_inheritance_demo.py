"""Пример из `basics.md`: наследование классов `Animal`, `Cat`, `Dog`."""


class Animal:
    def __init__(self, name):
        self.name = name

    def speak(self):
        return "Животное издает звук"


class Cat(Animal):
    def speak(self):
        return f"{self.name} говорит: Мяу!"


class Dog(Animal):
    def speak(self):
        return f"{self.name} говорит: Гав!"


if __name__ == "__main__":
    cat = Cat("Мурка")
    dog = Dog("Бадди")

    print("=== Пример наследования Animal -> Cat/Dog ===")
    print(cat.speak())
    print(dog.speak())
