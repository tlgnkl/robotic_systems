"""Демонстрации из разделов про класс `Dog` из `basics.md`."""


def run_basic_dog_example():
    class Dog:
        species = "Canis familiaris"

        def __init__(self, name, age):
            self.name = name
            self.age = age

        def bark(self):
            return f"{self.name} говорит: Гав!"

        def describe(self):
            return f"{self.name} - {self.age} лет, вид: {self.species}"

    dog1 = Dog("Бадди", 3)
    dog2 = Dog("Чарли", 5)
    dog3 = Dog("Люси", 2)

    print("=== Базовый пример Dog ===")
    print(dog1.name)
    print(dog1.age)
    print(dog1.species)
    print(dog2.name)
    print(dog2.age)
    print(dog1.bark())
    print(dog1.describe())
    print(dog2.bark())


def run_class_attribute_example():
    class Dog:
        species = "Canis familiaris"
        total_dogs = 0

        def __init__(self, name, age):
            self.name = name
            self.age = age
            Dog.total_dogs += 1

    dog1 = Dog("Бадди", 3)
    dog2 = Dog("Чарли", 5)

    print("\n=== Классовые атрибуты и счётчик ===")
    print(dog1.species)
    print(dog2.species)
    print(Dog.total_dogs)

    Dog.species = "Собака"
    print(dog1.species)
    print(dog2.species)

    dog1.name = "Макс"
    print(dog1.name)
    print(dog2.name)


def run_special_methods_example():
    class Dog:
        def __init__(self, name, age):
            self.name = name
            self.age = age

        def __str__(self):
            return f"Собака {self.name}, {self.age} лет"

        def __repr__(self):
            return f"Dog('{self.name}', {self.age})"

    dog = Dog("Бадди", 3)

    print("\n=== Специальные методы __str__ и __repr__ ===")
    print(str(dog))
    print(repr(dog))


if __name__ == "__main__":
    run_basic_dog_example()
    run_class_attribute_example()
    run_special_methods_example()
