"""Пример из `basics.md`: работа с классом `BankAccount`."""


class BankAccount:
    bank_name = "Python Bank"

    def __init__(self, account_holder, balance=0):
        self.account_holder = account_holder
        self.balance = balance
        self.account_number = id(self)

    def deposit(self, amount):
        if amount > 0:
            self.balance += amount
            return f"Внесено {amount}. Баланс: {self.balance}"
        return "Сумма должна быть положительной"

    def withdraw(self, amount):
        if 0 < amount <= self.balance:
            self.balance -= amount
            return f"Снято {amount}. Баланс: {self.balance}"
        return "Недостаточно средств или неверная сумма"

    def get_balance(self):
        return f"Баланс счета: {self.balance}"


if __name__ == "__main__":
    account1 = BankAccount("Иван Иванов", 1000)
    account2 = BankAccount("Петр Петров")

    print("=== Пример BankAccount ===")
    print(account1.deposit(500))
    print(account1.withdraw(200))
    print(account2.deposit(100))
    print(f"Банк: {BankAccount.bank_name}")
