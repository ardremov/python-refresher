class Bank:
    # constructor + instance fields
    def __init__(self, name: str, accnum: int, bal: int or float = 0):
        self.name = name
        self.accnum = accnum
        self.bal = bal

    # withdraws amt from bal
    def withdraw(self, amt):
        if amt > self.bal:
            raise ValueError("You do not have the funds to make this transaction.")
        self.bal -= amt
        print(f"You have successfully withdrawn {amt} from your account.")
        return self.bal

    # deposits amt to bal
    def deposit(self, amt):
        self.bal += amt
        print(f"You have successfully deposited {amt} into your account.")
        return self.bal

    # prints current balance
    def printBal(self):
        print(f"Your current balance is {self.bal}.")
        return self.bal
