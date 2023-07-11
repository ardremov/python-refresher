class Bank:
  def __init__(self, bal, name, accnum):
    self.bal = bal
    self.name = name
    self.accnum = accnum

  def withdraw(self, amt):
    if amt > self.bal:
      print("You do not have the funds to make this transaction.")
      return
    self.bal -= amt

  def deposit(self, amt):
    self.bal += amt

  def printBal(self):
    print("Your current balance is " + self.bal)

