import unittest
import hello
import bank


class TestHello(unittest.TestCase):
    def test_hello(self):
        self.assertEqual(hello.hello(), "Hello, world!")

    def test_add(self):
        self.assertEqual(hello.add(1, 1), 2)
        self.assertEqual(hello.add(5, 5), 10)
        self.assertEqual(hello.add(0, 0), 0)

    def test_sub(self):
        self.assertEqual(hello.sub(1, 1), 0)
        self.assertEqual(hello.sub(3, 1), 2)
        self.assertEqual(hello.sub(10, 5), 5)

    def test_mul(self):
        self.assertEqual(hello.mul(2, 3), 6)
        self.assertEqual(hello.mul(2, 4), 8)
        self.assertEqual(hello.mul(2, 0), 0)

    def test_div(self):
        self.assertEqual(hello.div(8, 2), 4)
        self.assertEqual(hello.div(1, 1), 1)
        with self.assertRaises(ValueError("Can't divide by zero!")):
            hello.div(1, 0)

    def test_sqrt(self):
        self.assertEqual(hello.sqrt(4), 2)
        self.assertEqual(hello.sqrt(1), 1)
        self.assertEqual(hello.sqrt(0), 0)

    def test_power(self):
        self.assertAlmostEqual(hello.power(1, 2), 1)
        self.assertAlmostEqual(hello.power(2, 2), 4)
        self.assertAlmostEqual(hello.power(10, 0), 1)

    def test_log(self):
        self.assertAlmostEqual(hello.log(2.71828182846), 1)
        self.assertAlmostEqual(hello.log(7.38905609893), 2)
        self.assertAlmostEqual(hello.log(20.0855369232), 3)

    def test_exp(self):
        self.assertEqual(hello.exp(0), 1)
        self.assertAlmostEqual(hello.exp(1), 2.71828182846)
        self.assertAlmostEqual(hello.exp(2), 7.38905609893)

    def test_sin(self):
        self.assertEqual(hello.sin(0), 0)
        self.assertEqual(hello.sin(1), 0.8414709848078965)

    def test_cos(self):
        self.assertEqual(hello.cos(0), 1)
        self.assertEqual(hello.cos(1), 0.5403023058681398)

    def test_tan(self):
        self.assertEqual(hello.tan(0), 0)
        self.assertEqual(hello.tan(1), 1.5574077246549023)

    def test_cot(self):
        self.assertEqual(hello.cot(0), float("inf"))
        self.assertEqual(hello.cot(1), 0.6420926159343306)

    def test_bank(self):
        # create object
        test = Bank("Root", 1234, 100)
        user = Bank("User", 123456, 15)

        # check withdraw method
        with self.assertRaises(test.withdraw(101)):
            ValueError("You do not have the funds to make this transaction.")
        self.assertNotEqual(test.withdraw(101), -1)
        self.assertEqual(test.withdraw(50), 50)

        # check deposit method
        self.assertEqual(test.deposit(50), 100)
        self.assertNotEqual(test.deposit(0), 0)

        @patch("sys.stdout", new_callable=io.StringIO)
        def test_depo(mock_stdout):
            test.deposit(50)
            assert (
                mock_stdout.getvalue()
                == f"You have successfully deposited {50} into your account."
            )

        # check printBal method
        @patch("sys.stdout", new_callable=io.StringIO)
        def test_printBal(mock_stdout):
            user.printBal()
            assert mock_stdout.getvalue() == f"Your current balance is {15}."


if __name__ == "__main__":
    unittest.main()
