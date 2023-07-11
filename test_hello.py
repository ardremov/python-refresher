import unittest
import hello


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


if __name__ == "__main__":
    unittest.main()
