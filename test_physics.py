import unittest
import physics

class TestPhysics(unittest.TestCase):
    def test_calc_buoyancy(self):
        self.assertAlmostEqual(physics.calculate_buoyancy(10, 10), 981)
        self.assertRaises(ValueError, lambda: physics.calculate_buoyancy(0, 0))
        self.assertNotEqual(physics.calculate_buoyancy(1, 1), 10)

    def test_will_it_float(self):
        self.assertFalse(physics.will_it_float(1, 1001))
        self.assertTrue(physics.will_it_float(1000, 1))
        self.assertEqual(physics.will_it_float(1, 1000), None)
        self.assertRaises(ValueError,lambda: physics.will_it_float(0, 0))
        self.assertRaises(ValueError,lambda: physics.will_it_float(-1, 0))
        self.assertRaises(ValueError,lambda: physics.will_it_float(0, -1))
        self.assertRaises(ValueError,lambda: physics.will_it_float(0, 1))
        self.assertRaises(ValueError,lambda: physics.will_it_float(1, 0))
        self.assertRaises(ValueError,lambda: physics.will_it_float(-1, 1))
        self.assertRaises(ValueError,lambda: physics.will_it_float(1, -1))
        self.assertRaises(ValueError,lambda: physics.will_it_float(-1, -1))

    def test_calculate_pressure(self):
        self.assertEqual(physics.calculate_pressure(0), 101.3)
        self.assertNotEqual(physics.calculate_pressure(100), 101.3)
        self.assertEqual(physics.calculate_pressure(-100), 981101.3)


if __name__ == "__main__":
    unittest.main()
