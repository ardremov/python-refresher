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

    def test_calculate_acceleration(self):
        self.assertEqual(physics.calculate_acceleration(4, 2), 2)
        self.assertRaises(ValueError,lambda: physics.calculate_acceleration(5, 0))
        self.assertEqual(physics.calculate_acceleration(6, 2), 3)

    def test_calculate_angular_acceleration(self):
        self.assertEqual(physics.calculate_angular_acceleration(5, 1), 5)
        self.assertRaises(ValueError,lambda: physics.calculate_angular_acceleration(5, 0))

    def test_calculate_torque(self):
        self.assertAlmostEqual(physics.calculate_torque(1, 1.5708, 1), 1)

    def test_calculate_moment_of_inertia(self):
        self.assertEqual(physics.calculate_moment_of_inertia(1, 1), 1)
        self.assertEqual(physics.calculate_moment_of_inertia(3, 4), 48)
        self.assertEqual(physics.calculate_moment_of_inertia(0, 0), 0)

    def test_calculate_auv_acceleration(self):
        self.assertRaises(ValueError,lambda: physics.calculate_auv_acceleration(0, 0, 0))

    def test_calculate_auv_angular_acceleration(self):
        self.assertRaises(ValueError,lambda: physics.calculate_auv_angular_acceleration(0, 0, 0))
    
    def test_calculate_auv2_acceleration(self):
        self.assertRaises(ValueError,lambda: physics.calculate_auv2_acceleration(0, 0, 0, 0))

    def test_calculate_auv2_angular_acceleration(self):
        self.assertRaises(ValueError,lambda: physics.calculate_auv2_angular_acceleration(0, 0, 0, 0))

if __name__ == "__main__":
    unittest.main()
