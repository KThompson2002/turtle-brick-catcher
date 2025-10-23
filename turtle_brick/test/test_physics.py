"""Testing the physics module."""

import unittest

import numpy as np

from turtle_brick import physics


class TestPhysics(unittest.TestCase):
    """Testing the physics.World member functions."""

    def setUp(self):
        """Create an instance of world and it's variables."""
        self.brick = np.array([1.0, 2.0, 3.0])
        self.world = physics.World(self.brick, 9.8, 2.0, 0.1)

    def test_init(self):
        """Test that all initialization values are stored properly."""
        assert np.array_equal(self.world._brick, self.brick)
        assert self.world.vel == 0

    def test_property(self):
        """Test that the brick property returns the location."""
        assert np.array_equal(self.world.brick, self.brick)

    def test_brick_setter(self):
        """Test that the brick setter correctly changes the location."""
        assert np.array_equal(self.world._brick, self.brick)

        brick_loc_new = np.array([2.0, 3.0, 4.0])
        self.world.brick = brick_loc_new
        assert np.array_equal(self.world._brick, brick_loc_new)

    def test_drop(self):
        """Test that the brick drops at the rate of gravity."""
        initial_vel = 0.0
        initial_z = self.world.brick[2]

        self.world.drop()

        expected_vel = initial_vel + self.world.gravity * self.world.dt
        assert np.isclose(self.world.vel, expected_vel)

        expected_z = initial_z - expected_vel * self.world.dt

        assert np.isclose(self.world.brick[2], expected_z)
        assert self.world.brick[2] < initial_z


if __name__ == '__main__':
    unittest.main()
