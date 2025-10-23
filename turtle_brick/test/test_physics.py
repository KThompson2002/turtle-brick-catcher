from turtle_brick import physics
import numpy as np


def test_init():
    """Test that all initialization values are stored properly."""
    brick = np.array([1.0, 2.0, 3.0])
    gravity = 9.81
    radius = 5.0
    dt = 0.01

    world = physics.World(brick, gravity, radius, dt)

    assert np.array_equal(world._brick, brick)
    assert world.gravity == gravity
    assert world.radius == radius
    assert world.dt == dt
    assert world.vel == 0 


def test_property():
    """Test that the brick property returns the location"""
    brick_loc = np.array([1.0, 2.0, 3.0])
    world = physics.World(brick_loc, 9.8, 2.0, 0.1)
    
    assert np.array_equal(world.brick, brick_loc)
    
    
def test_brick_setter():
    """Test that the brick setter correctly changes the location"""
    brick_loc = np.array([1.0, 2.0, 3.0])
    world = physics.World(brick_loc, 9.8, 2.0, 0.1)
    
    assert np.array_equal(world._brick, brick_loc)
    
    brick_loc_new = np.array([2.0, 3.0, 4.0])
    world.brick = brick_loc_new
    assert np.array_equal(world._brick, brick_loc_new)
    
    
def test_drop():
    """Test that the brick drops at the rate of gravity"""
    brick_loc = np.array([1.0, 2.0, 3.0])
    world = physics.World(brick_loc, 9.8, 2.0, 0.1)
    initial_vel = 0.0
    initial_z = world.brick[2]
    
    world.drop()
    
    expected_vel = initial_vel + world.gravity * world.dt
    assert np.isclose(world.vel, expected_vel)
    
    expected_z = initial_z - expected_vel * world.dt

    assert np.isclose(world.brick[2], expected_z)
    assert world.brick[2] < initial_z
    