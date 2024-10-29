# test_world.py
import pytest
from turtle_brick.physics import World  # Replace 'your_module' with the actual module name


class TestWorld:
    """Tests for the World class."""

    def test_initialization(self):
        """Test that the World class initializes correctly."""
        initial_brick_position = (1.0, 2.0, 3.0) 
        gravity = 9.8 
        radius = 5.0  
        dt = 0.1 

        world = World(initial_brick_position, gravity, radius, dt)

        assert world.brick == initial_brick_position
        assert world.gravity == gravity
        assert world.radius == radius
        assert world.dt == dt
        assert world.velocity == 0.0  

    def test_brick_drop(self):
        """Test the drop method updates the brick position."""
        initial_brick_position = (1.0, 2.0, 5.0) 
        gravity = 9.8 
        radius = 5.0  
        dt = 0.1  

        world = World(initial_brick_position, gravity, radius, dt)

        world.drop()
        
        expected_new_z = initial_brick_position[2] - (world.velocity * dt + 0.5 * gravity * dt**2)
        
        assert world.brick[0] == initial_brick_position[0]
        assert world.brick[1] == initial_brick_position[1] 
        assert world.brick[2] == pytest.approx(expected_new_z)
        assert world.velocity > 0.0 


if __name__ == "__main__":
    pytest.main()
