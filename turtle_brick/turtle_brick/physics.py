class World:
    """Keep track of the physics of the world."""

    def __init__(self, brick, gravity, radius, dt):
        """
        Initialize the world.

        Args:
        brick - The (x,y,z) location of the brick
        gravity - the acceleration due to gravity in m/s^2
        radius - the radius of the platform
        dt - timestep in seconds of the physics simulation
        """
        self._brick = brick
        self.gravity = gravity
        self.radius = radius
        self.dt = dt
        self.velocity = 0.0 #starts at 0 because brick is not moving

    @property
    def brick(self):
        """
        Get the brick's location.

        Return:
            (x,y,z) location of the brick
        """
        return self._brick

    @brick.setter
    def brick(self, location):
        """
        Set the brick's location.

        Args:
           location - the (x,y,z) location of the brick
        """
        self._brick = location

    def drop(self):
        """
        Update the brick's location by having it fall in gravity for one timestep
        """
        
        #Find bricks velocity
        self.velocity += self.gravity * self.dt
        
        #Use kinematic equations to get the new location of the brick. ONLY z direction
        fall = (self.velocity * self.dt) + (0.5 * self.gravity * self.dt**2)
        self.brick = (self._brick[0], self._brick[1], self._brick[2] - fall)
        
        
        