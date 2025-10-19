import numpy as np

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
        self.vel = 0
        pass

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
        self._brick = location.copy()
    def drop(self):
        """
        Update the brick's location by having it fall in gravity for one timestep
        """
        self.vel += self.gravity*self.dt
        self.brick[2] -= self.vel*self.dt 
        pass
    
    def tilt(self, angle):
        a = self.gravity*np.sin(angle)
        
        self.vel += a * self.dt
        s = self.vel * self.dt
        dy = s * np.cos(angle)
        dz = s * np.sin(angle)
        
        self.brick[1] -= dy
        self.brick[2] -= dz