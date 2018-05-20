from vector2d import Vector2D
from vector2d import Point2D

class Obstacle(object):

    def __init__(self, world, x, y, radius):


        #keep a reference to the world object
        self.world = world

        self.x = x
        self.y = y
        self.radius = radius
        self.pos = Vector2D(x,y)
