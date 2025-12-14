import numpy as np

class Wall:
    def __init__(self, xy, width, height):
        """
        Data class for wall generation.
        Wall looks as follows:
            
                    _____________  __
                    |           |   |
                    |           |   |
                    |           |   h
                    |           |   |
         base point x___________|  _|
                    |____ w ____|
            
        :param xy: base point (x, y)
        :param width: width of the wall
        :param height: height of the wall
        """
        self.base_point = list(xy)
        self.width = width
        self.height = height
        if width < 0:
            self.base_point[0] += width
            self.width = -width
        if height < 0:
            self.base_point[1] += height
            self.height = -height

        self.left_boundary = self.base_point[0]  # eq is x <= xbase
        self.bottom_boundary = self.base_point[1]  # y <= ybase
        # x >= xbase + width
        self.right_boundary = self.left_boundary + self.width
        # y >= ybase + height
        self.top_boundary = self.bottom_boundary + self.height

        self.wall_centre = (self.base_point[0] + self.width/2, 
                            self.base_point[1] + self.height/2)
        self.bottom_left_corner = tuple(self.base_point)
        self.bottom_right_corner = (self.right_boundary, self.bottom_boundary)
        self.top_right_corner = (self.right_boundary, self.top_boundary)
        self.top_left_corner = (self.left_boundary, self.top_boundary)

class Hole(Wall):
    # Here we have the programmer's version of a spherical cow.
    # Behold the Square-ical HOL-E-cow.
    def __init__(self, xy, r):
        """
        Data class for Hole generation.
        The described hole will be approximated to a bounding wall
        around the hole.
        
        :param xy: Centre of the hole (x, y)
        :param r: radius if the hole
        """
        self.centre = xy
        self.radius = r

        # approximate the hole to a bounding wall
        base_point = (self.centre[0] - r, self.centre[1] - r)
        width = 2*r
        height = 2*r
        super().__init__(base_point, width, height)

class Labyrinth:
    def __init__(self):
        self.walls = []
        self.holes = []

    def add_wall(self, xy, width, height):
        self.walls.append(Wall(xy, width, height))

    def add_hole(self, xy, r):
        self.holes.append(Hole(xy, r))

    def build_labyrinth(self):
        self.add_wall((-11, 11), 22, 1)
        self.add_wall((-11, -11), -1, 22)
        self.add_wall((-11, -11), 22, -1)
        self.add_wall((11, 11), 22, -1)
        self.add_wall((10, 7.5), 1, 3.5)
        self.add_wall((2, 6.5), 9, 1)
        self.add_wall((2, 6.5), 1, -3)
        self.add_wall((-1.5, 9), 1, 2)
        self.add_wall((-5, 5), 1, 2)
        self.add_wall((-11, 6.5), 2, 1)
        self.add_hole((-10, 8.5), 1)
        self.add_hole((-3, 4), 1)
        self.add_hole((1, 4), 1)
        self.add_wall((-1.5, 3), 1, 2)
        self.add_hole((-6, 4), 1)
        self.add_wall((-8, 3.5), 1, 1)
        self.add_wall((-11, 0), 2, 1)
        self.add_wall((-5, 3), 1, -6)
        self.add_wall((-5, -3), -3, 1)
        self.add_wall((-8, -5.5), 1, -2)
        self.add_wall((-5, -5.5), 1, -2)
        self.add_hole((-10, -6.5), 1)
        self.add_wall((-2, -5.5), 1, -3)
        self.add_wall((1, -5.5), 1, -3)
        self.add_hole((-3, -4), 1)
        self.add_hole((0, -7), 1)
        self.add_wall((5, 0), -1, -11.5)
        self.add_wall((0, 0.5), 1, -1.5)
        self.add_hole((2.5, 0), 1)
        self.add_wall((7.5, 6.5), 1, -15.5)
        self.add_wall((7.5, 6.5), 1, -15.5)
        self.add_wall((7.5, 6.5), 1, -15.5)





if __name__ == "__main__":
    lb = Labyrinth()
    lb.add_wall((0, 0), 2, 2)
    lb.add_hole((4, 4), 1)
    lb.add_hole((5, 5), 1)
    print(lb.walls)
    print(lb.holes[0].base_point)
