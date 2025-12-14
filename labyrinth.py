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
        self.base_point = xy
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
        self.bottom_left_corner = self.base_point
        self.bottom_right_corner = (self.right_boundary, self.bottom_boundary)
        self.top_right_corner = (self.right_boundary, self.top_boundary)
        self.top_left_corner = (self.left_boundary, self.top_boundary)

class Hole(Wall):
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
        width = r
        height = r
        super().__init__(base_point, width, height)

class Labyrinth:
    def __init__(self):
        self.walls = []
        self.holes = []

    def add_wall(self, xy, width, height):
        self.walls.append(Wall(xy, width, height))

    def add_hole(self, xy, r):
        self.holes.append(Hole(xy, r))




if __name__ == "__main__":
    lb = Labyrinth()
    lb.add_wall((0, 0), 2, 2)
    lb.add_hole((4, 4), 1)
    lb.add_hole((5, 5), 1)
    print(lb.walls)
    print(lb.holes[0].base_point)
