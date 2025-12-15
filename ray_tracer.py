import numpy as np
import polytope

class Ray:
    def __init__(self, starting_point=(0, 0), slope=0):
        """
        Describes a ray in the form:
            y = mx + c
        
        :param starting_point: point of ray emergence (x, y) 
        :param slope: slope of the ray wrt +ve x-axis
        """
        # not really a starting point
        self.slope = slope
        self.update_ray(starting_point)

    def __update_starting_point(self, new_starting_point):
        self.starting_point = new_starting_point

    def __compute_c(self):
        # find c in y = mx + c
        self.c = self.starting_point[1] - self.slope*self.starting_point[0]

    def update_ray(self, new_starting_point):
        self.__update_starting_point(new_starting_point)
        self.__compute_c()

    def compute_x(self, y):
        if self.slope != 0:
            return (y - self.c)/self.slope
        return np.inf
    
    def compute_y(self, x):
        return self.slope * x + self.c


class RayTracer:
    def __init__(self, starting_point=(0, 0), num_rays_per_180degrees=60):
        self.starting_point = starting_point
        self.num_rays = num_rays_per_180degrees        
        
        self.rays = []
        self.ray_angles = np.linspace(0, np.pi, self.num_rays)
        self.slopes = np.tan(self.ray_angles)
        for slope in self.slopes:
            self.rays.append(Ray(starting_point, slope))
        

    def update_rays(self, new_starting_point):
        """
        Updates rays from a new starting point xy = (x, y) 
        
        :param new_starting_point: New point of emergence of rays (x, y)
        """
        self.starting_point = new_starting_point
        for ray in self.rays:
            ray.update_ray(new_starting_point)
        
    def intersection_distance(self, xy):
        return np.sqrt((self.starting_point[0] - xy[0])**2
                       + (self.starting_point[1] - xy[1])**2)
    
    def is_point_on_object(self, object, point, object_face_index):
        if object_face_index == 0:
            p2 = object.top_left_corner
            p1 = object.bottom_left_corner
        elif object_face_index == 1:
            p1 = object.bottom_left_corner
            p2 = object.bottom_right_corner
        elif object_face_index == 2:
            p1 = object.bottom_right_corner
            p2 = object.top_right_corner
        elif object_face_index == 3:
            p2 = object.top_right_corner
            p1 = object.top_left_corner
        else:
            return False
        
        return p1 <= point <= p2

    def compute_ray_intersections(self, objects):
        """
        Computes the intersection points between the rays and 
        given list of objects.
        
        :param objects: List of objects 
                (objects must have left_boundary, bottom_boundary,
                                    right_boundary, and top_boundary 
                                    properties)
        """
        left = []
        bottom = []
        right = []
        top = []
        for ray in self.rays:
            valid_intersections = []
            for object in objects:
                x1 = object.left_boundary
                y1 = ray.compute_y(x1)

                y2 = object.bottom_boundary
                x2 = ray.compute_x(y2)

                x3 = object.right_boundary
                y3 = ray.compute_y(x3)
                
                y4 = object.top_boundary
                x4 = ray.compute_x(y4)

                points = [(x1, y1), (x2, y2) ,(x3, y3), (x4, y4)]
                d1 = self.intersection_distance(points[0])
                d2 = self.intersection_distance(points[1])
                d3 = self.intersection_distance(points[2])
                d4 = self.intersection_distance(points[3])

                dmin = np.inf
                imin = -1
                for i, d in enumerate([d1, d2, d3, d4]):
                    if self.is_point_on_object(object, points[i], i):
                        if d < dmin:
                            dmin = d
                            imin = i
                    # print(True)
                if dmin != np.inf:
                    valid_intersections.append([dmin, points[imin]])
            # valid_intersections.sort(key=lambda intersection: intersection[0])
            intersections_to_left = []
            intersections_to_bottom = []
            intersections_to_right = []
            intersections_to_top = []
            for intersection in valid_intersections:
                if np.tan(-np.pi/12) <= ray.slope <= np.tan(np.pi/12):
                    if intersection[1][0] < ray.starting_point[0]:
                        intersections_to_left.append(intersection)
                    else:
                        intersections_to_right.append(intersection)
                elif np.tan(np.pi/2 + np.pi/12) <= ray.slope <= np.tan(np.pi/2 - np.pi/12):
                    if intersection[1][1] < ray.starting_point[1]:
                        intersections_to_bottom.append(intersection)
                    else:
                        intersections_to_top.append(intersection)

            intersections_to_left.sort(key=lambda intersection: intersection[0])
            intersections_to_bottom.sort(key=lambda intersection: intersection[0])
            intersections_to_right.sort(key=lambda intersection: intersection[0])
            intersections_to_top.sort(key=lambda intersection: intersection[0])
            if intersections_to_left != []:
                left.append(intersections_to_left[0])
            if intersections_to_bottom != []:
                bottom.append(intersections_to_bottom[0])
            if intersections_to_right != []:
                right.append(intersections_to_right[0])
            if intersections_to_top != []:
                top.append(intersections_to_top[0])
            
                # closest_intersections += valid_intersections
        left.sort(key=lambda intersection: intersection[0])
        bottom.sort(key=lambda intersection: intersection[0])
        right.sort(key=lambda intersection: intersection[0])
        top.sort(key=lambda intersection: intersection[0])
        # closest_intersections.sort(key=lambda intersection: intersection[0])
        if len(left) == 0:
            left = [[-1, (-10, 0)]]
        if len(bottom) == 0:
            bottom = [[-1, (0, -10)]]
        if len(right) == 0:
            right = [[-1, (10, 0)]]
        if len(top) == 0:
            top = [[-1, (0, 10)]]
            
        return left[0], bottom[0], right[0], top[0]
    

class ConvexSetConstructor:
    def __init__(self):
        pass

    def get_convex_set(self, rays_starting_point, closest_intersections):
        x = rays_starting_point[0]
        y = rays_starting_point[1]

        polytopes_A= []
        polytopes_b= []
        for intersection in closest_intersections:
            x1 = intersection[1][0]
            y1 = intersection[1][1]
            a = x1 - x
            b = y1 - y
            c = x1*(x1 - x) + y1*(y1 - y)
            c_xy = a*x + b*y
            if c_xy >= c:
                polytopes_A.append([-a, -b])
                polytopes_b.append([-c])
            else:
                polytopes_A.append([a, b])
                polytopes_b.append([c])


        polytopes_A = np.asarray(polytopes_A, dtype=float)
        polytopes_b = np.asarray(polytopes_b, dtype=float)

        poly = polytope.Polytope(polytopes_A, polytopes_b)
        return poly
               




