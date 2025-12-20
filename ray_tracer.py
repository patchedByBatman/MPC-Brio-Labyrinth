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
    def __init__(self, starting_point=(0, 0), num_rays_per_180degrees=30):
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
                if np.tan(-np.pi/2) <= ray.slope <= np.tan(np.pi/2):
                    if intersection[1][0] < ray.starting_point[0]:
                        intersections_to_left.append(intersection)
                    else:
                        intersections_to_right.append(intersection)
                elif np.tan(0) <= ray.slope <= np.tan(180):
                    if intersection[1][1] < ray.starting_point[1]:
                        intersections_to_bottom.append(intersection)
                    else:
                        intersections_to_top.append(intersection)

            intersections_to_left.sort(key=lambda intersection: intersection[0])
            intersections_to_bottom.sort(key=lambda intersection: intersection[0])
            intersections_to_right.sort(key=lambda intersection: intersection[0])
            intersections_to_top.sort(key=lambda intersection: intersection[0])
            if intersections_to_left != []:
                left.append(intersections_to_left[0] + [ray.slope])
            if intersections_to_bottom != []:
                bottom.append(intersections_to_bottom[0] + [ray.slope])
            if intersections_to_right != []:
                right.append(intersections_to_right[0] + [ray.slope])
            if intersections_to_top != []:
                top.append(intersections_to_top[0] + [ray.slope])
            
                # closest_intersections += valid_intersections
        left.sort(key=lambda intersection: intersection[0])
        bottom.sort(key=lambda intersection: intersection[0])
        right.sort(key=lambda intersection: intersection[0])
        top.sort(key=lambda intersection: intersection[0])

        points = left + bottom + right + top

        points.sort(key=lambda point : point[2])
        return points
        for point in points:
            if point[1][0] < self.starting_point[0]:
                left = [point]
                break
        for point in points:
            if self.starting_point[0] < point[1][0]:
                right = [point]
                break
        # print(left)

        points.sort(key=lambda point : np.abs(point[-1])/point[0]**2*np.abs(ray.starting_point[0] - point[1][0]))
        for point in points[::-1]:
            if point[1][1] < self.starting_point[1]:
                bottom = [point]
                break

        for point in points[::-1]:
            if self.starting_point[1] < point[1][1]:
                top = [point]
                break
        
        # print(left)
        # closest_intersections.sort(key=lambda intersection: intersection[0])
        # if len(left) == 0:
        #     left = [[-1, (self.starting_point[0] - 1, 0)]]
        # if len(bottom) == 0:
        #     bottom = [[-1, (0, self.starting_point[1] - 1)]]
        # if len(right) == 0:
        #     right = [[-1, (self.starting_point[0] + 1, 0)]]
        # if len(top) == 0:
        #     top = [[-1, (0, self.starting_point[1] + 1)]]
            
        # return left[0], bottom[0], right[0], top[0]
        return left + bottom + right + top
    

class ConvexSetConstructor:
    def __init__(self):
        pass

    def get_convex_set_old(self, rays_starting_point, closest_intersections):
        x = rays_starting_point[0]
        y = rays_starting_point[1]

        polytopes_A= []
        polytopes_b= []
        left, bottom, right, top = closest_intersections
        
        polytopes_A.append([-1, 0])
        polytopes_b.append([-left[1][0]])

        polytopes_A.append([0, -1])
        polytopes_b.append([-bottom[1][1]])

        polytopes_A.append([1, 0])
        polytopes_b.append([right[1][0]])

        polytopes_A.append([0, 1])
        polytopes_b.append([top[1][1]])

        polytopes_A = np.asarray(polytopes_A, dtype=float)
        polytopes_b = np.asarray(polytopes_b, dtype=float)

        poly = polytope.Polytope(polytopes_A, polytopes_b)
        return poly
    
    def get_convex_set(self, rays_starting_point, closest_intersections):
        x = rays_starting_point[0]
        y = rays_starting_point[1]

        x_up = x
        x_low = x
        y_up = y
        y_low = y

        is_x_low_limit_reached = False
        is_x_up_limit_reached = False
        is_y_low_limit_reached = False
        is_y_up_limit_reached = False

        for dxy in np.linspace(0, 10, 1000):
            if not is_x_up_limit_reached:
                x_up += dxy
            if not is_x_low_limit_reached:
                x_low -= dxy
            if not is_y_up_limit_reached:
                y_up += dxy
            if not is_y_low_limit_reached:
                y_low -= dxy

            poly = polytope.Polytope.from_box([[x_low, x_up], [y_low, y_up]])
            point_in_poly = False
            for point in closest_intersections:
                if point[1] in poly:  
                    x_up -= dxy
                    x_low += dxy
                    y_up -= dxy
                    y_low += dxy
                    poly = polytope.Polytope.from_box([[x_low, x_up], [y_low, y_up]])
                    if point[1] not in poly:  
                        x_up += dxy

                    if not is_x_up_limit_reached:
                        x_up -= dxy
                        poly = polytope.Polytope.from_box([[x_low, x_up], [y_low, y_up]])
                        if point[1] in poly:
                            x_up += dxy
                        else:
                            is_x_up_limit_reached = True

                    if not is_x_low_limit_reached:
                        x_low += dxy
                        poly = polytope.Polytope.from_box([[x_low, x_up], [y_low, y_up]])
                        if point[1] in poly:
                            x_low -= dxy
                        else:
                            is_x_low_limit_reached = True
                    
                    if not is_y_up_limit_reached:
                        y_up -= dxy
                        poly = polytope.Polytope.from_box([[x_low, x_up], [y_low, y_up]])
                        if point[1] in poly:
                            y_up += dxy
                        else:
                            is_y_up_limit_reached = True

                    if not is_y_low_limit_reached:
                        y_low += dxy
                        poly = polytope.Polytope.from_box([[x_low, x_up], [y_low, y_up]])
                        if point[1] in poly:
                            y_low -= dxy
                        else:
                            is_y_low_limit_reached = True
                    
                    point_in_poly = True
                    break

            if point_in_poly:
                x_up -= dxy
                x_low += dxy
                y_up -= dxy
                y_low += dxy
                break

        # for dx in np.linspace(0, 10, 1000):
        #     x_low -= dx
        #     poly = polytope.Polytope.from_box([[x_low, x_up], [-12, 12]])
        #     point_in_poly = False
        #     for point in closest_intersections:
        #         px, py = point[1]
        #         if (y-y_lim <= py <= y+y_lim) and point[1] in poly:
        #             point_in_poly = True
        #             break

        #     if point_in_poly:
        #         x_low += dx
        #         break

        # for dy in np.linspace(0, 10, 1000):
        #     y_up += dy
        #     poly = polytope.Polytope.from_box([[-12, 12], [y_low, y_up]])
        #     point_in_poly = False
        #     for point in closest_intersections:
        #         px, py = point[1]
        #         if (x-x_lim <= px <= x+x_lim) and point[1] in poly:
        #             point_in_poly = True
        #             break

        #     if point_in_poly:
        #         y_up -= dy
        #         break

        # for dy in np.linspace(0, 10, 1000):
        #     y_low -= dy
        #     poly = polytope.Polytope.from_box([[-12, 12], [y_low, y_up]])
        #     point_in_poly = False
        #     for point in closest_intersections:
        #         px, py = point[1]
        #         if (x-x_lim <= px <= x+x_lim) and point[1] in poly:
        #             point_in_poly = True
        #             break

        #     if point_in_poly:
        #         y_low += dy
        #         break
        
        poly1 = polytope.Polytope.from_box([[x_low, x_up], [-12, 12]])
        poly2 = polytope.Polytope.from_box([[-12, 12], [y_low, y_up]])
        return poly1.intersect(poly2)

            # if not is_in:
            #     return poly




                      

        
        # polytopes_A.append([-1, 0])
        # polytopes_b.append([-left[1][0]])

        # polytopes_A.append([0, -1])
        # polytopes_b.append([-bottom[1][1]])

        # polytopes_A.append([1, 0])
        # polytopes_b.append([right[1][0]])

        # polytopes_A.append([0, 1])
        # polytopes_b.append([top[1][1]])

        # polytopes_A = np.asarray(polytopes_A, dtype=float)
        # polytopes_b = np.asarray(polytopes_b, dtype=float)

        # poly = polytope.Polytope(polytopes_A, polytopes_b)
        # return poly
               
if __name__ == "__main__":
    intersections = [[1, (-5,5), 2], [2, (1, 1), 3]]
    csc = ConvexSetConstructor()
    poly = csc.get_convex_set((0, 0), intersections)
    from polytope_helper import get_patch
    poly_patch = get_patch(poly)
    from matplotlib import pyplot as plt
    ax = plt.gca()
    ax.add_patch(poly_patch)
    polytope.box2poly
    plt.show()



