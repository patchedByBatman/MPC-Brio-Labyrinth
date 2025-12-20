import numpy as np

class SolutionPath:
    def __init__(self):
        self.num_samples_per_meter = 1
        x_start = 10
        x_end = (2 - 0.5)/2
        y_start = (7.5 + 11)/2
        y_end = (7.5 + 11)/2
        dist = x_start - x_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path = [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (5 + 9)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (-1.5 - 4)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (11 + 4.5)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (-9 - 5)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (6.5 + 5)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (-11 - 8)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (3 + 1)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (-9 - 5)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (0 -2)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (-11 - 8)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (-5.5 - 3)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (-7 -5)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (-11 -8.5)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (2 + 4)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (-1 -5.5)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (-2 +0)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (3 + 1)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (5 + 7.5)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = (-11 -9)/2
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = (8.5 + 11)/2
        y_start = y_end
        y_end = y_end
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]

        x_start = x_end
        x_end = x_end
        y_start = y_end
        y_end = 6
        pathx = np.linspace(x_start, x_end, int(dist*self.num_samples_per_meter))
        pathy = np.linspace(y_start, y_end, int(dist*self.num_samples_per_meter))
        self.path += [(x, y) for x, y in zip(pathx, pathy)]


        # self.path = [
        #     (8, 9),
        #     (5, 9),
        #     (2, 9),
        #     (0, 9),
        #     (0, 7),
        #     (-3, 7),
        #     (-3, 9),
        #     (-7, 9),
        #     (-7, 6),
        #     (-10, 5),
        #     (-10, 2.5),
        #     (-6.5, 2),
        #     (-6.5, -1),
        #     (-9.5, -1),
        #     (-9.5, -4),
        #     (-6, -4),
        #     (-6, -10),
        #     (3, -10),
        #     (3, -2.5),
        #     (-1.5, -2.5),
        #     (-1.5, 2),
        #     (6, 2),
        #     (6, -10),
        #     (9.5, -10),
        #     (9.5, 5)
        # ]


