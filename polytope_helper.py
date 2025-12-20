import numpy as np
from matplotlib.patches import Polygon, Circle
from polytope import extreme, cheby_ball

# This is an internal helper function taken from polytope library Polytope class
def get_patch(poly1, **kwargs):
    """Return matplotlib patch for given Polytope.

    Example::

    > # Plot Polytope objects poly1 and poly2 in the same plot
    > import matplotlib.pyplot as plt
    > fig = plt.figure()
    > ax = fig.add_subplot(111)
    > p1 = _get_patch(poly1, color="blue")
    > p2 = _get_patch(poly2, color="yellow")
    > ax.add_patch(p1)
    > ax.add_patch(p2)
    > ax.set_xlim(xl, xu) # Optional: set axis max/min
    > ax.set_ylim(yl, yu)
    > plt.show()

    @type poly1: L{Polytope}
    @param kwargs: any keyword arguments valid for
        matplotlib.patches.Polygon
    """
    V = extreme(poly1)
    rc, xc = cheby_ball(poly1)
    if V is not None:
        x = V[:, 1] - xc[1]
        y = V[:, 0] - xc[0]
        mult = np.sqrt(x**2 + y**2)
        x = x / mult
        angle = np.arccos(x)
        corr = np.ones(y.size) - 2 * (y < 0)
        angle = angle * corr
        ind = np.argsort(angle)
        # create patch
        patch = Polygon(
            V[ind, :],
            closed=True,
            **kwargs)
        patch.set_zorder(0)
        return patch
    return Circle((0, 0), 2, fc="green")
