import numpy as np

def trilaterate_two_anchors(base1, base2):
    """
    Find the two possible positions of the rover given the two basestations.
    
    Parameters:
        base1: base_station class object, the first basestation.
        base2: base_station class object, the second basestation.
    
    Returns:
        A tuple of two possible (x, y) points
    """

    # Distance between anchor points
    D = np.hypot(base2.x - base1.x, base2.y - base1.y)

    #Check to see if a distance has been read yet
    if (base1.dist == -1) or (base2.dist == -1):
        return ValueError("No distance has been read yet, cannot triliterate.")

    # Check for no intersection
    if (D > base1.dist + base2.dist) or (D < abs(base1.dist - base2.dist)) or (D == 0):
        raise ValueError("No intersection or infinite solutions (bad input).")

    # Point along the line between p1 and p2
    a = (base1.dist**2 - base2.dist**2 + D**2) / (2 * D)
    h = np.sqrt(base1.dist**2 - a**2)

    # Coordinates of point 2 (midpoint between intersections)
    x3 = base1.x + a * (base2.x - base1.x) / D
    y3 = base1.y + a * (base2.y - base1.y) / D

    # Offsets for the two intersection points
    rx = -(base2.y - base1.y) * (h / D)
    ry =  (base2.x - base1.x) * (h / D)

    # The two possible intersection points
    sol1 = (x3 + rx, y3 + ry)
    sol2 = (x3 - rx, y3 - ry)

    return sol1, sol2
