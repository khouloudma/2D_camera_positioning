import math


def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1
    x3 = 0
    y3 = 0
    x4 = 0
    y4 = 0
    d = 0.001 + math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    # print("d = ",d)
    a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
    # print("a = ",a)
    if (r0 ** 2 - a ** 2) <= 0:
        h = math.sqrt(5 + abs(r0 ** 2 - a ** 2 + 1) + r0 ** 2 - a ** 2)
    else:
        h = math.sqrt(r0 ** 2 - a ** 2)

    # print("h = ",h)

    x2 = x0 + a * (x1 - x0) / d
    # print("x2 = ",x2)

    y2 = y0 + a * (y1 - y0) / d
    # print("y2 = ",y2)
    x3 = x2 + h * (y1 - y0) / d
    # print("x3 = ",x3)
    y3 = y2 - h * (x1 - x0) / d
    x4 = x2 - h * (y1 - y0) / d
    y4 = y2 + h * (x1 - x0) / d

    return (x3, y3, x4, y4)
