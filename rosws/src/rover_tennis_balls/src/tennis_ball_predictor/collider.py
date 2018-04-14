import numpy as np
from scipy.optimize import least_squares

import itertools


class Ray:
    def __init__(self, pos, vec, conf):
        self.pos = np.array(pos)
        self.vec = np.array(vec)
        self.conf = conf


class Point:
    def __init__(self, pos, conf):
        self.pos = np.array(pos)
        self.conf = conf


def f(pt, rays, points):
    errors = np.full([(len(rays) + len(points)) * 3], np.inf)
    for i in range(0, len(rays)):
        ray = rays[i]  # type: Ray
        time_on_ray = ray.vec.dot((pt - ray.pos).T)  # calculate t in p = ray.pos * ray.dir * t
        if time_on_ray > 0:  # if point is on ray...
            errors[i*3:(i+1)*3] = (pt - (ray.pos + time_on_ray * ray.vec)).T * ray.conf # do distance to nearest point along ray
        else:
            errors[i * 3:(i + 1) * 3] = (pt - ray.pos).T * ray.conf # else distance to origin of ray (farthest point back)
    k = len(rays) * 3
    for i in range(0, len(points)):
        point = points[i]
        errors[k + i * 3:k + (i+1) * 3] = (pt - point.pos) * point.conf  # for each axis of point, use distance * conf

    return errors.ravel()  # scipy wants it to be column-wise


def calc_min_f(rays, points):
    """
    Tries to find the closest point to a bunch of rays and points

    :param rays: list of rays
    :param points: list of points
    :return: nearest point, confidence
    """
    rays_p = [ray.pos for ray in rays] + [point.pos for point in points]
    guess = np.stack(rays_p).mean(axis=0).ravel()

    ans = least_squares(f, guess, args=(rays, points))
    c = confidence(rays, points)

    return ans, c


def confidence(rays, points):
    """
    Calculates confidence of points and rays

    :param rays: list of rays
    :param points: list of points
    :return: confidence 0 - 1
    """

    d = 0
    div_mes = 0
    if len(rays) > 1:
        div_mes = []
        for i in itertools.combinations(rays, 2):  # for all combinations of rays..
            dot = i[0].vec.dot(i[1].vec) # take the dot product (rays are normalized so this is cos(theta))
            diversity = abs(np.arccos(dot) / np.pi) * ((i[0].conf+i[1].conf) / 2)
            # arccos = angle between vectors if vectors have length 1
            # / pi scales to -1,1
            # average confidence scales result, result is now "diversity" of angles that can be reached by these rays
            div_mes.append(diversity)
        m = max(div_mes) * len(div_mes) # normalize & divide by length
        div_mes = sum(map(lambda x: x / float(m), div_mes)) # sum is now high if large diversity, but won't explode
    if len(points) > 1:
        f_p = points[0].pos
        d = 0
        for i in points[1:]:
            i = i.pos
            d += np.sqrt(
                (f_p[0] - i[0])**2 + (f_p[1] - i[1])**2 + (f_p[2] - i[2]) ** 2  # take total distance of point n to point 0 for all n
            )
        d /= len(points)-1
        d = 1 - max(0, min(1, d))
    if len(points) == 1:
        d = 0.5
    return (d * 0.3 + min(1, div_mes) * 0.3) + (1 - (1 / float((len(rays) + len(points)) + 1))) * 0.4 # magic!