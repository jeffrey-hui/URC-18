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
    if ans.success:
        c = confidence(rays, points)
    else:
        c = 0

    return ans, c


def confidence(rays, points):
    """
    Calculates confidence of points and rays

    :param rays: list of rays
    :param points: list of points
    :return: confidence 0 - 1
    """

    conf_scale = min(1, len(rays + points) / 4)

    return conf_scale * (sum(
        (x.conf for x in rays + points)
    ) / len(rays + points))
