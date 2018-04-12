import numpy as np
from scipy.optimize import least_squares


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
        time_on_ray = ray.vec.dot((pt - ray.pos).T)
        if time_on_ray > 0:
            errors[i*3:(i+1)*3] = (pt - (ray.pos + time_on_ray * ray.vec)).T * ray.conf
        else:
            errors[i * 3:(i + 1) * 3] = (pt - ray.pos).T * ray.conf
    k = len(rays) * 3
    for i in range(0, len(points)):
        point = points[i]
        errors[k + i * 3:k + (i+1) * 3] = (pt - point.pos) * point.conf

    return errors.ravel()


def calc_min_f(rays, points):
    rays_p = [ray.pos for ray in rays] + [point.pos for point in points]
    guess = np.stack(rays_p).mean(axis=0).ravel()

    ans = least_squares(f, guess, args=(rays, points))

    return ans
