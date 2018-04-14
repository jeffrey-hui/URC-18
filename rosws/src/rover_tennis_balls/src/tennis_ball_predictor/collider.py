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
    c = confidence(rays, points)

    return ans, c


def confidence(rays, points):
    d = 0
    div_mes = 0
    if len(rays) > 1:
        div_mes = []
        for i in itertools.combinations(rays, 2):
            dot = i[0].vec.dot(i[1].vec)
            diversity = (1 - dot) * ((i[0].conf+i[1].conf) / 2)
            div_mes.append(diversity)
        m = max(div_mes) * len(div_mes)
        div_mes = sum(map(lambda x: x / float(m), div_mes))
    if len(points) > 1:
        f_p = points[0].pos
        d = 0
        for i in points[1:]:
            i = i.pos
            d += np.sqrt(
                (f_p[0] - i[0])**2 + (f_p[1] - i[1])**2 + (f_p[2] - i[2]) ** 2
            )
        d /= len(points)-1
        d = 1 - max(0, min(1, d))
    if len(points) == 1:
        d = 0.5
    return (d * 0.3 + min(1, div_mes) * 0.3) + (1 - (1 / float((len(rays) + len(points)) + 1))) * 0.4