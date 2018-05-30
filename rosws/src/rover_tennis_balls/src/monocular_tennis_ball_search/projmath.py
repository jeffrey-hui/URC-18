import numpy as np


def create_uvw_triplet(x, y, w):
    return np.array([
        x * w,
        y * w,
        w
    ])


def solve_pos(x, y, w, P):
    inv_p = np.linalg.inv(P)
    uvw = create_uvw_triplet(x, y, w)
    return np.matmul(inv_p, uvw).reshape([3])


def get_ray(x, y, P):
    p1 = solve_pos(x, y, 0.2, P)
    p2 = solve_pos(x, y, 0.8, P)
    diff = p2 - p1
    length = np.sqrt(diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2)
    diff /= length
    return diff
