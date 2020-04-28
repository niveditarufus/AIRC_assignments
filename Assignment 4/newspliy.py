#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import scipy.interpolate as si
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
from geomdl import BSpline


def checkCollisionLineSegment(
    pta,
    ptb,
    x,
    y,
    radius,
    ):
    x_pts = np.linspace(pta[0], ptb[0], 10)
    y_pts = np.linspace(pta[1], ptb[1], 10)
    status = False
    for i in range(0, x_pts.size):
        dist = math.sqrt((x_pts[i] - x) ** 2 + (y_pts[i] - y) ** 2)
        if radius > dist:
            status = True
    return status


def checkCollisionCircle(
    x,
    y,
    radius,
    xob,
    yob,
    radiusob,
    ):
    dist = math.sqrt((x - xob) ** 2 + (y - yob) ** 2)
    if radius + radiusob < dist:
        return False
    else:
        return True


def bspline(
    cv,
    n=100,
    degree=3,
    periodic=False,
    ):
    cv = np.asarray(cv)
    count = len(cv)
    if periodic:
        (factor, fraction) = divmod(count + degree + 1, count)
        cv = np.concatenate((cv, ) * factor + (cv[:fraction], ))
        count = len(cv)
        degree = np.clip(degree, 1, degree)
    else:
        degree = np.clip(degree, 1, count - 1)
    kv = None
    if periodic:
        kv = np.arange(0 - degree, count + degree + degree - 1,
                       dtype='int')
    else:
        kv = np.concatenate(([0] * degree, np.arange(count - degree
                            + 1), [count - degree] * degree))
    u = np.linspace(periodic, count - degree, n)
    return np.array(si.splev(u, (kv, cv.T, degree))).T


def main():
    curve = BSpline.Curve()
    curve.degree = 1
    curve.ctrlpts = [[0, 0], [20, -10]]
    curve.knotvector = [0, 0, 1, 1]
    curve.delta = 0.005
    curve_points = curve.evalpts
    curve_points = np.asarray(curve_points)
    plt.plot(curve_points[:, 0], curve_points[:, 1], label = "path taken if no obtcales were present")

    line_pts = curve_points

    circle_collision_pts = []
    circle_collision_idx = []
    for i in range(0, line_pts.shape[0]):
        if checkCollisionCircle(
            line_pts[i][0],
            line_pts[i][1],
            0.5,
            5,
            -5,
            2,
            ):
            circle_collision_pts.append(line_pts[i])
            circle_collision_idx.append(i)

    poly_collision_pts = []
    poly_collision_idx = []
    for i in range(0, line_pts.shape[0]):
        if checkCollisionLineSegment([12, -3], [14, -3],
                line_pts[i][0], line_pts[i][1], 0.5) \
            or checkCollisionLineSegment([14, -3], [16, -7],
                line_pts[i][0], line_pts[i][1], 0.5) \
            or checkCollisionLineSegment([16, -7], [10, -6],
                line_pts[i][0], line_pts[i][1], 0.5) \
            or checkCollisionLineSegment([10, -6], [12, -3],
                line_pts[i][0], line_pts[i][1], 0.5):
            poly_collision_pts.append(line_pts[i])
            poly_collision_idx.append(i)

    important_indexes = [np.min(circle_collision_idx) - 1,
                         np.max(circle_collision_idx) + 1,
                         np.min(poly_collision_idx) - 1,
                         np.max(poly_collision_idx) + 1]
    print(important_indexes)

    important_points = []
    for i in important_indexes:
        important_points.append(line_pts[i])
    print(important_points)

    important_points = np.asarray(important_points)
    # plt.scatter(important_points[:, 0], important_points[:, 1])

    knots_to_add = []
    for i in important_indexes:
        knots_to_add.append(line_pts[i].tolist()[0])

    print(curve_points.shape, line_pts.shape)
    min_cost = 1000000000000000
    # for a in range(0, 21):
    #     for b in range(0, 11):
    #         for c in range(0, 21):
    #             for d in range(0, 11):
    #                 curve = BSpline.Curve()
    #                 curve.degree = 3
    #                 curve.ctrlpts = [
    #                     [4, -2],
    #                     important_points[0].tolist(),
    #                     [a, -b],
    #                     important_points[1].tolist(),
    #                     important_points[2].tolist(),
    #                     [c, -d],
    #                     important_points[3].tolist(),
    #                     [20, -10],
    #                     ]
    #                 curve.knotvector = [
    #                     0,
    #                     0,
    #                     0,
    #                     0,
    #                     knots_to_add[0] / 20,
    #                     knots_to_add[1] / 20,
    #                     knots_to_add[2] / 20,
    #                     knots_to_add[3] / 20,
    #                     1,
    #                     1,
    #                     1,
    #                     1,
    #                     ]
    #                 curve.delta = 0.005
    #                 curve_points = curve.evalpts
    #                 curve_points = np.asarray(curve_points)
    #                 dist = np.linalg.norm(curve_points - line_pts)
    #                 collission = False
    #                 for i in range(0, curve_points.shape[0]):
    #                     if checkCollisionLineSegment([10, -6], [16,
    #                             -7], curve_points[i][0],
    #                             curve_points[i][1], 0.5) \
    #                         or checkCollisionLineSegment([12, -3], [16,
    #                             -7], curve_points[i][0],
    #                             curve_points[i][1], 0.5) \
    #                         or checkCollisionCircle(
    #                         curve_points[i][0],
    #                         curve_points[i][1],
    #                         0.5,
    #                         5,
    #                         -5,
    #                         2,
    #                         ):
    #                         collission = True
    #                         break
    #                 if dist < min_cost and collission == False:
    #                     print('found a better param')
    #                     print(a, -b, c, -d, dist)
    #                     print('non colliding')
    #                     min_params = [a, -b, c, -d]
    #                     min_cost = dist

    min_params = [3, 0, 9, -10]

    print(min_params)
    curve = BSpline.Curve()
    curve.degree = 3
    curve.ctrlpts = [
        [4, -2],
        important_points[0].tolist(),
        [min_params[0], min_params[1]],
        important_points[1].tolist(),
        important_points[2].tolist(),
        [min_params[2], min_params[3]],
        important_points[3].tolist(),
        [20, -10],
        ]
    curve.knotvector = [
        0,
        0,
        0,
        0,
        knots_to_add[0] / 20,
        knots_to_add[1] / 20,
        knots_to_add[2] / 20,
        knots_to_add[3] / 20,
        1,
        1,
        1,
        1,
        ]
    curve.delta = 0.005
    curve_points = curve.evalpts
    curve_points = np.asarray(curve_points)

    plt.plot((important_points[0][0],important_points[1][0]),(important_points[0][1],important_points[1][1]), color='r', label = "segment to be modified")
    plt.plot((important_points[2][0],important_points[3][0]),(important_points[2][1],important_points[3][1]), color='y', label = "segment to be modified")
    # plt.plot(curve_points[:, 0], curve_points[:, 1])

    obstacle1 = plt.Circle((5, -5), 2, color='k')
    plt.gcf().gca().add_artist(obstacle1)

    vertices = np.array([(12, -3), (14, -3), (16, -7), (10, -6)])
    obstacle2 = Polygon(vertices, closed=True, color='k')
    plt.gcf().gca().add_artist(obstacle2)

    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
