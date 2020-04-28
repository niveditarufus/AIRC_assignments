import numpy as np
import scipy.interpolate as si
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
from geomdl import BSpline


def bspline(cv, n=100, degree=3, periodic=False):
    cv = np.asarray(cv)
    count = len(cv)
    if periodic:
        factor, fraction = divmod(count+degree+1, count)
        cv = np.concatenate((cv,) * factor + (cv[:fraction],))
        count = len(cv)
        degree = np.clip(degree,1,degree)
    else:
        degree = np.clip(degree,1,count-1)
    kv = None
    if periodic:
        kv = np.arange(0-degree,count+degree+degree-1,dtype='int')
    else:
        kv = np.concatenate(([0]*degree, np.arange(count-degree+1), [count-degree]*degree))
    u = np.linspace(periodic,(count-degree),n)
    return np.array(si.splev(u, (kv,cv.T,degree))).T

def get_segments(cir_pts, cir_idx, trapezium_pts, trapezium_idx, pts):
    segment_idx = [np.min(cir_idx)-2, np.max(cir_idx)+2, np.min(trapezium_idx)-2, np.max(trapezium_idx)+2]
    seg_points = []
    for i in segment_idx:
        seg_points.append(pts[i])
    seg_points = np.asarray(seg_points)
    return seg_points, segment_idx


def CollisionTrapezium(a, b, x, y, radius):
    x_pts = np.linspace(a[0], b[0], 10)
    y_pts = np.linspace(a[1], b[1], 10)
    c = False
    for i in range(0,x_pts.size):
        dist = np.hypot((x_pts[i]-x), (y_pts[i]-y))
        if radius > dist:
            c = True
    return c

def CollisionCircle(x, y, radius, center_x, center_y, radiusob):
    dist = np.hypot((x-center_x),(y-center_y))
    t = radius + radiusob
    if  t < dist:
        return False
    else:
        return True

def generate_bspline(degree, control_points, knots, delta = 0.005):
    curve = BSpline.Curve()
    curve.degree = degree
    curve.ctrlpts = control_points
    curve.knotvector = knots
    curve.delta = delta
    curve_points = curve.evalpts
    curve_points = np.asarray(curve_points)
    return curve, curve_points

def get_points_with_circle(pts, radius, x, y, ob_radius):
    cir_pts = []
    cir_idx = []
    for i in range(0, len(pts)):
        if(CollisionCircle(pts[i][0], pts[i][1], radius, x, y, ob_radius)):
            cir_pts.append(pts[i])
            cir_idx.append(i)
    return cir_pts, cir_idx

def get_points_with_trapezium(pts):
    trapezium_pts = []
    trapezium_idx = []
    for i in range(0,len(pts)):
        if(CollisionTrapezium([12,-3], [14,-3],pts[i][0],pts[i][1],0.5) or CollisionTrapezium([14,-3], [16,-7], pts[i][0],pts[i][1],0.5) or CollisionTrapezium([16,-7],[10,-6], pts[i][0],pts[i][1],0.5) or CollisionTrapezium([10,-6],[12,-3],pts[i][0],pts[i][1],0.5)):
            trapezium_pts.append(pts[i])
            trapezium_idx.append(i)
    return trapezium_pts, trapezium_idx


def get_knots(segment_idx, pts):
    new_knots = []
    for i in segment_idx:
        new_knots.append(pts[i].tolist()[0])
    return new_knots

def get_optimal_trajectory(seg_points, new_knots, pts):

    length = np.linspace(0,11, 15)
    breadth = np.linspace(0,11, 15)
    min_cost = 1000000000000
    for x1 in length:
        for y1 in breadth:
            for x2 in length:
                for y2 in breadth:
                    print(x1,y1,x2,y2)
                    control_points = [[3.5,-1.6], seg_points[0].tolist(), [x1,-y1], seg_points[1].tolist(), seg_points[2].tolist(), [x2,-y2], seg_points[3].tolist(), [20,-10]]
                    knots = [0, 0, 0, 0, (new_knots[0]/20), (new_knots[1]/20), (new_knots[2]/20), (new_knots[3]/20), 1, 1, 1, 1]
                    curve, curve_points = generate_bspline(3, control_points,knots)
                    dist = np.linalg.norm(curve_points-pts)
                    c = False
                    for i in range(0,curve_points.shape[0]):
                        if(CollisionTrapezium([10,-6], [16,-7],curve_points[i][0],curve_points[i][1],0.5) or CollisionTrapezium([12,-3],[16,-7],curve_points[i][0],curve_points[i][1],0.5) or CollisionCircle(curve_points[i][0], curve_points[i][1], 0.5, 5, -5, 2)):
                            c = True
                            break
                    if dist<min_cost and c==False:
                        min_params = [x1,-y1, x2,-y2]
                        min_cost = dist
    return min_params, min_cost

def main():

    circle_radius = 2
    circle_x = 5
    circle_y = -5
    robot_radius = 0.5
    curve, pts = generate_bspline(1, [[0,0], [20,-10]], [0,0,1,1])
    plt.plot(pts[:,0], pts[:,1], label = "path taken if no obtcales were present")
    
    circle_collision_pts, circle_collision_idx = get_points_with_circle(pts, robot_radius, circle_x, circle_y, circle_radius)
    trap_collision_pts, trap_collision_idx = get_points_with_trapezium(pts)
    seg_points, seg_idx = get_segments(circle_collision_pts, circle_collision_idx, trap_collision_pts, trap_collision_idx, pts)

    plt.plot((seg_points[0][0],seg_points[1][0]),(seg_points[0][1],seg_points[1][1]), color='r', label = "segment to be modified")
    plt.plot((seg_points[2][0],seg_points[3][0]),(seg_points[2][1],seg_points[3][1]), color='y', label = "segment to be modified")

    new_knots = get_knots(seg_idx, pts)

    min_params, min_cost = get_optimal_trajectory(seg_points, new_knots, pts)
    print(min_params)

    control_points = [[0,0], seg_points[0].tolist(), [min_params[0], min_params[1]], seg_points[1].tolist(), seg_points[2].tolist(), [min_params[2], min_params[3]], seg_points[3].tolist(), [20,-10]]
    knots = [0, 0, 0, 0, (new_knots[0]/20), (new_knots[1]/20), (new_knots[2]/20), (new_knots[3]/20), 1, 1, 1, 1]
    curve, curve_points = generate_bspline(3, control_points, knots)
    plt.plot(curve_points[:,0], curve_points[:,1], label = "modified path")

    obstacle1 = plt.Circle((circle_x, circle_y), circle_radius, color='k')
    plt.gcf().gca().add_artist(obstacle1)

    vertices = np.array([(12,-3),(14,-3),(16,-7),(10,-6)])
    obstacle2 = Polygon(vertices, closed=True, color='k')
    plt.gcf().gca().add_artist(obstacle2)

    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()