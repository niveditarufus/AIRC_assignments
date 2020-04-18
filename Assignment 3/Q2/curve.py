import matplotlib.pyplot as plt
import numpy as np
import scipy.special

fig, ax = plt.subplots()

def bezier_path(control_points, n_points=100):
    traj = []
    n = len(control_points) - 1
    time_steps = np.linspace(0, 1, n_points)

    for t in time_steps:
        temp = np.sum([bernstein(n, i, t) * control_points[i] for i in range(0, n + 1)], axis=0)
        traj.append(temp)
    traj = np.array(traj)

    return traj


def bernstein(n, i, t):
    return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


def subdivide_curve(points, t=0.5):
    n = len(points)
    for k in range(1,n):
        newpoints = []
        for i in range(0, n-k):
            x = (1-t) * points[i][0] + t * points[i+1][0]
            y = (1-t) * points[i][1] + t * points[i+1][1]
            newpoints.append((x,y))
        newpoints = np.array(newpoints)
        ax.plot(newpoints.T[0], newpoints.T[1], '--o', label = "Control Polygon")
        points = newpoints


def main():
    # control_points = np.array([[5., 1.], [-3, 1.], [-8, 0], [-11, -5], [-7., -8.]])
    control_points = np.array([[5., 1.], [-5, 1.], [-5, -6], [5., -6]])
    n = len(control_points)
    t = 0.5
    # t = 0.2
    # t = 0.8
    path = bezier_path(control_points, n_points=100)

    ax.plot(path.T[0], path.T[1], label="Bezier Path")
    ax.plot(control_points.T[0], control_points.T[1], '--o', label="Control Polygon")
    subdivide_curve(control_points,t)
    ax.legend()
    ax.axis("equal")
    ax.grid(True)
    plt.title("Subdivision algorithm, t="+str(t)+" ,n="+str(n))
    plt.show()

if __name__ == '__main__':
    main()
