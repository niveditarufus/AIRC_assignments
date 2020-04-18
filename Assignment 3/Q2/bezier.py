import matplotlib.pyplot as plt
import numpy as np
import scipy.special


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


def main():
    control_points = np.array([[5., 1.], [-3, 1.], [-8, 0], [-11, -5], [-7., -8.]])
    # control_points = np.array([[5., 1.], [-5, 1.], [-5, -6], [5., -6]])

    path = bezier_path(control_points, n_points=1000)

    fig, ax = plt.subplots()
    ax.plot(path.T[0], path.T[1], label="Bezier Curve")
    ax.plot(control_points.T[0], control_points.T[1],
            '--o', color = 'green', label="Control Points")
    ax.legend()
    ax.axis("equal")
    ax.grid(True)
    plt.title("Bezier Curve with "+str(len(control_points))+" control points")
    plt.show()

if __name__ == '__main__':
    main()
