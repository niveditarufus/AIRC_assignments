import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def first_derivative(time):
	dx = 1
	dy = 2*time
	dz = 3*(time**2)
	return dx, dy, dz


def second_derivative(time):
	d_dx = 0
	d_dy = 2
	d_dz = 6*time
	return d_dx, d_dy, d_dz


def third_derivative(time):
	d_ddx = 0
	d_ddy = 0
	d_ddz = 6
	return d_ddx, d_ddy, d_ddz


def make_param(time):
	x_time = time
	y_time = time**2
	z_time = time**3

	dx, dy, dz = first_derivative(time)
	d_dx, d_dy, d_dz = second_derivative(time)
	d_ddx, d_ddy, d_ddz = third_derivative(time)

	r = np.array([x_time, y_time, z_time])
	r_dot = np.array([dx, dy, dz])
	r_ddot = np.array([d_dx, d_dy, d_dz])
	r_dddot = np.array([d_ddx, d_ddy, d_ddz])

	return x_time, y_time, z_time, r, r_dot, r_ddot, r_dddot


def TNB(r, r_dot, r_ddot, r_dddot):
	r_dot_norm = np.linalg.norm(r_dot)
	T = r_dot/r_dot_norm

	B = np.cross(r_dot, r_ddot)
	B_norm = np.linalg.norm(B)
	B = B/B_norm

	N = np.cross(B, T)

	return T, N, B


def get_torsion(r, r_dot, r_ddot, r_dddot):
	t_norm = (np.linalg.norm(np.cross(r_dot, r_ddot))**2)
	torsion = np.dot(np.cross(r_dot, r_ddot),r_dddot)
	torsion = torsion/t_norm
	return torsion


def get_curvature(r, r_dot, r_ddot, r_dddot):
	curvature = np.linalg.norm(np.cross(r_dot, r_ddot))
	c_norm = (np.linalg.norm(r_dot)**3)
	curvature = curvature/c_norm
	return curvature


def get_center(r, N, curvature):
	center = r - (N/curvature)
	return center

def plotting(time,T, B, N, center, curvature, torsion, x_time, y_time, z_time, n_points=100):
	fig = plt.figure()
	ax = plt.axes(projection="3d")

	t = np.linspace(0, 1, n_points)

	x = t
	y = t**2
	z = t**3

	rad = np.linspace(0, 2*np.pi, 100)

	x_c = center[0] + np.cos(rad)*(1/curvature)*T[0] + np.sin(rad)*(1/curvature)*N[0]
	y_c = center[1] + np.cos(rad)*(1/curvature)*T[1] + np.sin(rad)*(1/curvature)*N[1]
	z_c = center[2] + np.cos(rad)*(1/curvature)*T[2] + np.sin(rad)*(1/curvature)*N[2]

	x_T = x_time+T[0]
	x_N = x_time+N[0]
	x_B = x_time+B[0]

	y_T = y_time+T[1]
	y_N = y_time+N[1]
	y_B = y_time+B[1]

	z_T = z_time+T[2]
	z_N = z_time+N[2]
	z_B = z_time+B[2]

	ax.scatter(x_time, y_time, z_time, color = 'black', marker = '*', s = 80, label = 'x, y, z at t='+str(time))
	ax.plot3D(x, y, z, label = "The curve")
	ax.plot3D([x_time,x_T], [y_time,y_T], [z_time,z_T], label = "T")
	ax.plot3D([x_time,x_N], [y_time,y_N], [z_time,z_N], label = "N")
	ax.plot3D([x_time,x_B], [y_time,y_B], [z_time,z_B], label = "B")
	ax.plot3D(x_c, y_c, z_c, label = "Osculating Circle")
	ax.legend()
	plt.title("Curvature: " + str(round(curvature,3))+", Torsion: " + str(round(torsion,3)))
	plt.show()


def main():
	time = 0.5
	x_time, y_time, z_time, r, r_dot, r_ddot, r_dddot = make_param(time)
	T, N, B = TNB(r, r_dot, r_ddot, r_dddot)
	torsion = get_torsion(r, r_dot, r_ddot, r_dddot)
	curvature = get_curvature(r, r_dot, r_ddot, r_dddot)
	center = get_center(r, N, curvature)
	plotting(time, T, B, N, center, curvature, torsion, x_time, y_time, z_time)



if __name__ == '__main__':
	main()


