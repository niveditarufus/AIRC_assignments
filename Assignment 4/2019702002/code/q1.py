import numpy as np
import matplotlib.pyplot as plt
fig, ax = plt.subplots()

def get_b_spline(degree, knot_vector, num_points = 100):
	m = len(knot_vector)
	n = m - degree - 1
	u_vector = np.linspace(knot_vector[0], knot_vector[m-1], num_points)

	N = []
	for i in range(0, m-1):
		N_0 = []
		for u in u_vector:
			if(u>=knot_vector[i] and u<knot_vector[i+1]):
				x = 1
			else:
				x = 0
			N_0.append(x)
		N_0 = np.array(N_0)
		if(degree == 0):
			ax.step(u_vector, N_0, label = "N("+str(i)+","+str(degree)+")")
			plt.legend()
		N.append(N_0)

	if(degree>0):
		N = np.array(N)
		points = N
		print(N)
		print(u_vector.shape)

		
		for p in range(1, (degree + 1)):
			# for i in range(len(N)):
			# 	ax.plot(u_vector, N[i],'--', color = 'grey')
			# 	plt.legend()
			
			points = []
			n = m - p -1
			for i in range(0, n):

				temp = []
				for j in range(0, len(u_vector)):
					# print(p,i,j,n)
					x = ((u_vector[j] - knot_vector[i])/(knot_vector[i+p] - knot_vector[i]))*N[i][j]
					y = ((knot_vector[i+p+1] - u_vector[j])/(knot_vector[i+p+1] - knot_vector[i+1]))*N[i+1][j]

					temp.append((x+y))
				points.append(np.array(temp))
			points = np.array(points)
			N = points


		for i in range(len(N)):
			ax.plot(u_vector, N[i], label = "N("+str(i)+","+str(degree)+")")
			plt.legend()


		print(N[0].shape)


def main():
	knot_vector = np.array([ 0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2])
	# knot_vector = np.array([0, 0.3, 1, 3, 4, 5, 9, 10])
	# knot_vector = np.array([0,1,2,3])
	degree = 7
	num_points = 1000
	get_b_spline(degree, knot_vector, num_points)
	plt.title("degree = "+str(degree))
	plt.show()

if __name__ == '__main__':
	main()