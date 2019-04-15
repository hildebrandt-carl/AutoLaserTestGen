# Created by Minbiao Han and Roman Sharykin
# CS6501-003 Spring 2019
# Ref: The SMT_Based Automatic Road Network Generation in Vehicle Simulation Environment (BaekGyu Kim et al. 2016)

from z3 import *
import matplotlib.pyplot as plt
import numpy as np

# Curve Coverage Criteria [n_min, n_max, theta_min, theta_max, d_min, d_max]
# n_min: min number of curves
# n_max: max number of curves
# theta_min: min curvature of each curve
# theta_max: max curvature of each curve
# d_min: min distance of any two adjacent curves
# d_max: max distance of any two adjacent curves

C_g = [5, 100, 0, 35, 5, 22]  # global coverage criteria

C_l = []  # local coverage criteria
solN = 5

# Adding local coverage criteria
C_l.append([4, 7, 0, 1, 10, 14])
C_l.append([2, 7, 1, 3, 11, 15])
C_l.append([3, 9, 0, 2, 8, 12])
C_l.append([3, 8, 1, 2, 12, 20])
C_l.append([2, 10, 0, 3, 10, 15])

# Boundary Information
x_min = 0
y_min = 0
z_min = 0
x_max = 300
y_max = 300
z_max = 300


def abs(x):
	return If(x >= 0, x, -x)

#####################################
# Complete the code
# Q1: Road Segment Generation Algorithm
#####################################
# Note: Curves need to be generated with curves in order across a certain axis


def road_seg_gen(X, Y, Z, x_min, y_min, z_min, x_max, y_max, z_max, d_min, d_max, theta_min, theta_max, N):

	s = Solver()

	# Boundary Constraint
	for i in range(N):
		s.add(X[i] >= x_min, X[i] <= x_max)
		s.add(Y[i] >= y_min, Y[i] <= y_max)
		s.add(Z[i] >= z_min, Z[i] <= z_max)

	# Curve distance constraint
	for i in range(N - 1):
		s.add(X[i + 1] - X[i] <= d_max, X[i + 1] - X[i] >= d_min)
		s.add(Y[i + 1] - Y[i] <= d_max, Y[i + 1] - Y[i] >= d_min)
		s.add(Z[i + 1] - Z[i] <= d_max, Z[i + 1] - Z[i] >= d_min)

	# Curvature constraint
	for i in range(N - 2):
		l_0 = (Y[i + 1] - Y[i])/(X[i + 1] - X[i])
		l_1 = (Z[i + 1] - Z[i])/(X[i + 1] - X[i])
		r_0 = (Y[i + 2] - Y[i + 1])/(X[i + 2] - X[i + 1])
		r_1 = (Z[i + 2] - Z[i + 1])/(X[i + 2] - X[i + 1])
		s.add(abs(l_0 - r_0) >= theta_min, abs(l_0 - r_0) <= theta_max)
		s.add(abs(l_1 - r_1) >= theta_min, abs(l_1 - r_1) <= theta_max)

	# Alternating road constraint
	for i in range(N - 2):
		l_0 = (Y[i + 1] - Y[i]) / (X[i + 1] - X[i])
		r_0 = (Y[i + 2] - Y[i + 1]) / (X[i + 2] - X[i + 1])

		s.add((X[i + 1] - X[i]) != 0)
		s.add((X[i + 2] - X[i + 1]) != 0)
		s.add((l_0 * r_0) <= 0)

	if len(curNumArray) >= 1:

		# Last waypoint from previous road seg
		last_waypoint_x = solArrayX[-1][-1].as_long()
		last_waypoint_y = solArrayY[-1][-1].as_long()
		last_waypoint_z = solArrayZ[-1][-1].as_long()

		# Last waypoint == first waypoint
		s.add(X[0] == last_waypoint_x)
		s.add(Y[0] == last_waypoint_y)
		s.add(Z[0] == last_waypoint_z)

		# Second last waypoint
		penultimate_waypoint_x = solArrayX[-1][-2].as_long()
		penultimate_waypoint_y = solArrayY[-1][-2].as_long()
		penultimate_waypoint_z = solArrayZ[-1][-2].as_long()

		# Theta Global
		lg_0 = (Y[0] - penultimate_waypoint_y) / (X[0] - penultimate_waypoint_x)
		lg_1 = (Z[0] - penultimate_waypoint_z) / (X[0] - penultimate_waypoint_x)
		rg_0 = (Y[1] - Y[0]) / (X[1] - X[0])
		rg_1 = (Z[1] - Z[0]) / (X[1] - X[0])

		# Get the global constraint values
		theta_min_g = C_g[2]
		theta_max_g = C_g[3]

		# Add the global constraint
		s.add(abs(lg_0 - rg_0) <= theta_max_g, abs(lg_0 - rg_0) >= theta_min_g)
		s.add(abs(lg_1 - rg_1) <= theta_max_g, abs(lg_1 - rg_1) >= theta_min_g)

	# Evaluate the Road Segement
	s.check()
	model = s.model()

	# Add the road segment to the solution arrays
	x_return = []
	for x in X:
		x_return.append(model.eval(x))

	solArrayX.append(x_return)
	print("x")
	print(solArrayX)

	y_return = []
	for y in Y:
		y_return.append(model.eval(y))
	solArrayY.append(y_return)
	print("y")
	print(solArrayY)

	z_return = []
	for z in Z:
		z_return.append(model.eval(z))
	solArrayZ.append(z_return)
	print("z")
	print(solArrayZ)

	curNumArray.append(N)


def blind_spot(X, Y, num_blocks, theta_min, theta_max, beam_max):

	s = Solver()

	block_angle = Real('block_angle')

	s.add(min(theta_min) <= block_angle, min(theta_max) >= block_angle)

	for i in range(num_blocks):
		s.add(X[i] >= 0, X[i] <= beam_max)
		# s.add(Y[i] == tan * X[i])

	check = s.check()
	print(check)

	model = s.model()
	print(model)

	print("Block Angle: ", model[block_angle])

	ba = float(model[block_angle].as_fraction())
	tan = np.tan(ba)

	for i in range(num_blocks):
		s.add(Y[i] == tan * X[i])

	s.check()
	s.model()


	# Block Distance Constraint
	# Beam_min <= sqrt((x_r - x_b) ^ 2 + (y_r + y_b) ^ 2) <= Beam_max
	# Min(Angle_max) < = Block_angle <= Max(Angle_min)
	# Y = tan(Block_angle)x

	# Block Size Constraint
	# 0 <= sqrt((x_r - x_b) ^ 2 + (y_r + y_b) ^ 2) = Min(Beam_max)
	# Max(Min_Angle) <= Block_angle <= Min(Max_Angle)
	# Angle = (S_MostBeams(Max_Angle - Min_Angle)) /  # Beams +    S_MostBeams(Min_Angle)
	# Size = 2(sqrt((x_r - x_b) ^ 2 + (y_r + y_b) ^ 2)) * sin(angle / 2)


#####################################
# Complete the code
# Q2: Coverage Consistency Check
#####################################
def coverage_consistence(C_l, C_g, K):
	# Consistency check for criteria N
	for i in range(0, 1):
		x = [item[i] for item in C_l]
		if (K - 1) + np.sum(x) < C_g[i]:
			return False

	# Consistency check for Theta_min D_min
	for i in range(2, 6):
		for j in range(5):
			if C_l[j][i] < C_g[i] and ((i == 2) or (i == 4)):
				return False
			if C_l[j][i] > C_g[i] and ((i == 3) or (i == 5)):
				return False
	return True

# Sequential Road Network Information
# Each array is a 2D array
# e.g. solArrayX = [[1, 2, 3], [4, 5, 6]] means the road network has 2 segments and each segment has 3 waypoints.
# The X-coordinates of the 3 waypoints of Segment1 are 1, 2, and 3, the X-coordinates of the 3 waypoints of
# Segment2 are 4, 5, and 6.
# For your outputs, the size of your solArrayX/Y/Z will be (solN * N), which means your road network has
# solN segments, and each segment has N waypoints


solArrayX = []
solArrayY = []
solArrayZ = []
curNumArray = []


def main():
	# Coverage consistency check
	cov_con = coverage_consistence(C_l, C_g, solN)
	if cov_con == False:
		print("No-Sol\n")
		return 0
	else:
		#####################################
		# Complete the code
		# Q3: Sequential Road Network Generation Algorithm
		#####################################

		# # For the number of road segments
		# for sol_ind in range(solN):
		# 	# Get all the constraints
		# 	n_min = C_l[sol_ind][0]
		# 	# n_max = C_l[sol_ind][1]
		# 	# theta_min = C_l[sol_ind][2]
		# 	# theta_max = C_l[sol_ind][3]
		# 	# d_min = C_l[sol_ind][4]
		# 	# d_max = C_l[sol_ind][5]
		#
		# 	num_seg = n_min
		#
		# 	# Create an array z3 variables num_seg long

		num_blocks = 1
		beam_max = 1  # meters

		X = [Int("x_%s" % i) for i in range(num_blocks)]
		Y = [Int("y_%s" % i) for i in range(num_blocks)]

		theta_min = [ 0.5, 1, 1.5]
		theta_max = [10, 20, 30, 40]


		b_s = blind_spot(X=X, Y=Y, num_blocks=num_blocks, theta_min=theta_min, theta_max=theta_max, beam_max=beam_max)
		print(b_s)


	# 		# Generate the road
	# 		road_seg_gen(
	# 					X = X,
	# 					Y = Y,
	# 					Z = Z,
	# 					x_min = x_min,
	# 					y_min = y_min,
	# 					z_min = z_min,
	# 					x_max = x_max,
	# 					y_max = y_max,
	# 					z_max = z_max,
	# 					d_min = d_min,
	# 					d_max = d_max,
	# 					theta_min = theta_min,
	# 					theta_max = theta_max,
	# 					N = num_seg)
	#
	# # Plot our road
	# for j in range(len(solArrayZ)):
	# 	a = []
	# 	b = []
	# 	for i in range(len(solArrayZ[j])):
	# 		a.append(solArrayY[j][i].as_long())
	# 		b.append(solArrayZ[j][i].as_long())
	# 		if j == 0:
	# 			plt.plot(a, b, 'r')
	# 		if j == 1:
	# 			plt.plot(a, b, 'b')
	# 		if j == 2:
	# 			plt.plot(a, b, 'g')
	# 		if j == 3:
	# 			plt.plot(a, b, 'k')
	# 		if j == 4:
	# 			plt.plot(a, b, 'y')
	#
	# plt.show()
	#
	# # Write the generated road information into a file
	# file = open("coordinates.txt", "w")
	# for seg in range(solN):
	# 	file.write('Seg ' + str(seg + 1) + ':\n')
	# 	for point in range(curNumArray[seg]):
	# 		file.write(
	# 			str(30 * int(str(solArrayY[seg][point]))) + ", 0, " + str(30 * int(str(solArrayZ[seg][point]))) + "\n")
	# file.close()
	#
	#
	# # DO NOT CHANGE THIS PART OF THE CODE
	# # Additional part for generating the correct output format for visualising the generated road in Unity
	#
	# # if you wish to view the generated coordinates without alterations for Unity, just comment out
	# # this last part up to if __name__ == '__main__':
	# with open('coordinates.txt') as f:
	# 	content = f.readlines()
	#
	# content = [x.strip() for x in content]
	# last = ''
	# to_remove = []
	# for cont in content:
	# 	if 'Seg' in cont:
	# 		to_remove.append(cont)
	#
	# for rem in to_remove:
	# 	content.remove(rem)
	#
	# to_remove = []
	# for cont in content:
	# 	if last == cont:
	# 		to_remove.append(cont)
	# 	last = cont
	#
	# for rem in to_remove:
	# 	content.remove(rem)
	#
	# with open('coordinates.txt', 'w') as f:
	# 	for item in content:
	# 		print(item)
	# 		f.write("%s\n" % item)


if __name__ == '__main__':
	main()