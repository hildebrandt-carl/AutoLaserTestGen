from z3 import *
import matplotlib.pyplot as plt
import numpy as np
import math

# Declare pi
pi = math.pi

# Laser scanner details for the block distance test
theta_min = [-(1), -(pi/2)]
theta_max = [1, (pi/2)]
num_blocks = 1
beam_length = [5, 10]
num_beams = [5, 10]


# Add the block distance constraints
def block_distance(theta_min, theta_max, beam_length):

	# Create the solver
	s = Solver()

	# Create the variables we need to solve this constraint
	x = Real('x')
	block_angle = Real('block_angle')

	# The block needs to be inside the field of view of the scanner
	s.add(theta_min[0] <= block_angle)
	s.add(theta_max[0] >= block_angle)

	# The block needs to be placed inside the range of only one scanner
	s.add(x > beam_length[0])
	s.add(x < beam_length[1])

	# Get the model
	s.check()
	model = s.model()

	# Use the equation of a straight line to use the block angle and x 
	# to generate the y position of the robot
	ba = float(model[block_angle].as_fraction())
	t = np.tan(ba)
	y = t * model.eval(x).as_fraction()

	return float(model.eval(x).as_fraction()), y


# Main function which calls the distance constraints
def main():

		b_d = block_distance(theta_min=theta_min,
							 theta_max=theta_max,
							 beam_length=beam_length)
		print(b_d)


if __name__ == '__main__':
	main()