from z3 import *
import matplotlib.pyplot as plt
import numpy as np
import math

# Declare pi
pi = math.pi

# Laser scanner details for the blind spot test
theta_min = [-(1), -(pi/2)]
theta_max = [1, (pi/2)]
num_blocks = 1
beam_length = [5, 10]
num_beams = [5, 10]

# Add the blind spot constraints
def blind_spot(theta_min, beam_length):

	# Create the solver
	s = Solver()

	# Create the variables we need to solve this constraint
	x = Real('x')
	block_angle = Real('block_angle')

	# block angle needs to be inside only the field of view of one sensor
	s.add(theta_min[0] > block_angle, theta_min[1] < block_angle)

	# The block should be placed inside the beam length but not too close 
	# to the robot
	s.add(x > 0.5, x <= beam_length[1])

	# Get the model
	s.check()
	model = s.model()

	# Use the equation of a straight line to use the block angle and x 
	# to generate the y position of the robot
	ba = float(model[block_angle].as_fraction())
	t = np.tan(ba)
	y = t * model.eval(x).as_fraction()

	return float(model.eval(x).as_fraction()), y

# Main function which calls the blind spot constraints
def main():
		b_s = blind_spot(theta_min=theta_min,
						 beam_length=beam_length)
		print(b_s)

if __name__ == '__main__':
	main()