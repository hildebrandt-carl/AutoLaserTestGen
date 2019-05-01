from z3 import *
import matplotlib.pyplot as plt
import numpy as np
import math

# Declare pi
pi = math.pi

# Laser scanner details for the block size test
theta_min = [-(pi/2), -(pi/2)]
theta_max = [(pi/2), (pi/2)]
num_blocks = 1
beam_length = [10, 10]
num_beams = [16, 32]


# Add the block size constraints
def block_size(theta_min, theta_max, beam_length, num_beams):

	# Create the solver
	s = Solver()

	# Create the variables we need to solve this constraint
	sz = Real('sz')

	# Used to calculate the x and y co-ordinates of the block
	x = beam_length[0]/10
	y = 0

	# Calculate the angle between each of the beams.
	tot_angle = (abs(theta_max[0]) + abs(theta_min[0]))
	sectors = max(num_beams) - 1
	sector_angle = (tot_angle/sectors)
	
	# Use this angle to calculate the distance between the beams
	sector_sz = np.tan(sector_angle/2) * x

	# The blocks size needs to be less than this
	s.add(sz > 0, sz <= sector_sz)

	# Get the model
	s.check()
	model = s.model()

	size = float(model[sz].as_fraction())
	return x, y, size


# Main function which calls the block size constraints
def main():
		b_p = block_size(theta_min=theta_min,
						 theta_max=theta_max, beam_length=beam_length,
						 num_beams= num_beams)
		print(b_p)


if __name__ == '__main__':
	main()