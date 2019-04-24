from z3 import *
import matplotlib.pyplot as plt
import numpy as np
import math


pi = math.pi

# #Blind Spot
# theta_min = [-(1), -(pi/2)]
# theta_max = [1, (pi/2)]
# num_blocks = 1
# beam_length = [5, 10]
# num_beams = [5, 10]
#
# # Block Distance
# theta_min = [-(1), -(pi/2)]
# theta_max = [1, (pi/2)]
# num_blocks = 1
# beam_length = [5, 10]
# num_beams = [5, 10]

# Block Size
theta_min = [-(pi/2), -(pi/2)]
theta_max = [(pi/2), (pi/2)]
num_blocks = 1
beam_length = [10, 10]
num_beams = [16, 32]


def blind_spot(theta_min, beam_length):

	s = Solver()

	x = Real('x')
	block_angle = Real('block_angle')

	s.add(theta_min[0] > block_angle, theta_min[1] < block_angle)

	s.add(x > 0, x <= beam_length[1])

	s.check()
	model = s.model()
	ba = float(model[block_angle].as_fraction())
	t = np.tan(ba)
	y = t * model.eval(x).as_fraction()

	# print(model.eval(block_angle))

	return float(model.eval(x).as_fraction()), y


def block_distance(theta_min, theta_max, beam_length):

	s = Solver()

	x = Real('x')
	block_angle = Real('block_angle')

	s.add(theta_min[0] <= block_angle)
	s.add(theta_max[0] >= block_angle)
	s.add(x > beam_length[0])
	s.add(x < beam_length[1])

	s.check()
	model = s.model()
	ba = float(model[block_angle].as_fraction())
	t = np.tan(ba)
	y = t * model.eval(x).as_fraction()

	return float(model.eval(x).as_fraction()), y


def block_size(theta_min, theta_max, beam_length, num_beams):

	s = Solver()

	x = beam_length[0]/15
	y = 0
	#block_angle = Real('block_angle')
	sz = Real('sz')

	# s.add(x > 0, x >= (beam_length[1])/2, x <= (beam_length[1]))
	# for a minimum of 2 beams
	tot_angle = (abs(theta_max[0]) + abs(theta_min[0]))
	sectors = max(num_beams) - 1
	sector_angle = (tot_angle/sectors)
	#s.add(block_angle == sector_angle + theta_min[0])


	sector_sz = np.tan(sector_angle/2) * x

	print(sector_sz)

	s.add(sz > 0, sz <= sector_sz)

	s.check()
	model = s.model()

	size = float(model[sz].as_fraction())

	print(size)


	return x, y, size



def main():
		# b_s = blind_spot(theta_min=theta_min, beam_length=beam_length)
		# print(b_s)
		#
		# b_d = block_distance(theta_min=theta_min, theta_max=theta_max, beam_length=beam_length)
		# print(b_d)

		b_p = block_size(theta_min=theta_min,
						 theta_max=theta_max, beam_length=beam_length,
						 num_beams= num_beams)
		print(b_p)


if __name__ == '__main__':
	main()