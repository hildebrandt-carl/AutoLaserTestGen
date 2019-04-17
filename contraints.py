from z3 import *
import matplotlib.pyplot as plt
import numpy as np
import math


pi = math.pi

theta_min = [-(pi / 4), pi / 4]
theta_max = [pi / 4, 3 * pi / 4]
num_blocks = 1
beam_length = [1, 2]
sensor_angles = [1, 2, 3, 4]
num_beams = [5, 10]


def blind_spot(theta_min, theta_max, beam_length):

	s = Solver()

	x = Real('x')
	block_angle = Real('block_angle')
	s.add(theta_min[0] <= block_angle, theta_max[1] >= block_angle)


	s.add(x > 0, x <= beam_length[1])

	s.check()
	model = s.model()
	ba = float(model[block_angle].as_fraction())
	t = np.tan(ba)
	y = t * model.eval(x).as_fraction()

	return float(model.eval(x).as_fraction()), y


def block_distance(theta_min, theta_max, beam_length):

	s = Solver()

	x = Real('x')
	block_angle = Real('block_angle')

	s.add(theta_min[0] <= block_angle)
	s.add(theta_max[0] >= block_angle)
	s.add(x >= beam_length[0])
	s.add(x <= beam_length[1])

	s.check()
	model = s.model()
	ba = float(model[block_angle].as_fraction())
	t = np.tan(ba)
	y = t * model.eval(x).as_fraction()

	return float(model.eval(x).as_fraction()), y


def block_size(theta_min, theta_max, beam_length, num_beams):

	s = Solver()

	x = Real('x')
	block_angle = Real('block_angle')

	s.add(x > 0, x >= (beam_length[1])/2, x <= (beam_length[1]))
	s.add(block_angle == ((theta_max[0] - theta_min[0])/max(num_beams)) + theta_min[0])

	s.check()
	model = s.model()

	ba = float(model[block_angle].as_fraction())
	t = np.tan(ba)
	y = t * model.eval(x).as_fraction()
	if ba < 0:
		ba = ba + 2*pi
	s = np.sin(ba/2)
	size = 2 * float(model.eval(x).as_fraction()) * s

	return float(model.eval(x).as_fraction()), y, size



def main():
		b_s = blind_spot(theta_min=theta_min, theta_max=theta_max, beam_length=beam_length)
		print(b_s)

		b_d = block_distance(theta_min=theta_min, theta_max=theta_max, beam_length=beam_length)
		print(b_d)

		b_p = block_size(num_blocks=num_blocks, theta_min=theta_min,
						 theta_max=theta_max, beam_length=beam_length, sensor_angles= sensor_angles,
						 num_beams= num_beams)
		print(b_p)


if __name__ == '__main__':
	main()