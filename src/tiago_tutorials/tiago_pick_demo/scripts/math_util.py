import math

def eular_to_q(roll, pitch, yaw):
	cy = math.cos(yaw * 0.5)
	sy = math.sin(yaw * 0.5)
	cr = math.cos(roll * 0.5)
	sr = math.sin(roll * 0.5)
	cp = math.cos(pitch * 0.5)
	sp = math.sin(pitch * 0.5)

	q_w = cy * cr * cp + sy * sr * sp
	q_x = cy * sr * cp - sy * cr * sp
	q_y = cy * cr * sp + sy * sr * cp
	q_z = sy * cr * cp - cy * sr * sp

	return q_w, q_x, q_y, q_z

def q_to_eular(w, x, y, z):
	ysqr = y * y
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))
	
	return X, Y, Z

def eular_dist(x_1, y_1, z_1, x_2, y_2, z_2):
	dx = (x_1 - x_2)**2
	dy = (y_1 - y_2)**2
	dz = (z_1 - z_2)**2
	r = math.sqrt(dx + dy + dz)
	return r


if __name__ == '__main__':
	print("eular to quaternion: ",eular_to_q(180, 0, 0))
	print("quaternion to eular: ",q_to_eular(0.0, 0.0, 0.0, 0))
