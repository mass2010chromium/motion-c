from motion import so3, se3

m1 = se3.identity()
m2 = (so3.from_rotation_vector([2, 0, 0]), [-1, 0, 0])
tmp = se3.interpolator(m1, m2)

N = 10000000
for i in range(N):
    result = tmp(i/N)
