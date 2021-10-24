from motion import so3, se3
from motion.se3 import interpolate
import time

m1 = se3.identity()
m2 = (so3.from_rotation_vector([2, 0, 0]), [-1, 0, 0])
tmp = se3.interpolator(m1, m2)

N = 10000000
t1 = time.time()
for i in range(N):
    result = tmp(i/N)
t2 = time.time()
print(t2 - t1)

t1 = time.time()
for i in range(N):
    result = interpolate(m1, m2, i/N)
t2 = time.time()
print(t2 - t1)
