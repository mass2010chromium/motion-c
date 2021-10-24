from klampt.math import vectorops as vo1
import motion.vectorops as vo2
import numpy as np
import time

v1 = [1.0, 1.5, -2.8]
v2 = [0.6, -0.7, 0.2]
v3 = [0.0, 0.0, 0.0]
c = 2

NITER = 1000000
ts = time.time()
for i in range(NITER):
    v3 = vo1.madd(v1, v2, c)
te = time.time()
print(te - ts)

ts = time.time()
for i in range(NITER):
    v3 = vo1.mul(v1, c)
te = time.time()
print(te - ts)

ts = time.time()
for i in range(NITER):
    v3 = vo2.madd(v1, v2, c)
te = time.time()
print(te - ts)

v1 = np.array(v1)
v2 = np.array(v2)
ts = time.time()
for i in range(NITER):
    v3 = v1 + v2 * c
te = time.time()
print(te - ts)
