from klampt.math import so3 as so3
import motion.so3 as _so3
import numpy as np
import time

v1 = [1.0, 1.5, -2.8]
mat1 = list(range(9));
mat2 = so3.from_rotation_vector(v1);
mat3 = [0]*9;
c = 2

NITER = 10000
ts = time.time()
for i in range(NITER):
    mat3 = so3.mul(mat1, mat2)
te = time.time()
print(te - ts)

NITER = 1000000
ts = time.time()
for i in range(NITER):
    mat3 = _so3.mul(mat1, mat2)
te = time.time()
print(te - ts)

mat1 = np.array(np.reshape(mat1, (3, 3), 'F'))
mat2 = np.array(np.reshape(mat2, (3, 3), 'F'))
ts = time.time()
for i in range(NITER):
    mat3 = mat1 @ mat2
te = time.time()
print(te - ts)
