from klampt.math import so3 as so3
from klampt.math import se3 as se3
import motion.se3 as _se3
import time

v1 = [1.0, 1.5, -2.8]
mat1 = (so3.from_rotation_vector(so3.deskew(list(range(9)))), v1)
mat2 = se3.from_rotation(so3.matrix(so3.from_rotation_vector(v1)))
mat3 = ([0]*9, [0]*3)
mat4 = so3.matrix(so3.from_rotation_vector(v1))
c = 2

NITER = 10000
ts = time.time()
for i in range(NITER):
    mat3 = se3.error(mat1, mat2)
te = time.time()
print(te - ts)
NITER = 1000000
ts = time.time()
tmp = 0
for i in range(NITER):
    #mat3 = _se3.error(mat1, mat2)
    tmp = se3.homogeneous(mat1)
    mat1 = se3.from_homogeneous(tmp)
te = time.time()
print(te - ts)
