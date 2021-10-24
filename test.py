from klampt.math import vectorops as vo1
import motion.vectorops as vo2
import numpy as np

v1 = [1.0, 1.5, -2.8]
v2 = [0.6, -0.7, 0.2]
v3 = [0.0, 0.0, 0.0]
c = 2
c2 = 0.5

#def print(*args):
#    pass
N=1

for i in range(N):
    print(vo1.madd(v1, v2, c), vo2.madd(v1, v2, c))
    print(vo1.sub(v1, v2), vo2.sub(v1, v2))
    print(vo1.sub(v1, c), vo2.sub(v1, c))
    print(vo1.mul(v1, v2), vo2.mul(v1, v2))
    print(vo1.mul(v1, c), vo2.mul(v1, c))
    print(vo1.div(v1, v2), vo2.div(v1, v2))
    print(vo1.div(v1, c), vo2.div(v1, c))
    print(vo1.maximum(v1, v2), vo2.maximum(v1, v2))
    print(vo1.maximum(v1, c), vo2.maximum(v1, c))
    print(vo1.minimum(v1, v2), vo2.minimum(v1, v2))
    print(vo1.minimum(v1, c), vo2.minimum(v1, c))
    print(vo1.dot(v1, v2), vo2.dot(v1, v2))
    print(vo1.normSquared(v1), vo2.normSquared(v1))
    print(vo1.norm(v2), vo2.norm(v2))
    print(vo1.unit(v2), vo2.unit(v2))
    print()
    print(vo1.norm_L1(v2), vo2.norm_L1(v2))
    print(vo1.norm_L2(v2), vo2.norm_L2(v2))
    print(vo1.norm_Linf(v2), vo2.norm_Linf(v2))
    print(vo1.distanceSquared(v1, v2), vo2.distanceSquared(v1, v2))
    print(vo1.distance(v1, v2), vo2.distance(v1, v2))
    print(vo1.cross(v1, v2), vo2.cross(v1, v2))
    print(vo1.interpolate(v1, v2, c2), vo2.interpolate(v1, v2, c2))
