#pragma once

#include <Python.h>

#include <math.h>
#include "vectorops.h"

#ifdef MOTION_DEBUG
#ifdef SO3_STRICT
#define PARSE_VEC_FIX(name, len) \
int parse_ ## name (double* dest, PyObject* vec) { \
    Py_ssize_t n = PyObject_Length(vec); \
    if (n < 0) { \
        PyErr_SetString(PyExc_TypeError, "object has no length"); \
        return 1; \
    } \
    if (n != (len)) { \
        PyErr_SetString(PyExc_ValueError, #name " expected " #len " element vector"); \
        return 1; \
    } \
    PyObject* it = list_to_vector_n(vec, dest, (len)); \
    if (it == NULL) return 1; \
    Py_DECREF(it); \
    return 0; \
}
#else
#define PARSE_VEC_FIX(name, len) \
inline int parse_ ## name (double* dest, PyObject* vec) { \
    Py_ssize_t n = PyObject_Length(vec); \
    if (n < 0) { \
        PyErr_SetString(PyExc_TypeError, "object has no length"); \
        return 1; \
    } \
    if (n > (len)) { \
        PyErr_SetString(PyExc_ValueError, #name " expected " #len " element vector"); \
        return 1; \
    } \
    return list_to_vector(vec, dest); \
}
#endif
#else
#define PARSE_VEC_FIX(name, len) \
inline int parse_ ## name (double* dest, PyObject* vec) { \
    return list_to_vector(vec, dest); \
}
#endif

PARSE_VEC_FIX(rotation, 9);
PARSE_VEC_FIX(vec3, 3);
PARSE_VEC_FIX(quat, 4);

const double SO3_ID[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

/**
 * Returns the identity rotation
 */
PyObject* so3_identity(PyObject* self, PyObject* args);

/**
 * Inverts the rotation
 */
PyObject* so3_inv(PyObject* self, PyObject* args);

inline void __so3_inv(double* dest, double* src) {
    dest[0] = src[0];
    dest[1] = src[3];
    dest[2] = src[6];
    dest[3] = src[1];
    dest[4] = src[4];
    dest[5] = src[7];
    dest[6] = src[2];
    dest[7] = src[5];
    dest[8] = src[8];
}

/**
 * Applies the rotation to a point
 */
PyObject* so3_apply(PyObject* self, PyObject* args);

inline void __so3_apply(double* dest, double* r, double* v) {
    dest[0] = r[0]*v[0] + r[3]*v[1] + r[6]*v[2];
    dest[1] = r[1]*v[0] + r[4]*v[1] + r[7]*v[2];
    dest[2] = r[2]*v[0] + r[5]*v[1] + r[8]*v[2];
}

/**
 * Returns the 3x3 rotation matrix corresponding to R
 */
PyObject* so3_matrix(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

#ifndef MOTION_DEBUG
inline
#endif
PyObject* __so3_matrix(double* buffer) {
    double buffer2[9];
    __so3_inv(buffer2, buffer);
    PyObject* l1 = vector_to_list(buffer2, 3);
#ifdef MOTION_DEBUG
    if (l1 == NULL) {
        return NULL;
    }
#endif
    PyObject* l2 = vector_to_list(buffer2+3, 3);
#ifdef MOTION_DEBUG
    if (l2 == NULL) {
        Py_DECREF(l1);
        return NULL;
    }
#endif
    PyObject* l3 = vector_to_list(buffer2+6, 3);
#ifdef MOTION_DEBUG
    if (l3 == NULL) {
        Py_DECREF(l1);
        Py_DECREF(l2);
        return NULL;
    }
#endif
    PyObject* result = Py_BuildValue("[NNN]", l1, l2, l3);
#ifdef MOTION_DEBUG
    if (result == NULL) {
        Py_DECREF(l1);
        Py_DECREF(l2);
        Py_DECREF(l3);
        return NULL;
    }
#endif
    return result;
}

/**
 * Returns an R corresponding to the 3x3 rotation matrix mat
 */
PyObject* so3_from_matrix(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

#ifndef MOTION_DEBUG
inline
#endif
int __so3_from_matrix(double* dest, PyObject* rot) {
#ifdef MOTION_DEBUG
    Py_ssize_t n = PyObject_Length(rot);
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return 1;
    }
#ifdef SO3_STRICT
    if (n != 3)
#else
    if (n < 3)
#endif
    {
        PyErr_SetString(PyExc_ValueError, "Rotation is 3x3 matrix");
        return 1;
    }
#endif

    PyObject* it = PyObject_GetIter(rot);
#ifdef MOTION_DEBUG
    if (it == NULL) {
        return 1;
    }
#endif

    double buf[9];
    double* buf_head = buf;
    PyObject* curr;
    while ((curr = PyIter_Next(it))) {
#ifdef MOTION_DEBUG
        if (curr == NULL) {
            Py_DECREF(it);
            return 1;
        }
#endif
        if (parse_vec3(buf_head, curr)) {
            Py_DECREF(curr);
            Py_DECREF(it);
            return 1;
        }
        buf_head += 3;
        Py_DECREF(curr);
    }
    Py_DECREF(it);
    __so3_inv(dest, buf);
    return 0;
}

/**
 * Multiplies two rotations.
 */
PyObject* so3_mul(PyObject* self, PyObject* args);

inline void __so3_mul(double* dest, double* r1, double* r2) {
    double r1_T[9];
    __so3_inv(r1_T, r1);
    dest[0] = __vo_dot(r1_T, r2, 3);
    dest[1] = __vo_dot(r1_T+3, r2, 3);
    dest[2] = __vo_dot(r1_T+6, r2, 3);
    dest[3] = __vo_dot(r1_T, r2+3, 3);
    dest[4] = __vo_dot(r1_T+3, r2+3, 3);
    dest[5] = __vo_dot(r1_T+6, r2+3, 3);
    dest[6] = __vo_dot(r1_T, r2+6, 3);
    dest[7] = __vo_dot(r1_T+3, r2+6, 3);
    dest[8] = __vo_dot(r1_T+6, r2+6, 3);
}

/**
 * Computes the trace of the rotation matrix.
 */
PyObject* so3_trace(PyObject* self, PyObject* args);

inline double __so3_trace(const double* rot) {
    return rot[0] + rot[4] + rot[8];
}

/**
 * Returns absolute deviation of R from identity
 */
PyObject* so3_angle(PyObject* self, PyObject* args);

inline double __so3_angle(const double* rot) {
    double ctheta = clamp_unity((__so3_trace(rot) - 1) / 2);
    return acos(ctheta);
}

/**
 * Converts a rotation matrix to a roll,pitch,yaw angle triple. The result is given in radians.
 */
PyObject* so3_rpy(PyObject* self, PyObject* args);

inline void __so3_rpy(double* dest, const double* rot) {
    double _sb = clamp_unity(rot[2]);   // rot[2, 0]
    double b = -asin(_sb);
    double cb = cos(b);
    double c, a;
    if (fabs(cb) > 1e-7) {
        double cb_inv = 1 / cb;
        double ca = clamp_unity(rot[0] * cb_inv); // rot[0, 0]
        if (signum(rot[1]) == signum(cb_inv)) {  // rot[1, 0]
            a = acos(ca);
        }
        else {
            a = (2 * M_PI) - acos(ca);
        }
        double cc = clamp_unity(rot[8] * cb_inv); // rot[2, 2]
        if (signum(rot[5]) == signum(cb_inv)) {  // rot[2, 1]
            c = acos(cc);
        }
        else {
            c = (2 * M_PI) - acos(cc);
        }
    }
    else {
        // b is close to 90 degrees.
        // this reduces the degrees of freedom, so we can set c = 0
        c = 0;
        double _sa = clamp_unity(rot[3]);   // rot[0, 1]
        a = -asin(_sa);
        if (signum(cos(a)) != signum(rot[4])) {  // rot[1, 1]
            a = M_PI - a;
        }
    }
    dest[0] = c;
    dest[1] = b;
    dest[2] = a;
}

/**
 * Converts from roll,pitch,yaw angle triple to a rotation matrix.  The triple is given in radians.  The x axis is 'roll', y is 'pitch', and z is 'yaw'.
 */
PyObject* so3_from_rpy(PyObject* self, PyObject* args);

void __so3_rotation(double* dest, const double* axis, double angle);
void __so3_deskew(double* dest, const double* rot);

inline void __so3_from_rpy(double* dest, double* rpy) {
    double Rx[9];
    double Ry[9];
    double Rz[9];
    __so3_rotation(Rx, SO3_ID+0, rpy[0]);
    __so3_rotation(Ry, SO3_ID+3, rpy[1]);
    __so3_rotation(Rz, SO3_ID+6, rpy[2]);
    double scratch1[9];
    __so3_mul(scratch1, Ry, Rz);
    __so3_mul(dest, Rz, scratch1);
}

/**
 * Returns the rotation vector w (exponential map) representation of R such that e^[w] = R.  Equivalent to axis-angle representation with w/||w||=axis, ||w||=angle.
 */
PyObject* so3_rotation_vector(PyObject* self, PyObject* args);

inline void __so3_rotation_vector(double* dest, double* rot) {
    double theta = __so3_angle(rot);
    if (fabs(theta - M_PI) < 0.5) {
        // for values close to pi this alternate technique has better numerical performance
        double c = cos(theta);
        double c_inv = 1 / (1.0 - c);
        double x2 = (rot[0] - c) * c_inv;
        double y2 = (rot[4] - c) * c_inv;
        double z2 = (rot[4] - c) * c_inv;
        if (x2 < 0) {
            _assert(x2 > -1e-5);
            x2 = 0;
        }
        if (y2 < 0) {
            _assert(y2 > -1e-5);
            y2 = 0;
        }
        if (z2 < 0) {
            _assert(z2 > -1e-5);
            z2 = 0;
        }
        double x = theta * sqrt(x2);
        double y = theta * sqrt(y2);
        double z = theta * sqrt(z2);
        if (fabs(theta - M_PI) < 1e-5) {
            // determined up to sign changes, we know r12=2xy,r13=2xz,r23=2yz
            double xy = rot[3];
            double xz = rot[6];
            double yz = rot[7];
            if (x > y) {
                if (x > z) {
                    // x is largest
                    if (xy < 0) y = -y;
                    if (xz < 0) z = -z;
                }
                else {
                    // z is the largest
                    if (yz < 0) y = -y;
                    if (xz < 0) x = -x;
                }
            }
            else {
                if (y > z) {
                    // y is the largest
                    if (xy < 0) x = -x;
                    if (yz < 0) z = -z;
                }
                else {
                    // z is the largest
                    if (yz < 0) y = -y;
                    if (xz < 0) x = -x;
                }
            }
        }
        else {
            double eps = theta - M_PI;
            if (eps * (rot[3*1+2] - rot[3*2+1]) > 0) {
                x = -x;
            }
            if (eps * (rot[3*2+0] - rot[3*0+2]) > 0) {
                y = -y;
            }
            if (eps * (rot[3*0+1] - rot[3*1+0]) > 0) {
                z = -z;
            }
        }
        dest[0] = x;
        dest[1] = y;
        dest[2] = z;
    }
    else {
        double scale = 1;
        if (fabs(theta) > 1e-5) {
            scale = theta / sin(theta);
        }
        __so3_deskew(dest, rot);
        __vo_mul(dest, dest, scale, 3);
    }
}

/**
 * Returns the (axis,angle) pair representing R
 */
PyObject* so3_axis_angle(PyObject* self, PyObject* args);

/**
 * Converts an axis-angle representation (axis,angle) to a 3D rotation matrix.
 */
PyObject* so3_from_axis_angle(PyObject* self, PyObject* args);

/**
 * Converts a rotation vector representation w to a 3D rotation matrix.
 */
PyObject* so3_from_rotation_vector(PyObject* self, PyObject* args);

/**
 * Given a unit quaternion (w,x,y,z), produce the corresponding rotation matrix.
 */
PyObject* so3_from_quaternion(PyObject* self, PyObject* args);

inline void __so3_from_quaternion(double* rot, double* q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double x2 = x + x,  y2 = y + y,  z2 = z + z;
    double xx = x * x2, xy = x * y2, xz = x * z2;
    double yy = y * y2, yz = y * z2, zz = z * z2;
    double wx = w * x2, wy = w * y2, wz = w * z2;

    rot[0] = 1.0 - (yy + zz);
    rot[3] = xy - wz;
    rot[6] = xz + wy;
    rot[1] = xy + wz;
    rot[4] = 1.0 - (xx + zz);
    rot[7] = yz - wx;
    rot[2] = xz - wy;
    rot[5] = yz + wx;
    rot[8] = 1.0 - (xx + yy);
}

/**
 * Given a Klamp't rotation representation, produces the corresponding unit quaternion (w,x,y,z).
 */
PyObject* so3_quaternion(PyObject* self, PyObject* args);

inline int __so3_quaternion(double* q, double* rot) {
    double tr = __so3_trace(rot) + 1.0;

    if (tr > 1e-5) {
        // If the trace is nonzero, it's a nondegenerate rotation
        double s = sqrt(tr);
        q[0] = s / 2;
        s = 0.5 / s;
        q[1] = (rot[2+1*3] - rot[1+2*3]) * s;
        q[2] = (rot[0+2*3] - rot[2+0*3]) * s;
        q[3] = (rot[1+0*3] - rot[0+1*3]) * s;
        __vo_unit(q, q, 1e-5, 4);
    }
    else {
        // degenerate it's a rotation of 180 degrees
        static const int nxt[3] = {1, 2, 0};
        // check for largest diagonal entry
        int i = 0;
        if (rot[1+1*3] > rot[0+0*3]) {
            if (rot[2+2*3] > rot[1+1*3]) i = 2;
            else i = 1;
        }
        else if (rot[2+2*3] > rot[0+0*3]) i = 2;
        int j = nxt[i];
        int k = nxt[j];
        double s = sqrt(rot[i+i*3] - (rot[j+j*3] + rot[k+k*3]) + 1);
        if (fabs(s) < 1e-7) {
            return 1;
        }
        q[i] = s / 2;
        s = 0.5 / s;
        q[3] = (rot[k+j*3] - rot[j+k*3]) * s;
        q[j] = (rot[i+j*3] + rot[j+i*3]) * s;
        q[k] = (rot[i+k*3] + rot[k+i*3]) * s;   // TODO this is different from klampt.

        __vo_unit(q, q, 1e-5, 4);
    }
    return 0;
}

/**
 * Returns the absolute angle one would need to rotate in order to get from R1 to R2
 */
PyObject* so3_distance(PyObject* self, PyObject* args);

inline double __so3_distance(const double* r1, const double* r2) {
    double scratch1[9];
    double scratch2[9];
    __so3_inv(scratch1, r2);
    __so3_mul(scratch2, r1, scratch1);
    return __so3_angle(scratch2);
}

/**
 * Returns a 3D 'difference vector' that describes how far R1 is from R2.
 * More precisely, this is the (local) Lie derivative,
 * which is the rotation vector representation of R1*R2^T.
 *
 * Fun fact: this is related to the derivative of interpolate(R2,R1,u) at u=0
 * by d/du interpolate(R2,R1,0) = mul(error(R1,R2),R2).
 */
PyObject* so3_error(PyObject* self, PyObject* args);

inline void __so3_error(double* ret, double* r1, double* r2) {
    double scratch1[9];
    double scratch2[9];
    __so3_inv(scratch1, r2);
    __so3_mul(scratch2, r1, scratch1);
    __so3_rotation_vector(ret, scratch2);
}

/**
 * Returns the cross product matrix associated with w.
 * The matrix [w]R is the derivative of the matrix R
 * as it rotates about the axis w/||w|| with angular velocity ||w||. 
 */
PyObject* so3_cross_product(PyObject* self, PyObject* args);

inline void __so3_cross_product(double* dest, const double* vec) {
    dest[0] = 0;
    dest[1] = vec[2];
    dest[2] = -vec[1];
    dest[3] = -vec[2];
    dest[4] = 0;
    dest[5] = vec[0];
    dest[6] = vec[1];
    dest[7] = -vec[0];
    dest[8] = 0;
}

/**
 * Returns the diagonal of the 3x3 matrix reprsenting the so3 element R.
 */
PyObject* so3_diag(PyObject* self, PyObject* args);

inline void __so3_diag(double* dest, const double* rot) {
    dest[0] = rot[0];
    dest[1] = rot[4];
    dest[2] = rot[8];
}

/**
 * If R is a (flattened) cross-product matrix of the 3-vector w, this will return w.
 * Otherwise, it will return a representation w of (R-R^T)/2
 * (off diagonals of R) such that (R-R^T)/2 = cross_product(w). 
 */
PyObject* so3_deskew(PyObject* self, PyObject* args);

inline void __so3_deskew(double* dest, const double* rot) {
    dest[0] = (rot[5] - rot[7])/2;
    dest[1] = (rot[6] - rot[2])/2;
    dest[2] = (rot[1] - rot[3])/2;
}

/**
 * Given a unit axis and an angle in radians, returns the rotation matrix.
 */
PyObject* so3_rotation(PyObject* self, PyObject* args);

inline void __so3_rotation(double* dest, const double* axis, double angle) {
    double cm = cos(angle);
    double sm = sin(angle);
    __so3_cross_product(dest, axis);
    __vo_mul(dest, dest, sm, 9);
    dest[3*0+0] += axis[0]*axis[0]*(1-cm);
    dest[3*0+1] += axis[0]*axis[1]*(1-cm);
    dest[3*0+2] += axis[0]*axis[2]*(1-cm);
    dest[3*1+0] += axis[1]*axis[0]*(1-cm);
    dest[3*1+1] += axis[1]*axis[1]*(1-cm);
    dest[3*1+2] += axis[1]*axis[2]*(1-cm);
    dest[3*2+0] += axis[2]*axis[0]*(1-cm);
    dest[3*2+1] += axis[2]*axis[1]*(1-cm);
    dest[3*2+2] += axis[2]*axis[2]*(1-cm);
    dest[0] += cm;
    dest[4] += cm;
    dest[8] += cm;
}

/**
 * Given a unit vector v, finds R that defines a basis [x,y,z] such that x = v and y and z are orthogonal
 */
PyObject* so3_canonical(PyObject* self, PyObject* args);

inline void __so3_canonical(double* basis, double* axis) {
    if (fabs(axis[0] - 1.0) < 1e5) {
        memcpy(basis, SO3_ID, sizeof(SO3_ID));
        return;
    }
    if (fabs(axis[0] + 1.0) < 1e5) {
        // Flip of identity.
        memcpy(basis, SO3_ID, sizeof(SO3_ID));
        basis[0] = -1.0;
        basis[4] = -1.0;
        return;
    }
    double x = axis[0];
    double y = axis[1];
    double z = axis[2];
    basis[0] = x;
    basis[1] = y;
    basis[2] = z;
    double scale = (1.0 - x) / (1.0 - x*x);
    basis[3] = -y;
    basis[4] = x + scale*z*z;
    basis[5] = -scale*y*z;
    basis[6] = -z;
    basis[7] = -scale*y*z;
    basis[8] = x + scale*y*y;
}

/**
 * Finds the minimal-angle matrix that rotates v1 to v2.  v1 and v2 are assumed to be nonzero
 */
PyObject* so3_vector_rotation(PyObject* self, PyObject* args);

inline void __so3_vector_rotation(double* output, double* v1, double* v2) {
    double cross[3];

    __vo_unit(v1, v1, 1e-5, 3);
    __vo_unit(v2, v2, 1e-5, 3);
    __vo_cross3(cross, v1, v2);
    double dot = __vo_dot(v1, v2, 3);

    if (__vo_norm(cross, 3) < 1e-4) {
        if (dot < 0) {
            double scratch[9];
            __so3_canonical(scratch, v1);
            __so3_rotation(output, scratch+3, M_PI);
        }
        else {
            for (int i = 0; i < 9; ++i) {
                output[i] = SO3_ID[i];
            }
        }
        return;
    }
    double angle = acos(clamp_unity(dot));
    __vo_unit(cross, cross, 1e-5, 3);
    __so3_rotation(output, cross, angle);
}

/**
 * Interpolate linearly between the two rotations R1 and R2. 
 */
PyObject* so3_interpolate(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __so3_interpolate(double* result, double* a, double* b, double c) {
    double scratch1[9];
    double scratch2[9];
    double scratchv[3];
    __so3_inv(scratch1, a);
    __so3_mul(scratch2, scratch1, b);
    __so3_rotation_vector(scratchv, scratch2);
    double angle = __vo_norm(scratchv, 3);
    if (angle == 0) {
        memcpy(result, a, sizeof(SO3_ID));
        return;
    }
    __vo_div(scratchv, scratchv, angle, 3);
    __so3_rotation(scratch1, scratchv, angle * c);
    __so3_mul(result, a, scratch1);
}

typedef struct {
    double rot[9];
    double axis[3];
    double angle;
} so3_interpolator_t;

inline void __so3_interpolator_init(so3_interpolator_t* ret, double* r1, double* r2) {
    double r1_inv_buf[9];
    double scratch[9];
    double* m = r1_inv_buf;
    __so3_inv(r1_inv_buf, r1);
    __so3_mul(scratch, r1_inv_buf, r2);
    __so3_rotation_vector(m, scratch);
    double angle = __vo_norm(m, 3);
    if (angle == 0) {
        ret->axis[0] = 1.0;
        ret->axis[1] = 0.0;
        ret->axis[2] = 0.0;
    }
    else {
        __vo_mul(ret->axis, m, 1/angle, 3);
    }
    ret->angle = angle;
}

/**
 * Returns a function of one parameter u that interpolates linearly between the two rotations R1 and R2. After f(u) is constructed, calling f(u) is about 2x faster than calling interpolate(R1,R2,u).
 */
PyObject* so3_interpolator(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

/**
 * Returns the determinant of the 3x3 matrix R
 */
PyObject* so3_det(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __so3_det(const double* rot) {
    return rot[0] * rot[4] * rot[8]
         + rot[3] * rot[7] * rot[2]
         + rot[6] * rot[1] * rot[5]
         - rot[0] * rot[7] * rot[5]
         - rot[3] * rot[1] * rot[8]
         - rot[6] * rot[4] * rot[2];
}

/**
 * Returns true if R is a rotation matrix, i.e. is orthogonal to the given tolerance and has + determinant
 */
PyObject* so3_is_rotation(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline int __so3_is_rotation(double* rot, double tol) {
    double scratch1[9];
    double scratch2[9];
    __so3_inv(scratch1, rot);
    __so3_mul(scratch2, rot, scratch1);
    __vo_subv(scratch1, scratch2, SO3_ID, 9);
    for (int i = 0; i < 9; ++i) {
        if (fabs(scratch1[i]) > tol) return 0;
    }
    return __so3_det(scratch1) > 0;
}

/**
 * Returns a uniformly distributed rotation matrix.
 */
PyObject* so3_sample(PyObject* self, PyObject* args);

inline void __so3_sample(double* result) {
    double q[4];
    q[0] = sampleNormal(q+1);
    q[2] = sampleNormal(q+3);
    __vo_unit(q, q, 1e-5, 4);
    double theta = acos(q[3]) * 2;
    if (fabs(theta) < 1e-8) {
        q[0] = 0;
        q[1] = 0;
        q[2] = 0;
    }
    else {
        __vo_unit(q, q, 1e-5, 3);
    }
    __so3_rotation(result, q, theta);
}
