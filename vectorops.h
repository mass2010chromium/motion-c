#pragma once

#include <Python.h>

#include <math.h>

/**
 * Adds one or more vectors.
 */
PyObject* vectorops_add(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __vo_add(double* dest, double* a, double* b, int len) {
    switch(len) {
#ifdef CASE_3_6
        case 6:
            dest[5] = a[5] + b[5];
            dest[4] = a[4] + b[4];
            dest[3] = a[3] + b[3];
        case 3:
            dest[2] = a[2] + b[2];
            dest[1] = a[1] + b[1];
            dest[0] = a[0] + b[0];
            return;
#endif
        default:
            for (int i = 0; i < len; ++i) {
                dest[i] = a[i] + b[i];
            }
    }
}

/**
 * Return a+c*b where a and b are vectors.
 */
PyObject* vectorops_madd(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __vo_madd(double* dest, double* a, double* b, double c, int len) {
    switch(len) {
#ifdef CASE_3_6
        case 6:
            dest[5] = a[5] + c*b[5];
            dest[4] = a[4] + c*b[4];
            dest[3] = a[3] + c*b[3];
        case 3:
            dest[2] = a[2] + c*b[2];
            dest[1] = a[1] + c*b[1];
            dest[0] = a[0] + c*b[0];
            return;
#endif
        default:
            for (int i = 0; i < len; ++i) {
                dest[i] = a[i] + c*b[i];
            }
    }
}

#ifdef CASE_3_6 
#define __VO_BINOP_DECL(name, op) \
PyObject* vectorops_ ## name (PyObject* self, PyObject* const* args, Py_ssize_t nargs); \
 \
inline void __vo_ ## name (double* dest, double* a, double b, int len) { \
    switch(len) { \
        case 6: \
            dest[5] = op(a[5], b); \
            dest[4] = op(a[4], b); \
            dest[3] = op(a[3], b); \
        case 3: \
            dest[2] = op(a[2], b); \
            dest[1] = op(a[1], b); \
            dest[0] = op(a[0], b); \
            return; \
        default: \
            for (int i = 0; i < len; ++i) { \
                dest[i] = op(a[i], b); \
            } \
    } \
} \
 \
inline void __vo_ ## name ## v (double* dest, double* a, double* b, int len) { \
    switch(len) { \
        case 6: \
            dest[5] = op(a[5], b[5]); \
            dest[4] = op(a[4], b[4]); \
            dest[3] = op(a[3], b[3]); \
        case 3: \
            dest[2] = op(a[2], b[2]); \
            dest[1] = op(a[1], b[1]); \
            dest[0] = op(a[0], b[0]); \
            return; \
        default: \
            for (int i = 0; i < len; ++i) { \
                dest[i] = op(a[i], b[i]); \
            } \
    } \
}

#else
#define __VO_BINOP_DECL(name, op) \
PyObject* vectorops_ ## name (PyObject* self, PyObject* args); \
 \
inline void __vo_ ## name (double* dest, double* a, double b, int len) { \
    for (int i = 0; i < len; ++i) { \
        dest[i] = op(a[i], b); \
    } \
} \
 \
inline void __vo_ ## name ## v (double* dest, double* a, double* b, int len) { \
    for (int i = 0; i < len; ++i) { \
        dest[i] = op(a[i], b[i]); \
    } \
}

#endif

/**
 * Subtract a vector b from a, or subtract a scalar
 */
#define __VO_MINUS(a, b) (a) - (b)
__VO_BINOP_DECL(sub, __VO_MINUS)

/**
 * Multiply a vector either elementwise with another vector, or with a scalar.
 */
#define __VO_TIMES(a, b) (a) * (b)
__VO_BINOP_DECL(mul, __VO_TIMES)

/**
 * Elementwise division with another vector, or with a scalar.
 */
#define __VO_DIVIDE(a, b) (a) / (b)
__VO_BINOP_DECL(div, __VO_DIVIDE)

/**
 * Elementwise max
 */
// Lol not safe but its just for my usage anyway
#define __VO_MAX(a, b) ((a) > (b) ? (a) : (b))
__VO_BINOP_DECL(maximum, __VO_MAX)

/**
 * Elementwise min
 */
// Lol not safe but its just for my usage anyway
#define __VO_MIN(a, b) ((a) < (b) ? (a) : (b))
__VO_BINOP_DECL(minimum, __VO_MIN)

/**
 * Dot product.
 */
PyObject* vectorops_dot(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __vo_dot(const double* a, const double* b, int len) {
    double res = 0;
    switch(len) {
#ifdef CASE_3_6
        case 6:
            res += a[5] * b[5];
            res += a[4] * b[4];
            res += a[3] * b[3];
        case 3:
            res += a[2] * b[2];
            res += a[1] * b[1];
            res += a[0] * b[0];
            return res;
#endif
        default:
            for (int i = 0; i < len; ++i) {
                res += a[i] * b[i];
            }
            return res;
    }
}

/**
 * Returns the norm of a, squared.
 */
PyObject* vectorops_normSquared(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __vo_normSquared(const double* a, int len) {
    return __vo_dot(a, a, len);
}

/**
 * L2 norm
 */
PyObject* vectorops_norm(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __vo_norm(const double* a, int len) {
    return sqrt(__vo_normSquared(a, len));
}

/**
 * Returns the unit vector in the direction a.  If the norm of
 * a is less than epsilon, a is left unchanged.
 */
PyObject* vectorops_unit(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __vo_unit(double* dest, double* a, double eps, int len) {
    double norm = __vo_norm(a, len);
    if (norm > eps) {
        double norm_recip = 1 / norm;
        for (int i = 0; i < len; ++i) {
            dest[i] = a[i] * norm_recip;
        }
    }
    else {
        for (int i = 0; i < len; ++i) {
            dest[i] = a[i];
        }
    }
}

/**
 * L1 norm
 */
PyObject* vectorops_norm_L1(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __vo_norm_L1(const double* a, int len) {
    double res = 0;
    switch(len) {
#ifdef CASE_3_6
        case 6:
            res += fabs(a[5]);
            res += fabs(a[4]);
            res += fabs(a[3]);
        case 3:
            res += fabs(a[2]);
            res += fabs(a[1]);
            res += fabs(a[0]);
            return res;
#endif
        default:
            for (int i = 0; i < len; ++i) {
                res += fabs(a[i]);
            }
            return res;
    }
}

/**
 * L-infinity norm
 */
PyObject* vectorops_norm_Linf(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __vo_norm_Linf(const double* a, int len) {
    double max = 0;
    double tmp;
    switch(len) {
#ifdef CASE_3_6
        case 6:
            if ((tmp = fabs(a[5])) > max) max = tmp;
            if ((tmp = fabs(a[4])) > max) max = tmp;
            if ((tmp = fabs(a[3])) > max) max = tmp;
        case 3:
            if ((tmp = fabs(a[2])) > max) max = tmp;
            if ((tmp = fabs(a[1])) > max) max = tmp;
            if ((tmp = fabs(a[0])) > max) max = tmp;
            return max;
#endif
        default:
            for (int i = 0; i < len; ++i) {
                if ((tmp = fabs(a[i])) > max) max = tmp;
            }
            return max;
    }
}

/**
 * Squared L2 distance
 */
PyObject* vectorops_distanceSquared(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __vo_distanceSquared(const double* a, const double* b, int len) {
    double res = 0;
    switch(len) {
#ifdef CASE_3_6
        case 6:
            res += (a[5] - b[5]) * (a[5] - b[5]);
            res += (a[4] - b[4]) * (a[4] - b[4]);
            res += (a[3] - b[3]) * (a[3] - b[3]);
        case 3:
            res += (a[2] - b[2]) * (a[2] - b[2]);
            res += (a[1] - b[1]) * (a[1] - b[1]);
            res += (a[0] - b[0]) * (a[0] - b[0]);
            return res;
#endif
        default:
            for (int i = 0; i < len; ++i) {
                res += (a[i] - b[i]) * (a[i] - b[i]);
            }
            return res;
    }
}

/**
 * L2 distance
 */
PyObject* vectorops_distance(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __vo_distance(const double* a, const double* b, int len) {
    return sqrt(__vo_distanceSquared(a, b, len));
}

/**
 * Cross product between a 3-vector or a 2-vector
 */
PyObject* vectorops_cross(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline double __vo_cross2(const double* a, const double* b) {
    return a[0] * b[1] - a[1] * b[0];
}

inline void __vo_cross3(double* dest, double* a, double* b) {
    dest[0] = a[1] * b[2] - a[2] * b[1];
    dest[1] = a[2] * b[0] - a[0] * b[2];
    dest[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * Linear interpolation between a and b
 */
PyObject* vectorops_interpolate(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __vo_interpolate(double* dest, double* a, double* b, double u, int n) {
    __vo_subv(dest, b, a, n);
    __vo_madd(dest, a, dest, u, n);
}
