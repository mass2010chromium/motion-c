#pragma once

#include <Python.h>

#include <math.h>
#include "vectorops.h"
#include "so3.h"

#ifndef MOTION_DEBUG
inline
#endif
PyObject* return_se3(const double* data) {
    PyObject* rot = vector_to_list(data, 9);
#ifdef MOTION_DEBUG
    if (rot == NULL) {
        return NULL;
    }
#endif
    PyObject* trans = vector_to_list(data+9, 3);
#ifdef MOTION_DEBUG
    if (trans == NULL) {
        Py_DECREF(rot);
        return NULL;
    }
#endif
    PyObject* ret = Py_BuildValue("(NN)", rot, trans);
#ifdef MOTION_DEBUG
    if (ret == NULL) {
        Py_DECREF(trans);
        Py_DECREF(rot);
        return NULL;
    }
#endif
    return ret;
}

#ifndef MOTION_DEBUG
inline
#endif
int parse_se3 (double* dest, PyObject* vec) {
#ifdef MOTION_DEBUG
    Py_ssize_t n = PyObject_Length(vec);
#ifdef SE3_STRICT
    if (n != 2)
#else
    if (n < 2)
#endif
    {
        PyErr_SetString(PyExc_ValueError, "Transform expected (R, T)");
        return 1;
    }
#endif
    PyObject* it = PyObject_GetIter(vec);
#ifdef MOTION_DEBUG
    if (it == NULL) {
        return 1;
    }
#endif

    PyObject* curr = PyIter_Next(it);
#ifdef MOTION_DEBUG
    if (curr == NULL) {
        return 1;
    }
#endif
    if (parse_rotation(dest, curr)) {
        Py_DECREF(curr);    // Can't trigger gc cause container still exists. I think.
        Py_DECREF(it);
        return 1;
    }
    Py_DECREF(curr);    // Can't trigger gc cause container still exists. I think.

    curr = PyIter_Next(it);
#ifdef MOTION_DEBUG
    if (curr == NULL) {
        return 1;
    }
#endif
    if (parse_vec3(dest+9, curr)) {
        Py_DECREF(curr);    // Can't trigger gc cause container still exists. I think.
        Py_DECREF(it);
        return 1;
    }
    Py_DECREF(curr);    // Can't trigger gc cause container still exists. I think.
    Py_DECREF(it);
    return 0;
}

const double SE3_ID[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};

/**
 * Returns the identity transformation.
 */
PyObject* se3_identity(PyObject* self, PyObject* args);

/**
 * Returns the inverse of the transformation.
 */
PyObject* se3_inv(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __se3_inv(double* ret, double* transform) {
    __so3_inv(ret, transform);
    __so3_apply(ret+9, ret, transform+9);
    __vo_mul(ret+9, ret+9, -1, 3);
}

/**
 * Applies the transform T to the given point
 */
PyObject* se3_apply(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __se3_apply(double* ret, double* transform, double* point) {
    __so3_apply(ret, transform, point);
    __vo_add(ret, ret, transform+9, 3);
}

/**
 * Applies only the rotation part of T
 */
PyObject* se3_apply_rotation(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

/**
 * Returns the 3x3 rotation matrix corresponding to T's rotation
 */
PyObject* se3_rotation(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

/**
 * Returns a transformation T corresponding to the 3x3 rotation matrix mat
 */
PyObject* se3_from_rotation(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

/**
 * Returns the translation vector corresponding to T's translation
 */
PyObject* se3_translation(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

/**
 * Returns a transformation T that translates points by t
 */
PyObject* se3_from_translation(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

/**
 * Returns the 4x4 homogeneous transform corresponding to T
 */
PyObject* se3_homogeneous(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

/**
 * Returns a T corresponding to the 4x4 homogeneous transform mat
 */
PyObject* se3_from_homogeneous(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

/**
 * Composes two transformations.
 */
PyObject* se3_mul(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __se3_mul(double* ret, double* t1, double* t2) {
    __so3_mul(ret, t1, t2);
    __so3_apply(ret+9, t1, t2+9);
    __vo_add(ret+9, ret+9, t1+9, 3);
}

/**
 * Returns a distance metric between the two transformations. The rotation distance is weighted by Rweight and the translation distance is weighted by tweight
 */
PyObject* se3_distance(PyObject* self, PyObject* args, PyObject* kwargs);

inline double __se3_distance(const double* t1, const double* t2,
                             double rweight, double tweight) {
    return __so3_distance(t1, t2) * rweight
         + __vo_distance(t1+9, t2+9, 3) * tweight;
}

/**
 * Returns a 6D "difference vector" that describes how far T1 is from T2. More precisely, this is the Lie derivative (w,v).
 */
PyObject* se3_error(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __se3_error(double* ret, double* t1, double* t2) {
    __so3_error(ret, t1, t2);
    __vo_subv(ret+3, t1+9, t2+9, 3);
}

/**
 * Interpolate linearly between the two transformations T1 and T2.
 */
PyObject* se3_interpolate(PyObject* self, PyObject* const* args, Py_ssize_t nargs);

inline void __se3_interpolate(double* ret, double* t1, double* t2, double u) {
    __so3_interpolate(ret, t1, t2, u);
    __vo_interpolate(ret+3, t1+9, t2+9, u, 3);
}

typedef struct {
    double rot[9];
    double axis[3];
    double angle;
    double t1[3];
    double dt[3];
} se3_interpolator_t;

inline void __se3_interpolator_init(se3_interpolator_t* ret, double* t1, double* t2) {
    __so3_interpolator_init((so3_interpolator_t*) ret, t1, t2);
    memcpy(ret->t1, t1+9, sizeof(double)*3);
    __vo_subv(ret->dt, t2+9, t1+9, 3);
}

/**
 * Returns a function of one parameter u that interpolates linearly
 * between the two transformations T1 and T2.
 * 
 * After f(u) is constructed, calling f(u) is about 2x faster than calling interpolate(T1,T2,u).
 */
PyObject* se3_interpolator(PyObject* self, PyObject* const* args, Py_ssize_t nargs);
