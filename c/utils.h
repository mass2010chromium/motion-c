#pragma once

//#define CASE_3_6

#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <math.h>

#ifdef MOTION_DEBUG
#include <assert.h>
#define _assert assert
#else
#define _assert(x) ((void) (x))
#endif

/**
 * Create a new list from a C array.
 * Return value: new reference
 */
#ifndef MOTION_DEBUG
inline
#endif
PyObject* vector_to_list(const double* vec, Py_ssize_t size) {
    PyObject* ret = PyList_New(size);
#ifdef MOTION_DEBUG
    if (ret == NULL) {
        return NULL;
    }
#endif
    for (int i = 0; i < size; ++i) {
        PyObject* item = PyFloat_FromDouble(vec[i]);
#ifdef MOTION_DEBUG
        if (item == NULL) {
            Py_DECREF(ret);
            return NULL;
        }
#endif
        PyList_SET_ITEM(ret, i, item);
    }
    return ret;
}

/**
 * Parse a list into a buffer.
 * WILL FAIL if the object is malicious and
 * passes a object with spoofed len() lol
 */
#ifndef MOTION_DEBUG
inline
#endif
int list_to_vector(PyObject* list, double* vec) {
    PyObject* it = PyObject_GetIter(list);
#ifdef MOTION_DEBUG
    if (it == NULL) {
        return 1;
    }
#endif

    PyObject* curr;
    double* head = vec;
    while ((curr = PyIter_Next(it))) {
#ifdef MOTION_DEBUG
        if (curr == NULL) {
            return 1;
        }
        if (!(PyFloat_Check(curr) || PyLong_Check(curr))) {
            Py_DECREF(curr);
            Py_DECREF(it);
            PyErr_SetString(PyExc_TypeError, "expected float array");
            return 1;
        }
#endif
        *(head++) = PyFloat_AsDouble(curr); // Works for long too hmm
        Py_DECREF(curr);    // Can't trigger gc cause container still exists. I think.
    }
    Py_DECREF(it);
    return 0;
}

/**
 * Returns NULL on failure, or the iterator handle on success.
 */
#ifndef MOTION_DEBUG
inline
#endif
PyObject* list_to_vector_n(PyObject* list, double* vec, int n) {
    PyObject* it = PyObject_GetIter(list);
#ifdef MOTION_DEBUG
    if (it == NULL) {
        return NULL;
    }
#endif

    PyObject* curr;
    double* head = vec;
    for (int i = 0; i < n; ++i) {
        curr = PyIter_Next(it);
#ifdef MOTION_DEBUG
        if (curr == NULL) {
            return NULL;
        }
        if (!(PyFloat_Check(curr) || PyLong_Check(curr))) {
            Py_DECREF(curr);
            Py_DECREF(it);
            PyErr_SetString(PyExc_TypeError, "expected float array");
            return NULL;
        }
#endif
        *(head++) = PyFloat_AsDouble(curr); // Works for long too hmm
        Py_DECREF(curr);    // Can't trigger gc cause container still exists. I think.
    }
    return it;
}

/**
 * Signum, written to match Klampt
 */
inline int signum(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

/**
 * Clamp to +1 and -1 for trig
 */
inline double clamp_unity(double x) {
    if (x > 1.0) return 1.0;
    if (x < -1.0) return -1.0;
    return x;
}

/**
 * Gaussian distribution stolen from online.
 * Uses the Box-Muller method to generate values from
 * a gaussian distribution with mean 0 and stdev 1.
 * Source: https://stackoverflow.com/a/10645091
 * Optionally returns a second value through the pointer.
 */
double sampleNormal(double* b) {
    double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
    double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
    double r = u * u + v * v;
    if (r == 0 || r > 1) return sampleNormal(b);
    double c = sqrt(-2 * log(r) / r);
    if (b) *b = v * c;
    return u * c;
}
