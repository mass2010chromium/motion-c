#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "utils.h"

#include "vectorops.h"

#include <math.h>

PyDoc_STRVAR(py_vectorops_add_doc, "Adds one or more vectors.");
PyDoc_STRVAR(py_vectorops_madd_doc, "Return a+c*b where a and b are vectors.");
PyDoc_STRVAR(py_vectorops_sub_doc, "Subtract a vector b from a, or subtract a scalar");
PyDoc_STRVAR(py_vectorops_mul_doc, "Multiply a vector either elementwise with another vector, or with a scalar.");
PyDoc_STRVAR(py_vectorops_div_doc, "Elementwise division with another vector, or with a scalar.");
PyDoc_STRVAR(py_vectorops_maximum_doc, "Elementwise max");
PyDoc_STRVAR(py_vectorops_minimum_doc, "Elementwise min");
PyDoc_STRVAR(py_vectorops_dot_doc, "Dot product.");
PyDoc_STRVAR(py_vectorops_normSquared_doc, "Returns the norm of a, squared.");
PyDoc_STRVAR(py_vectorops_norm_doc, "L2 norm");
PyDoc_STRVAR(py_vectorops_unit_doc, "Returns the unit vector in the direction a.  If the norm of a is less than epsilon, a is left unchanged.");
PyDoc_STRVAR(py_vectorops_norm_L1_doc, "L1 norm");
PyDoc_STRVAR(py_vectorops_norm_Linf_doc, "L-infinity norm");
PyDoc_STRVAR(py_vectorops_distanceSquared_doc, "Squared L2 distance");
PyDoc_STRVAR(py_vectorops_distance_doc, "L2 distance");
PyDoc_STRVAR(py_vectorops_cross_doc, "Cross product between a 3-vector or a 2-vector");
PyDoc_STRVAR(py_vectorops_interpolate_doc, "Linear interpolation between a and b");

static PyMethodDef py_vectoropsMethods[] = {
    {"add", (PyCFunction) py_vectorops_add, METH_FASTCALL, py_vectorops_add_doc},
    {"madd", (PyCFunction) py_vectorops_madd, METH_FASTCALL, py_vectorops_madd_doc},
    {"sub", (PyCFunction) py_vectorops_sub, METH_FASTCALL, py_vectorops_sub_doc},
    {"mul", (PyCFunction) py_vectorops_mul, METH_FASTCALL, py_vectorops_mul_doc},
    {"div", (PyCFunction) py_vectorops_div, METH_FASTCALL, py_vectorops_div_doc},
    {"maximum", (PyCFunction) py_vectorops_maximum, METH_FASTCALL, py_vectorops_maximum_doc},
    {"minimum", (PyCFunction) py_vectorops_minimum, METH_FASTCALL, py_vectorops_minimum_doc},
    {"dot", (PyCFunction) py_vectorops_dot, METH_FASTCALL, py_vectorops_dot_doc},
    {"normSquared", (PyCFunction) py_vectorops_normSquared, METH_FASTCALL, py_vectorops_normSquared_doc},
    {"norm", (PyCFunction) py_vectorops_norm, METH_FASTCALL, py_vectorops_norm_doc},
    {"unit", (PyCFunction) py_vectorops_unit, METH_FASTCALL, py_vectorops_unit_doc},
    {"norm_L2", (PyCFunction) py_vectorops_norm, METH_FASTCALL, py_vectorops_norm_doc},
    {"norm_L1", (PyCFunction) py_vectorops_norm_L1, METH_FASTCALL, py_vectorops_norm_L1_doc},
    {"norm_Linf", (PyCFunction) py_vectorops_norm_Linf, METH_FASTCALL, py_vectorops_norm_Linf_doc},
    {"distanceSquared", (PyCFunction) py_vectorops_distanceSquared, METH_FASTCALL, py_vectorops_distanceSquared_doc},
    {"distance", (PyCFunction) py_vectorops_distance, METH_FASTCALL, py_vectorops_distance_doc},
    {"cross", (PyCFunction) py_vectorops_cross, METH_FASTCALL, py_vectorops_cross_doc},
    {"interpolate", (PyCFunction) py_vectorops_interpolate, METH_FASTCALL, py_vectorops_interpolate_doc},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef py_vectoropsModule = {
    PyModuleDef_HEAD_INIT,
    "motion.vectorops",
    NULL,   // Documentation
    -1,     /* size of per-interpreter state of the module,
                or -1 if the module keeps state in global variables. */
    py_vectoropsMethods
};

PyMODINIT_FUNC PyInit_vectorops() {
    return PyModule_Create(&py_vectoropsModule);
}

/**
 * Adds one or more vectors.
 */
PyObject* py_vectorops_add(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs <= 0) {
        PyErr_SetString(PyExc_TypeError, "Not enough arguments (expected 1+)");
        return NULL;
    }
#endif
    PyObject* first = args[0];

    Py_ssize_t n = PyObject_Length(first);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
#endif
    double buffer[n];
    if (list_to_vector(first, buffer)) {
        return NULL;
    }
    double buffer2[n];
    for (int i = 1; i < nargs; ++i) {
        PyObject* element = args[i];

#ifdef MOTION_DEBUG
        Py_ssize_t n2 = PyObject_Length(element);
        if (n != n2) {
            PyErr_SetString(PyExc_ValueError, "vector dimensions differ");
            return NULL;
        }
#endif
        if (list_to_vector(element, buffer2)) {
            return NULL;
        }
        vo_addv(buffer, buffer, buffer2, n);
    }
    return vector_to_list(buffer, n);
}

/**
 * Return a+c*b where a and b are vectors.
 */
PyObject* py_vectorops_madd(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs != 3) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 3)");
        return NULL;
    }
    if (!(PyFloat_Check(args[2]) || PyLong_Check(args[2]))) {
        PyErr_SetString(PyExc_TypeError, "Expected number for args[2]");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    PyObject* b = args[1];
    double c = PyFloat_AsDouble(args[2]);
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
    Py_ssize_t n2 = PyObject_Length(b);
    if (n != n2) {
        PyErr_SetString(PyExc_ValueError, "vector dimensions differ");
        return NULL;
    }
#endif
    double buffer[n];
    double buffer2[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    if (list_to_vector(b, buffer2)) {
        return NULL;
    }
    vo_madd(buffer, buffer, buffer2, c, n);
    return vector_to_list(buffer, n);
}

#ifdef MOTION_DEBUG
#define VO_BINOP(name) \
PyObject* py_vectorops_ ## name (PyObject* self, PyObject* const* args, Py_ssize_t nargs) { \
    if (nargs != 2) { \
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 2)"); \
        return NULL; \
    } \
    PyObject* a = args[0]; \
    PyObject* b = args[1]; \
    Py_ssize_t n = PyObject_Length(a); \
    if (n < 0) { \
        PyErr_SetString(PyExc_TypeError, "object has no length"); \
        return NULL; \
    } \
    double buffer[n]; \
    if (list_to_vector(a, buffer)) { \
        return NULL; \
    } \
    if (PyFloat_Check(b) || PyLong_Check(b)) { \
        vo_ ## name (buffer, buffer, PyFloat_AsDouble(b), n); \
        return vector_to_list(buffer, n); \
    } \
    double buffer2[n]; \
    Py_ssize_t n2 = PyObject_Length(b); \
    if (n != n2) { \
        PyErr_SetString(PyExc_ValueError, "vector dimensions differ"); \
        return NULL; \
    } \
    if (list_to_vector(b, buffer2)) { \
        return NULL; \
    } \
    vo_ ## name ## v (buffer, buffer, buffer2, n); \
    return vector_to_list(buffer, n); \
}
#else
#define VO_BINOP(name) \
PyObject* py_vectorops_ ## name (PyObject* self, PyObject* const* args, Py_ssize_t nargs) { \
    PyObject* a = args[0]; \
    PyObject* b = args[1]; \
    Py_ssize_t n = PyObject_Length(a); \
    double buffer[n]; \
    if (list_to_vector(a, buffer)) { \
        return NULL; \
    } \
    if (PyFloat_Check(b) || PyLong_Check(b)) { \
        vo_ ## name (buffer, buffer, PyFloat_AsDouble(b), n); \
        return vector_to_list(buffer, n); \
    } \
    double buffer2[n]; \
    if (list_to_vector(b, buffer2)) { \
        return NULL; \
    } \
    vo_ ## name ## v (buffer, buffer, buffer2, n); \
    return vector_to_list(buffer, n); \
}
#endif

/**
 * Subtract a vector b from a, or subtract a scalar
 */
VO_BINOP(sub)

/**
 * Multiply a vector either elementwise with another vector, or with a scalar.
 */
VO_BINOP(mul)

/**
 * Elementwise division with another vector, or with a scalar.
 */
VO_BINOP(div)

/**
 * Elementwise max
 */
VO_BINOP(maximum)

/**
 * Elementwise min
 */
VO_BINOP(minimum)

/**
 * Dot product.
 */
PyObject* py_vectorops_dot(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs != 2) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 2)");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    PyObject* b = args[1];
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
    Py_ssize_t n2 = PyObject_Length(b);
    if (n != n2) {
        PyErr_SetString(PyExc_ValueError, "vector dimensions differ");
        return NULL;
    }
#endif
    double buffer[n];
    double buffer2[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    if (list_to_vector(b, buffer2)) {
        return NULL;
    }
    double ret = vo_dot(buffer, buffer2, n);
    return PyFloat_FromDouble(ret);
}

/**
 * Returns the norm of a, squared.
 */
PyObject* py_vectorops_normSquared(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs != 1) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 1)");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
#endif
    double buffer[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    double ret = vo_dot(buffer, buffer, n);
    return PyFloat_FromDouble(ret);
}

/**
 * L2 norm
 */
PyObject* py_vectorops_norm(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
    PyFloatObject* ret = (PyFloatObject*) py_vectorops_normSquared(self, args, nargs);
    ret->ob_fval = sqrt(ret->ob_fval);
    return (PyObject*) ret;
}

/**
 * Returns the unit vector in the direction a.  If the norm of
 * a is less than epsilon, a is left unchanged.
 */
PyObject* py_vectorops_unit(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs <= 0 || nargs > 2) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 1-2)");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    double eps;
    if (nargs == 1) {
        eps = 1e-5;
    }
    else {
#ifdef MOTION_DEBUG
        if (!(PyFloat_Check(args[1]) || PyLong_Check(args[1]))) {
            PyErr_SetString(PyExc_TypeError, "Expected number for args[1]");
            return NULL;
        }
#endif
        eps = PyFloat_AsDouble(args[1]);
    }
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
#endif
    double buffer[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    double norm_val = vo_norm(buffer, n);
    if (norm_val > eps) {
        vo_mul(buffer, buffer, 1/norm_val, n);
    }
    return vector_to_list(buffer, n);
}

/**
 * L1 norm
 */
PyObject* py_vectorops_norm_L1(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs != 1) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 1)");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
#endif
    double buffer[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    double ret = vo_norm_L1(buffer, n);
    return PyFloat_FromDouble(ret);
}

/**
 * L-infinity norm
 */
PyObject* py_vectorops_norm_Linf(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs != 1) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 1)");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
#endif
    double buffer[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    double ret = vo_norm_Linf(buffer, n);
    return PyFloat_FromDouble(ret);
}

/**
 * Squared L2 distance
 */
PyObject* py_vectorops_distanceSquared(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs != 2) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 2)");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    PyObject* b = args[1];
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
    Py_ssize_t n2 = PyObject_Length(b);
    if (n != n2) {
        PyErr_SetString(PyExc_ValueError, "vector dimensions differ");
        return NULL;
    }
#endif
    double buffer[n];
    double buffer2[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    if (list_to_vector(b, buffer2)) {
        return NULL;
    }
    double ret = vo_distanceSquared(buffer, buffer2, n);
    return PyFloat_FromDouble(ret);
}

/**
 * L2 distance
 */
PyObject* py_vectorops_distance(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
    PyFloatObject* ret = (PyFloatObject*) py_vectorops_distanceSquared(self, args, nargs);
    ret->ob_fval = sqrt(ret->ob_fval);
    return (PyObject*) ret;
}

/**
 * Cross product between a 3-vector or a 2-vector
 */
PyObject* py_vectorops_cross(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs != 2) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 2)");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    PyObject* b = args[1];
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
    Py_ssize_t n2 = PyObject_Length(b);
    if (n != n2) {
        PyErr_SetString(PyExc_ValueError, "vector dimensions differ");
        return NULL;
    }
#endif
    double buffer[n];
    double buffer2[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    if (list_to_vector(b, buffer2)) {
        return NULL;
    }
    if (n == 2) {
        double ret = vo_cross2(buffer, buffer2);
        return PyFloat_FromDouble(ret);
    }
    if (n == 3) {
        double buffer3[n];
        vo_cross3(buffer3, buffer, buffer2);
        return vector_to_list(buffer3, 3);
    }
#ifdef MOTION_DEBUG
    PyErr_SetString(PyExc_ValueError, "crossprod only valid for 2 or 3D");
#endif
    return NULL;
}

/**
 * Linear interpolation between a and b
 */
PyObject* py_vectorops_interpolate(PyObject* self, PyObject* const* args, Py_ssize_t nargs) {
#ifdef MOTION_DEBUG
    if (nargs != 3) {
        PyErr_SetString(PyExc_TypeError, "Wrong Number of arguments (expected 3)");
        return NULL;
    }
#endif
    PyObject* a = args[0];
    PyObject* b = args[1];
#ifdef MOTION_DEBUG
    if (!(PyFloat_Check(args[2]) || PyLong_Check(args[2]))) {
        PyErr_SetString(PyExc_TypeError, "Expected number for args[2]");
        return NULL;
    }
#endif
    double c = PyFloat_AsDouble(args[2]);
    Py_ssize_t n = PyObject_Length(a);
#ifdef MOTION_DEBUG
    if (n < 0) {
        PyErr_SetString(PyExc_TypeError, "object has no length");
        return NULL;
    }
    Py_ssize_t n2 = PyObject_Length(b);
    if (n != n2) {
        PyErr_SetString(PyExc_ValueError, "vector dimensions differ");
        return NULL;
    }
#endif
    double buffer[n];
    double buffer2[n];
    if (list_to_vector(a, buffer)) {
        return NULL;
    }
    if (list_to_vector(b, buffer2)) {
        return NULL;
    }
    vo_interpolate(buffer2, buffer, buffer2, c, n);
    return vector_to_list(buffer2, n);
}
