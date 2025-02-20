#pragma once

#include <motionlib/vectorops.h>
#include <math.h>

// Angles must be normalized from 0 to 2pi!
static inline motion_dtype geolib_angle_err(motion_dtype a, motion_dtype b) {
    motion_dtype diff = a - b;
    if (diff < -M_PI) { return diff + 2*M_PI; }
    if (diff > M_PI) { return diff - 2*M_PI; }
    return diff;
}

// Angles must be normalized from 0 to 2pi!
static inline bool geolib_angle_range_contains(motion_dtype a, motion_dtype b, motion_dtype x) {
    if (b > a) {
        return x > a && x < b;
    }
    return x > a || x < b;
}

// Returns first intersection (closer to p1)
// Or if no intersection, returns false.
static inline bool geolib_circle_intersection(const motion_dtype* center, motion_dtype radius,
                                    const motion_dtype* p1, const motion_dtype* p2,
                                    motion_dtype eps, motion_dtype* ret) {
    motion_dtype delta[2];
    vo_subv(delta, p2, p1, 2);

    motion_dtype pos_to_p1[2];
    vo_subv(pos_to_p1, center, p1, 2);
    motion_dtype segment_length = vo_norm(delta, 2);
    if (segment_length < eps) {
        motion_dtype err = vo_distance(center, p1, 2) - radius;
        if (err < 0) { err = -err; }
        if (err < eps) {
            ret[0] = p1[0]; ret[1] = p1[1];
            return true;
        }
        return false;
    }

    /* Centering on the circle, let p = {x0, y0} be the position of p1.
     * Let v = {vx, vy} represent the unit vector along the line.
     * Parametrize distance along the line segment as t, starting from x.
     * The constraint becomes:
     *
     *   r = sqrt((x0 + t.vx)^2 + (y0 + t.vy)^2).
     *
     * Manipulating,
     *
     *   r^2 = x0^2 + y0^2 + 2t(x0.vx + y0.vy) + t^2(vx^2 + vy^2).
     *
     * Per the quadratic formula, a=1, b=2(x0.vx + y0.vy), c=(x0^2 + y0^2 - r^2).
     */
    motion_dtype segment_dir[2];
    vo_unit(segment_dir, delta, eps, 2);
    motion_dtype b = -2*vo_dot(pos_to_p1, segment_dir, 2);
    motion_dtype c = vo_normSquared(pos_to_p1, 2) - radius*radius;

    motion_dtype discriminant = b*b - 4*c;
    if (discriminant < 0) { return false; }
    motion_dtype t1 = (-b - sqrt(discriminant)) / 2;
    if (t1 > segment_length+eps) {
        // No hope for t2. This was the one on the p1 side.
        return false;
    }
    if (t1 > 0-eps) {
        // Return point corresponding to t1, since t1 is within the bounds.
        vo_madd(ret, p1, segment_dir, t1, 2);
        return true;
    }
    // Check second intersection.
    motion_dtype t2 = (-b + sqrt(discriminant)) / 2;
    if (t2 > 0-eps && t2 < segment_length+eps) {
        vo_madd(ret, p1, segment_dir, t2, 2);
        return true;
    }
    return false;
}

static inline motion_dtype geolib_point_segment_distance(const motion_dtype* pos,
                                    const motion_dtype* p1, const motion_dtype* p2,
                                    motion_dtype eps) {

    motion_dtype delta[2];
    vo_subv(delta, p2, p1, 2);

    motion_dtype pos_to_p1[2];
    vo_subv(pos_to_p1, pos, p1, 2);
    motion_dtype segment_length = vo_norm(delta, 2);
    if (segment_length < eps) {
        return vo_norm(pos_to_p1, 2);
    }

    motion_dtype distance;
    motion_dtype segment_dir[2];
    vo_unit(segment_dir, delta, eps, 2);
    motion_dtype in_line_check = vo_dot(pos_to_p1, segment_dir, 2);
    if (in_line_check < 0) {
        return vo_norm(pos_to_p1, 2);
    }
    else if (in_line_check > segment_length) {
        return vo_distance(pos, p2, 2);
    }
    else {
        motion_dtype line_distance = vo_cross2(pos_to_p1, segment_dir);
        if (line_distance < 0) return -line_distance;
        return line_distance;
    }
}
