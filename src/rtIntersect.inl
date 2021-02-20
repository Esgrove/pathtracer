static __forceinline void FINDMINMAX(float x0, float x1, float x2, float& min, float& max) {
    min = max = x0;
    if (x1 < min)
        min = x1;
    if (x1 > max)
        max = x1;
    if (x2 < min)
        min = x2;
    if (x2 > max)
        max = x2;
}

static __forceinline void CROSS(float dest[3], const float v1[3], const float v2[3]) {
    dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
    dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
    dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

static __forceinline float DOT(const float v1[3], const float v2[3]) {
    return (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
}

static __forceinline void SUB(float dest[3], const float v1[3], const float v2[3]) {
    dest[0] = v1[0] - v2[0];
    dest[1] = v1[1] - v2[1];
    dest[2] = v1[2] - v2[2];
}

static __forceinline bool planeBoxOverlap(float normal[3], float d, const float maxbox[3]) {
    int q;
    float vmin[3], vmax[3];
    for (q = 0; q <= 2; q++) {
        if (normal[q] > 0.0f) {
            vmin[q] = -maxbox[q];
            vmax[q] = maxbox[q];
        } else {
            vmin[q] = maxbox[q];
            vmax[q] = -maxbox[q];
        }
    }
    if (DOT(normal, vmin) + d > 0.0f)
        return false;
    if (DOT(normal, vmax) + d >= 0.0f)
        return true;
    return false;
}

/*======================== 0-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb) \
    p0 = a * v0[1] - b * v0[2]; \
    p2 = a * v2[1] - b * v2[2]; \
    if (p0 < p2) { \
        min = p0; \
        max = p2; \
    } else { \
        min = p2; \
        max = p0; \
    } \
    rad = fa * boxhalfsize[1] + fb * boxhalfsize[2]; \
    if (min > rad || max < -rad) \
        return false;

#define AXISTEST_X2(a, b, fa, fb) \
    p0 = a * v0[1] - b * v0[2]; \
    p1 = a * v1[1] - b * v1[2]; \
    if (p0 < p1) { \
        min = p0; \
        max = p1; \
    } else { \
        min = p1; \
        max = p0; \
    } \
    rad = fa * boxhalfsize[1] + fb * boxhalfsize[2]; \
    if (min > rad || max < -rad) \
        return false;

/*======================== 1-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb) \
    p0 = -a * v0[0] + b * v0[2]; \
    p2 = -a * v2[0] + b * v2[2]; \
    if (p0 < p2) { \
        min = p0; \
        max = p2; \
    } else { \
        min = p2; \
        max = p0; \
    } \
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[2]; \
    if (min > rad || max < -rad) \
        return false;

#define AXISTEST_Y1(a, b, fa, fb) \
    p0 = -a * v0[0] + b * v0[2]; \
    p1 = -a * v1[0] + b * v1[2]; \
    if (p0 < p1) { \
        min = p0; \
        max = p1; \
    } else { \
        min = p1; \
        max = p0; \
    } \
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[2]; \
    if (min > rad || max < -rad) \
        return false;

/*======================== 2-tests ========================*/

#define AXISTEST_Z12(a, b, fa, fb) \
    p1 = a * v1[0] - b * v1[1]; \
    p2 = a * v2[0] - b * v2[1]; \
    if (p2 < p1) { \
        min = p2; \
        max = p1; \
    } else { \
        min = p1; \
        max = p2; \
    } \
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[1]; \
    if (min > rad || max < -rad) \
        return false;

#define AXISTEST_Z0(a, b, fa, fb) \
    p0 = a * v0[0] - b * v0[1]; \
    p1 = a * v1[0] - b * v1[1]; \
    if (p0 < p1) { \
        min = p0; \
        max = p1; \
    } else { \
        min = p1; \
        max = p0; \
    } \
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[1]; \
    if (min > rad || max < -rad) \
        return false;

static __forceinline bool
    triBoxOverlap(const float boxcenter[3], const float boxhalfsize[3], const float vv0[3], const float vv1[3], const float vv2[3]) {
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
    /*       this gives 3x3=9 more tests */
    float v0[3], v1[3], v2[3];
    float min, max, d, p0, p1, p2, rad, fex, fey, fez;
    float normal[3], e0[3], e1[3], e2[3];

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */
    SUB(v0, vv0, boxcenter);
    SUB(v1, vv1, boxcenter);
    SUB(v2, vv2, boxcenter);

    /* compute triangle edges */
    SUB(e0, v1, v0); /* tri edge 0 */
    SUB(e1, v2, v1); /* tri edge 1 */
    SUB(e2, v0, v2); /* tri edge 2 */

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */
    fex = fabsf(e0[0]);
    fey = fabsf(e0[1]);
    fez = fabsf(e0[2]);
    AXISTEST_X01(e0[2], e0[1], fez, fey);
    AXISTEST_Y02(e0[2], e0[0], fez, fex);
    AXISTEST_Z12(e0[1], e0[0], fey, fex);

    fex = fabsf(e1[0]);
    fey = fabsf(e1[1]);
    fez = fabsf(e1[2]);
    AXISTEST_X01(e1[2], e1[1], fez, fey);
    AXISTEST_Y02(e1[2], e1[0], fez, fex);
    AXISTEST_Z0(e1[1], e1[0], fey, fex);

    fex = fabsf(e2[0]);
    fey = fabsf(e2[1]);
    fez = fabsf(e2[2]);
    AXISTEST_X2(e2[2], e2[1], fez, fey);
    AXISTEST_Y1(e2[2], e2[0], fez, fex);
    AXISTEST_Z12(e2[1], e2[0], fey, fex);

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in 0-direction */
    FINDMINMAX(v0[0], v1[0], v2[0], min, max);
    if (min > boxhalfsize[0] || max < -boxhalfsize[0])
        return false;

    /* test in 1-direction */
    FINDMINMAX(v0[1], v1[1], v2[1], min, max);
    if (min > boxhalfsize[1] || max < -boxhalfsize[1])
        return false;

    /* test in 2-direction */
    FINDMINMAX(v0[2], v1[2], v2[2], min, max);
    if (min > boxhalfsize[2] || max < -boxhalfsize[2])
        return false;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    CROSS(normal, e0, e1);
    d = -DOT(normal, v0); /* plane eq: normal.x+d=0 */
    if (!planeBoxOverlap(normal, d, boxhalfsize))
        return false;

    return true; /* box and triangle overlaps */
}

__forceinline static bool
    intersect_triangle1(const float orig[3], const float dir[3], const float vert0[3], const float vert1[3], const float vert2[3], float& t, float& u, float& v) {
    static const float EPSILON = 0.000000001f;

    float edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
    float det, inv_det;

    SUB(edge1, vert1, vert0);
    SUB(edge2, vert2, vert0);
    CROSS(pvec, dir, edge2);
    det = DOT(edge1, pvec);

    if (det > EPSILON) {
        SUB(tvec, orig, vert0);
        u = DOT(tvec, pvec);

        if (u < 0.0 || u > det)
            return false;

        CROSS(qvec, tvec, edge1);
        v = DOT(dir, qvec);
        if (v < 0.0 || u + v > det)
            return false;
    } else if (det < -EPSILON) {
        SUB(tvec, orig, vert0);
        u = DOT(tvec, pvec);

        if (u > 0.0 || u < det)
            return false;

        CROSS(qvec, tvec, edge1);
        v = DOT(dir, qvec);
        if (v > 0.0 || u + v < det)
            return false;
    } else
        return false;

    inv_det = 1.f / det;

    t = DOT(edge2, qvec) * inv_det;
    u *= inv_det;
    v *= inv_det;

    return true;
}

__forceinline static bool
    intersect_triangle2(const float orig[3], const float dir[3], const float vert0[3], const float vert1[3], const float vert2[3], float& t, float& u, float& v) {
    static const float EPSILON = 0.000001f;

    float edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
    float det, inv_det;

    SUB(edge1, vert1, vert0);
    SUB(edge2, vert2, vert0);
    CROSS(pvec, dir, edge2);
    det = DOT(edge1, pvec);

    if (det > 0.f) {
        SUB(tvec, orig, vert0);
        u = DOT(tvec, pvec);

        CROSS(qvec, tvec, edge1);
        v = DOT(dir, qvec);
    } else {
        SUB(tvec, orig, vert0);
        u = DOT(tvec, pvec);

        CROSS(qvec, tvec, edge1);
        v = DOT(dir, qvec);
    }

    inv_det = 1.f / det;

    t = DOT(edge2, qvec) * inv_det;
    u *= inv_det;
    v *= inv_det;

    return true;
}
