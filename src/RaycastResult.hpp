#pragma once

#include "RTTriangle.hpp"
#include "base/Math.hpp"

#include <limits>

namespace FW {

// Result information of a raycast
struct RaycastResult {
    const RTTriangle* tri;        // The triangle that was hit.
    float             t;          // Hit position is orig + t * dir.
    float             u, v;       // Barycentric coordinates at the hit triangle.
    Vec3f             point;      // Hit position.
    Vec3f             orig, dir;  // The traced ray. Convenience for tracing and visualization. This is not strictly needed.
    Vec2f             delta_size;

    RaycastResult(const RTTriangle* tri, float t, float u, float v, Vec3f point, const Vec3f& orig, const Vec3f& dir)
        : tri(tri)
        , t(t)
        , u(u)
        , v(v)
        , point(point)
        , orig(orig)
        , dir(dir) {}

    RaycastResult() : tri(nullptr), t(std::numeric_limits<float>::max()), u(), v(), point(), orig(), dir() {}

    inline operator bool() { return tri != nullptr; }
};

}  // namespace FW
