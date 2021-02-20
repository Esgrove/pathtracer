#pragma once

#include "Bvh.hpp"
#include "RTTriangle.hpp"
#include "RaycastResult.hpp"
#include "base/String.hpp"

#include <atomic>
#include <vector>

namespace FW {

// Given a vector n, forms an orthogonal matrix with n as the last column, i.e.,
// a coordinate system aligned such that n is its local z axis.
Mat3f formBasis(const Vec3f& n);

Vec2f getTexelCoords(Vec2f uv, const Vec2i size);

// Main class for tracing rays using BVHs.
class RayTracer {
public:
    RayTracer(void);
    ~RayTracer(void);

    // This function computes an MD5 checksum of the input scene data,
    // WITH the assumption that all vertices are allocated in one big chunk.
    static FW::String computeMD5(const std::vector<Vec3f>& vertices);

    RaycastResult raycast(const Vec3f& orig, const Vec3f& dir, bool occlusion) const;
    void constructHierarchy(std::vector<RTTriangle>& triangles, bool m_sahEnabled);
    void loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles);
    void saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles);

    int getRayCount() { return m_rayCount; }
    void resetRayCounter() { m_rayCount = 0; }

    std::vector<RTTriangle>* m_triangles;
    BVH bvh;
    bool useAlpha = true;

private:
    mutable std::atomic<int> m_rayCount = 0;
};

}  // namespace FW