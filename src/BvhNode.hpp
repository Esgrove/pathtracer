#pragma once

#include "RTTriangle.hpp"
#include "base/Math.hpp"

#include <iostream>
#include <memory>
#include <vector>

namespace FW {

struct Plane : public Vec4f {
    inline float dot(const Vec3f& p) const { return p.x * x + p.y * y + p.z * z + w; }
};

struct AABB {
    Vec3f min, max;
    inline AABB() : min(), max() {}
    inline AABB(const Vec3f& min, const Vec3f& max) : min(min), max(max) {}
    inline F32 area() const {
        Vec3f d(max - min);
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }
    inline int MaximumExtent() const {
        Vec3f d(max - min);
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }
};

inline std::ostream& operator<<(std::ostream& os, const FW::Vec3f& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

inline std::ostream& operator<<(std::ostream& os, const FW::Vec4f& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
}

inline std::ostream& operator<<(std::ostream& os, const AABB& bb) {
    return os << "BB(" << bb.min << ", " << bb.max << ")";
}

// Treenode using pointers
struct Node {
    // Axis-aligned bounding box
    FW::AABB box;

    // Indices in to the global triangle list
    unsigned int start_idx, end_idx;

    // leaf node?
    bool leaf = false;

    // split axis
    unsigned int axis = 0;

    // child nodes
    std::unique_ptr<Node> leftChild;
    std::unique_ptr<Node> rightChild;
};

// Treenode for depth-first ordered list
struct flatNode {
    // constructor
    flatNode(FW::AABB box, unsigned __int32 start, unsigned __int32 end, bool leaf, unsigned __int32 axis)
        : box(box)
        , start_idx(start)
        , end_idx(end)
        , leaf(leaf)
        , axis(axis)
        , triangles(end - start + 1) {};

    flatNode(FW::AABB box, unsigned __int32 start, unsigned __int32 end, unsigned __int32 right, bool leaf, unsigned __int32 axis)
        : box(box)
        , start_idx(start)
        , end_idx(end)
        , rightChild(right)
        , leaf(leaf)
        , axis(axis)
        , triangles(end - start + 1) {};

    // Axis-aligned bounding box (size = 24B)
    AABB box;

    // Indices into the global triangle list (size = 2*4B = 8B)
    unsigned __int32 start_idx, end_idx;

    // leaf node (size = 1B -> 4B)
    bool leaf = false;

    // index offset for right child node (size = 4B)
    unsigned __int32 rightChild = 0;

    // split axis (size = 4B)
    unsigned __int32 axis;

    // total size = 44B
    // can't get down to 32, have to increase to 64 for good cache line behavior

    // padding
    unsigned __int32 leftChild;  // 48B
    unsigned __int32 triangles;  // 52B

    unsigned __int32 pad0 = 0;  // 56B
    unsigned __int32 pad1 = 0;  // 60B
    unsigned __int32 pad2 = 0;  // 64B

    // intersect with bounding box
    bool intersect(const FW::Vec3f& orig, const FW::Vec3f& inv_dir, const int sign[3], const float& t1) const;
};
}  // namespace FW