#pragma once

#include "BvhNode.hpp"
#include "RTTriangle.hpp"
#include "base/Math.hpp"

#include <vector>

// https://en.wikipedia.org/wiki/Bounding_volume_hierarchy

namespace FW {

class BVH {
public:
    // constructor
    BVH() {};
    BVH(std::vector<RTTriangle>* triangles, bool sah);
    BVH(std::vector<RTTriangle>* triangles, std::vector<__int32> list, std::vector<flatNode> nodes, int depth)
        : m_triangles(triangles)
        , list(list)
        , nodes(nodes)
        , depth(depth) {};

    // builds tree recursively
    void construct(std::unique_ptr<Node>& node, int startIdx, int endIdx);

    // flattens tree to vector
    void flatten(std::unique_ptr<Node>& node);

    // finds bounding box limits
    FW::AABB computeBB(int startIdx, int endIdx);

    AABB Union(AABB a, AABB b);
    AABB Union(AABB a, Vec3f b);

    // root node
    std::unique_ptr<Node> root;

    // index list to triangles
    std::vector<__int32> list;

    // list of nodes
    std::vector<flatNode> nodes;

    // tree depth
    __int32 depth = 0;

private:
    bool m_sahEnabled;

    __int32 badSplits = 0;
    __int32 totalSplits = 0;
    __int32 maxTrisInLeaf = 3;

    // actual triangle data
    std::vector<RTTriangle>* m_triangles;
};
}  // namespace FW
