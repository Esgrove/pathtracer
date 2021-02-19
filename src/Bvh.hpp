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
    BVH(std::vector<RTTriangle>* triangles, std::vector<unsigned int> list, std::vector<flatNode> nodes, unsigned int depth)
        : triangles(triangles)
        , list(list)
        , nodes(nodes)
        , depth(depth) {};

    // builds tree recursively
    void construct(std::unique_ptr<Node>& node, unsigned int start_idx, unsigned int end_idx);

    // flattens tree to vector
    void flatten(std::unique_ptr<Node>& node);

    // finds bounding box limits
    FW::AABB computeBB(unsigned int start_idx, unsigned int end_idx);

    AABB Union(AABB a, AABB b);
    AABB Union(AABB a, Vec3f b);

    // root node
    std::unique_ptr<Node> root;

    // index list to triangles
    std::vector<unsigned __int32> list;

    // list of nodes
    std::vector<flatNode> nodes;

    // tree depth
    unsigned __int32 depth = 0;

private:
    bool sah_enabled;

    unsigned __int32 bad_splits = 0;
    unsigned __int32 total_splits = 0;

    // max triangles in leaf node
    unsigned __int32 max_tris = 3;

    // actual triangle data
    std::vector<RTTriangle>* triangles;
};
}  // namespace FW
