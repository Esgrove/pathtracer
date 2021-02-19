#include "Bvh.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <ppl.h>

namespace FW {

// contructor when building BVH
BVH::BVH(std::vector<RTTriangle>* triangles, bool sah) : m_triangles(triangles), m_sahEnabled(sah) {
    root = std::make_unique<Node>();
    this->depth = 0;

    // initialize index list for triangles
    list.resize(triangles->size());
    for (auto i = 0; i < triangles->size(); ++i) {
        list[i] = i;
    }

    // recursively build tree
    if (m_sahEnabled) {
        std::cout << "\n  Building tree using SAH..." << std::endl;
    } else {
        std::cout << "\n  Building tree..." << std::endl;
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    construct(root, 0, static_cast<int>(list.size() - 1));
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "    Done!" << std::endl;
    std::printf("    Time = %.3f ms\n", (float)std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000);

    // flatten tree to a vector
    std::cout << "\n  Flattening tree..." << std::endl;

    begin = std::chrono::steady_clock::now();
    flatten(root);
    end = std::chrono::steady_clock::now();

    std::cout << "    Done!" << std::endl;
    std::printf("    Time = %.3f ms\n\n", (float)std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000);

    // Print info
    std::printf("triangles: %zd, nodes: %zd, depth: %d\n", list.size(), nodes.size(), depth);
    std::printf("bad splits: %d, total: %d, percent: %.1f%%\n", badSplits, totalSplits, (float)badSplits / (float)totalSplits * 100.0f);
    std::printf("flatnode size: %zd bytes\n", sizeof(flatNode));

    std::cout << std::endl;

    if (list.size() < 100) {
        // print nodes
        std::printf("\n");
        std::printf("Nodes:\n");
        int i = 0;
        for (auto n : nodes) {
            std::printf(
                "%3d start: %2d, end: %2d, tris: %2d, right child: %2d, leaf: %s\n",
                i,
                n.startIdx,
                n.endIdx,
                n.endIdx - n.startIdx + 1,
                n.rightChild,
                n.leaf ? "yes" : "no");
            i += 1;
        }
        std::cout << std::endl;
    }
};

void BVH::flatten(std::unique_ptr<Node>& node) {
    // Build a vector from the binary tree:
    // Depth-first in pre-order => left child node is always next in the vector

    // create a flatNode from Node
    flatNode flat_node = flatNode(node->box, node->startIdx, node->endIdx, node->leaf, node->axis);

    // index of this node
    __int32 index = (__int32)nodes.size();

    // add to list
    nodes.push_back(flat_node);

    // continue if not a leaf
    if (!flat_node.leaf) {
        nodes[index].leftChild = index + 1;

        // go left recursively
        flatten(node->leftChild);

        // right child index is current size after left recursion
        nodes[index].rightChild = (__int32)nodes.size();

        // go right recursively
        flatten(node->rightChild);
    }
}

void BVH::construct(std::unique_ptr<Node>& node, int startIdx, int endIdx) {
    // compute bounding box for this node
    node->box = computeBB(startIdx, endIdx);

    // store contained nodes (indices)
    node->startIdx = startIdx;
    node->endIdx = endIdx;

    // number of triangles in node
    int tris = endIdx - startIdx + 1;

    // continue recursion?
    if (tris > maxTrisInLeaf) {
        int dim = 0;
        int mid = (startIdx + endIdx) / 2;

        if (m_sahEnabled) {
            // Check all possible splits using dynamic programming

            // precalculate total area division
            float area = 1.0f / node->box.area();
            float minCost = FLT_MAX;

            std::vector<float> box_l_areas(tris - 1);
            std::vector<float> box_r_areas(tris - 1);

            for (auto d = 0; d <= 2; d++) {
                // sort index list with centroids along dimension d
                concurrency::parallel_sort(list.begin() + startIdx, list.begin() + endIdx, [this, d](int x, int y) -> bool {
                    return (*m_triangles)[x].centroid()[d] < (*m_triangles)[y].centroid()[d];
                });

                // calculate and store all split areas:
                // AABB for first and last triangle
                AABB box_l((*m_triangles)[list[startIdx]].min(), (*m_triangles)[list[startIdx]].max());
                box_l_areas[0] = box_l.area();

                AABB box_r((*m_triangles)[list[endIdx]].min(), (*m_triangles)[list[endIdx]].max());
                box_r_areas[0] = box_r.area();

                // increase existing bounding box with next triangle and calculate area
                for (auto i = 1; i < tris - 1; ++i) {
                    box_l = Union(box_l, AABB((*m_triangles)[list[startIdx + i]].min(), (*m_triangles)[list[startIdx + i]].max()));
                    box_l_areas[i] = box_l.area();

                    box_r = Union(box_r, AABB((*m_triangles)[list[endIdx - i]].min(), (*m_triangles)[list[endIdx - i]].max()));
                    box_r_areas[i] = box_r.area();
                }

                // calculate cost
                for (auto i = 0; i < tris - 1; ++i) {
                    auto n_l = i + 1;
                    auto n_r = tris - 1 - i;

                    float cost = 1.0f / maxTrisInLeaf + (n_l * box_l_areas[i] + n_r * box_r_areas[n_r - 1]) * area;

                    if (cost < minCost) {
                        minCost = cost;
                        dim = d;
                        mid = startIdx + i + 1;
                    }
                }
            }

            // Split at chosen point if cost is smaller than testing all triangles
            if (minCost < tris) {
                node->axis = dim;
                totalSplits += 1;
                depth += 1;

                // If we get a bad split, use object median
                if (mid == startIdx || mid == endIdx) {
                    badSplits += 1;
                    mid = (startIdx + endIdx) / 2;
                }

                // rearrange list
                std::nth_element(list.begin() + startIdx, list.begin() + mid, list.begin() + endIdx, [this, dim](int x, int y) -> bool {
                    return (*m_triangles)[x].centroid()[dim] < (*m_triangles)[y].centroid()[dim];
                });

                // construct left child node, recurse
                node->leftChild = std::make_unique<Node>();
                construct(node->leftChild, startIdx, mid - 1);

                // construct right child node, recurse
                node->rightChild = std::make_unique<Node>();
                construct(node->rightChild, mid, endIdx);

            } else {
                node->leaf = true;
                return;
            }
        } else {
            // try spatial median first, else use object median

            // Bounding box of centroids
            AABB centroidBounds((*m_triangles)[list[startIdx]].centroid(), (*m_triangles)[list[startIdx]].centroid());
            for (auto i = startIdx + 1; i <= endIdx; ++i) {
                centroidBounds = Union(centroidBounds, (*m_triangles)[list[i]].centroid());
            }
            // longest axis
            dim = centroidBounds.MaximumExtent();
            node->axis = dim;

            // Spatial median: split on the center of the longest axis
            float split_coord = 0.5f * (centroidBounds.min[dim] + centroidBounds.max[dim]);

            // partition
            auto it = std::partition(list.begin() + startIdx, list.begin() + endIdx, [this, dim, split_coord](int i) {
                return (*m_triangles)[i].centroid()[dim] < split_coord;
            });
            mid = static_cast<int>(it - list.begin());

            // if spatial median fails, use object median
            if (mid == startIdx || mid == endIdx) {
                badSplits += 1;
                mid = (startIdx + endIdx) / 2;
                std::nth_element(list.begin() + startIdx, list.begin() + mid, list.begin() + endIdx, [this, dim](auto x, auto y) -> bool {
                    return (*m_triangles)[x].centroid()[dim] < (*m_triangles)[y].centroid()[dim];
                });
            }
            totalSplits += 1;
            depth += 1;

            // construct left child node, recurse
            node->leftChild = std::make_unique<Node>();
            construct(node->leftChild, startIdx, mid - 1);

            // construct right child node, recurse
            node->rightChild = std::make_unique<Node>();
            construct(node->rightChild, mid, endIdx);
        }
    } else {
        node->leaf = true;
        return;
    }
}

AABB BVH::computeBB(int startIdx, int endIdx) {
    // compute axis-aligned bounding box for triangles in list

    // initialize bounds to min and max vertex of first triangle
    Vec3f p1 = (*m_triangles)[list[startIdx]].m_vertices[0].p;
    Vec3f p2 = (*m_triangles)[list[startIdx]].m_vertices[1].p;
    Vec3f p3 = (*m_triangles)[list[startIdx]].m_vertices[2].p;

    Vec3f minV = Vec3f(min(p1.x, p2.x, p3.x), min(p1.y, p2.y, p3.y), min(p1.z, p2.z, p3.z));
    Vec3f maxV = Vec3f(max(p1.x, p2.x, p3.x), max(p1.y, p2.y, p3.y), max(p1.z, p2.z, p3.z));

    // loop over all triangles in list between given indices
    Vec3f minT, maxT;
    for (auto i = startIdx + 1; i <= endIdx; ++i) {
        // update bounds
        p1 = (*m_triangles)[list[i]].m_vertices[0].p;
        p2 = (*m_triangles)[list[i]].m_vertices[1].p;
        p3 = (*m_triangles)[list[i]].m_vertices[2].p;

        minT = Vec3f(min(p1.x, p2.x, p3.x), min(p1.y, p2.y, p3.y), min(p1.z, p2.z, p3.z));
        maxT = Vec3f(max(p1.x, p2.x, p3.x), max(p1.y, p2.y, p3.y), max(p1.z, p2.z, p3.z));

        minV = Vec3f(min(minV.x, minT.x), min(minV.y, minT.y), min(minV.z, minT.z));
        maxV = Vec3f(max(maxV.x, maxT.x), max(maxV.y, maxT.y), max(maxV.z, maxT.z));
    }
    return AABB(minV, maxV);
}

// union of AABB & AABB
AABB BVH::Union(AABB b1, AABB b2) {
    return AABB(
        Vec3f(min(b1.min.x, b2.min.x), min(b1.min.y, b2.min.y), min(b1.min.z, b2.min.z)),
        Vec3f(max(b1.max.x, b2.max.x), max(b1.max.y, b2.max.y), max(b1.max.z, b2.max.z)));
}
// union of AABB & point
AABB BVH::Union(AABB b, Vec3f p) {
    return AABB(
        Vec3f(min(b.min.x, p.x), min(b.min.y, p.y), min(b.min.z, p.z)),
        Vec3f(max(b.max.x, p.x), max(b.max.y, p.y), max(b.max.z, p.z)));
}

}  // namespace FW