#define _CRT_SECURE_NO_WARNINGS

#include "RayTracer.hpp"

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "rtIntersect.inl"

#include <algorithm>
#include <fstream>
#include <stdio.h>

// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer(void* buffer, size_t bufLen, unsigned int* pDigest);

namespace FW {

// Write a simple data type to a stream.
template<class T> std::ostream& write(std::ostream& stream, const T& x) {
    return stream.write(reinterpret_cast<const char*>(&x), sizeof(x));
}

// Read a simple data type from a stream.
template<class T> std::istream& read(std::istream& os, T& x) {
    return os.read(reinterpret_cast<char*>(&x), sizeof(x));
}

Vec2f getTexelCoords(Vec2f uv, const Vec2i size) {
    return Vec2f((uv.x - floor(uv.x)) * size.x, (uv.y - floor(uv.y)) * size.y);
}

/// form orthonormal basis matrix
Mat3f formBasis(const Vec3f& n) {
    Vec3f q = n;

    // replace the element with the smallest absolute value
    int minV = 0;
    if (abs(q.y) < abs(q[minV])) {
        minV = 1;
    }
    if (abs(q.z) < abs(q[minV])) {
        minV = 2;
    }
    q[minV] = 1.0f;

    Vec3f t = normalize(q.cross(n));
    Vec3f b = normalize(n.cross(t));

    Mat3f R;
    R.setCol(0, t);
    R.setCol(1, b);
    R.setCol(2, n);

    return R;
}

String RayTracer::computeMD5(const std::vector<Vec3f>& vertices) {
    unsigned char digest[16];
    MD5Buffer((void*)&vertices[0], sizeof(Vec3f) * vertices.size(), (unsigned int*)digest);

    // turn into string
    char ad[33];
    for (int i = 0; i < 16; ++i)
        ::sprintf(ad + i * 2, "%02x", digest[i]);
    ad[32] = 0;

    return FW::String(ad);
}

RayTracer::RayTracer() {}

RayTracer::~RayTracer() {}

void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles) {
    m_triangles = &triangles;

    std::ifstream infile(filename, std::ios::binary);

    // index list size
    __int32 listSize;
    read(infile, listSize);

    // index list
    std::vector<__int32> list;

    __int32 temp;
    for (auto i = 0; i < listSize; ++i) {
        read(infile, temp);
        list.push_back(temp);
    }

    // number of nodes
    __int32 bvhNodesSize;
    read(infile, bvhNodesSize);

    // nodes
    std::vector<flatNode> nodes;
    nodes.reserve(bvhNodesSize);

    __int32 startIdx, endIdx, rightChild, axis, depth;
    bool leaf;
    Vec3f min, max;
    for (auto i = 0; i < bvhNodesSize; ++i) {
        read(infile, min);
        read(infile, max);
        read(infile, startIdx);
        read(infile, endIdx);
        read(infile, rightChild);
        read(infile, leaf);
        read(infile, axis);
        nodes.push_back(flatNode(AABB(min, max), startIdx, endIdx, rightChild, leaf, axis));
    }
    read(infile, depth);

    // create BVH
    bvh = BVH(m_triangles, list, nodes, depth);

    // Print info
    std::printf("\nLoaded BVH:\n");
    std::printf("triangles: %d, nodes: %d, depth: %d\n\n", listSize, bvhNodesSize, depth);
    if (list.size() < 100) {
        int i = 0;
        std::printf("Nodes:\n");
        for (auto n : nodes) {
            std::printf(
                "%3d start: %2d, end: %2d, tris: %2d, right: %2d, leaf: %s\n",
                i,
                n.startIdx,
                n.endIdx,
                n.endIdx - n.startIdx + 1,
                n.rightChild,
                n.leaf ? "yes" : "no");
            i += 1;
        }
        std::printf("\n");
    }
}

void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
    // create output file
    std::ofstream outfile(filename, std::ios::binary);

    // index list size (vertices)
    write(outfile, (__int32)bvh.list.size());

    // index list
    for (int i = 0; i < bvh.list.size(); ++i) {
        write(outfile, (__int32)bvh.list[i]);
    }
    // number of nodes
    write(outfile, (__int32)bvh.nodes.size());

    // write each node
    for (auto i = 0; i < bvh.nodes.size(); ++i) {
        write(outfile, bvh.nodes[i].box.min);
        write(outfile, bvh.nodes[i].box.max);
        write(outfile, bvh.nodes[i].startIdx);
        write(outfile, bvh.nodes[i].endIdx);
        write(outfile, bvh.nodes[i].rightChild);
        write(outfile, bvh.nodes[i].leaf);
        write(outfile, bvh.nodes[i].axis);
    }
    // tree depth
    write(outfile, bvh.depth);
}

void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, bool use_SAH) {
    m_triangles = &triangles;

    // construct BVH
    bvh = BVH(m_triangles, use_SAH);
}

RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir, bool occlusion) const {
    // bool occlusion -> early exit when any hit will do (shadow rays and ambient occlusion)
    float tmin = 1.0f, umin = 0.0f, vmin = 0.0f;
    int imin = -1;
    bool hit = false;

    // precompute values for ray-box intersection
    Vec3f inv_dir = Vec3f(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
    int sign[3] = {inv_dir.x < 0.0f, inv_dir.y < 0.0f, inv_dir.z < 0.0f};

    // stack-based BVH traversal (adapted from the PBRT book):
    // currentNodeIndex holds the offset (index) of the node to be visited. It starts with a value of 0, representing the
    // root of the tree. The nodes that still need to be visited are stored in the nodesToVisit[] array, which acts as a
    // stack. toVisitOffset holds the offset (index) to the next free element in the stack.

    int toVisitOffset = 0, currentNodeIndex = 0;
    int nodesToVisit[64];

    ++m_rayCount;
    while (true) {
        flatNode node = bvh.nodes[currentNodeIndex];

        // Check ray against node bounding box
        if (node.intersect(orig, inv_dir, sign, tmin)) {
            // leaf node
            if (node.leaf) {
                // intersect with triangles
                for (auto i = node.startIdx; i <= node.endIdx; ++i) {
                    float t, u, v;
                    if ((*m_triangles)[bvh.list[i]].intersect_woop(orig, dir, t, u, v)) {
                        if (use_alpha) {
                            // test alpha texture
                            if ((*m_triangles)[bvh.list[i]].m_material->textures[MeshBase::TextureType_Alpha].exists()) {
                                // vertex texture coordinates
                                Vec2f tc1 = (*m_triangles)[bvh.list[i]].m_vertices[0].t;
                                Vec2f tc2 = (*m_triangles)[bvh.list[i]].m_vertices[1].t;
                                Vec2f tc3 = (*m_triangles)[bvh.list[i]].m_vertices[2].t;

                                // barycentric interpolation
                                Vec2f uv = (1 - u - v) * tc1 + u * tc2 + v * tc3;

                                Texture& alphaTex = (*m_triangles)[bvh.list[i]].m_material->textures[MeshBase::TextureType_Alpha];
                                const Image& img = *alphaTex.getImage();

                                Vec2i texelCoords = getTexelCoords(uv, img.getSize());
                                float alpha = img.getVec4f(texelCoords).y;  // green = opacity

                                if (alpha < 0.5f) {
                                    continue;
                                }
                            }
                        }
                        // update hit if closer
                        if (t > 0.0f && t < tmin) {
                            imin = bvh.list[i];
                            tmin = t;
                            umin = u;
                            vmin = v;
                            hit = true;
                        }
                        // early exit
                        if (occlusion && hit) {
                            // found a hit -> exit immediately!
                            return RaycastResult(&(*m_triangles)[imin], tmin, umin, vmin, orig + tmin * dir, orig, dir);
                        }
                    }
                }
                if (toVisitOffset == 0) {
                    // traversal complete
                    break;
                }
                // advance to next node
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            }
            // not a leaf node
            else {
                // For an interior node that the ray hits, it is necessary to visit both of its children.
                // it's desirable to visit the first child that the ray passes through before visiting the second one,
                // in case there is a primitive that the ray intersects in the first one, so that the ray's tMax value
                // can be updated, thus reducing the ray's extent and thus the number of node bounding boxes it intersects.
                //
                // An efficient way to perform a front-to-back traversal without incurring the expense of
                // intersecting the ray with both child nodes and comparing the distances is to use the
                // sign of the ray's direction vector for the coordinate axis along which primitives were
                // partitioned for the current node: if the sign is negative, we should visit the second child
                // before the first child, since the primitives that went into the second child's subtree were
                // on the upper side of the partition point.

                // Put far BVH node in the stack, advance to near node
                if (sign[node.axis]) {
                    // left node is far
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;  // left node is always next in the list
                    currentNodeIndex = node.rightChild;
                } else {
                    // right node is far
                    nodesToVisit[toVisitOffset++] = node.rightChild;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        }
        // didn't hit bounding box
        else {
            // stack is empty -> traversal complete
            if (toVisitOffset == 0) {
                break;
            }
            // next node to visit
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    // found a hit
    if (hit) {
        return RaycastResult(&(*m_triangles)[imin], tmin, umin, vmin, orig + tmin * dir, orig, dir);
    }

    return RaycastResult();
}

}  // namespace FW