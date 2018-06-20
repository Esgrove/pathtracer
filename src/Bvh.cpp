#include "Bvh.hpp"
#include <algorithm>
#include <iostream>
#include <chrono>
#include <ppl.h>

namespace FW
{

// contructor when building BVH
BVH::BVH(std::vector<RTTriangle>* triangles, bool sah) : triangles(triangles), sah_enabled(sah) 
{
	std::unique_ptr<Node> root = std::make_unique<Node>();

	this->depth = 0;

	// initialize index list for triangles
	list.resize(triangles->size());
	for (unsigned int i = 0; i < triangles->size(); i++) {
		list[i] = i;
	}

	// recursively build tree
	if (sah_enabled) { std::cout << "\n  Building tree using SAH..." << std::endl; }
	else			 { std::cout << "\n  Building tree..." << std::endl; }

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	construct(root, 0, (unsigned int)list.size() - 1);
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
	std::printf("triangles: %u, nodes: %u, depth: %u\n", list.size(), nodes.size(), depth);

	std::printf("bad splits: %u, total: %u, percent: %.1f%%\n", bad_splits, total_splits, (float)bad_splits / (float)total_splits*100.0f);

	std::printf("flatnode size: %u bytes\n", sizeof(flatNode));
	
	std::cout << std::endl;

	if (list.size() < 100) {
		// print nodes
		std::printf("\n");
		std::printf("Nodes:\n");
		unsigned int i = 0;
		for (auto n : nodes) {
			std::printf("%3d start: %2d, end: %2d, tris: %2d, right child: %2d, leaf: %s\n", i, n.start_idx, n.end_idx, n.end_idx - n.start_idx + 1, n.rightChild, n.leaf ? "yes" : "no");
			i += 1;
		}
		std::cout << std::endl;
	}
};

void BVH::flatten(std::unique_ptr<Node> &node) 
{
	// Build a vector from the binary tree:
	// Depth-first in pre-order => left child node is always next in the vector
	
	// create a flatNode from Node
	flatNode flat_node = flatNode(node->box, node->start_idx, node->end_idx, node->leaf, node->axis);

	// index of this node
	unsigned __int32 index = (unsigned __int32)nodes.size();

	// add to list
	nodes.push_back(flat_node);

	// continue if not a leaf
	if (!flat_node.leaf) {

		nodes[index].leftChild = index + 1;

		// go left recursively
		flatten(node->leftChild);

		// right child index is current size after left recursion
		nodes[index].rightChild = (unsigned __int32)nodes.size();

		// go right recursively
		flatten(node->rightChild);
	}
}

void BVH::construct(std::unique_ptr<Node> &node, unsigned int start_idx, unsigned int end_idx)
{
	// compute bounding box for this node
	node->box = computeBB(start_idx, end_idx);

	// store contained nodes (indices)
	node->start_idx = start_idx;
	node->end_idx   = end_idx;

	// number of triangles in node
	unsigned int tris = end_idx - start_idx + 1;

	// continue recursion?
	if (tris > max_tris) {

		unsigned int dim = 0;
		unsigned int mid = (start_idx + end_idx) / 2;
		
		if (sah_enabled) {

			// Check all possible splits using dynamic programming

			float area = 1.0f / node->box.area(); // precalculate total area division
			float minCost = FLT_MAX;

			std::vector<float> box_l_areas(tris-1);
			std::vector<float> box_r_areas(tris-1);

			for (unsigned int d = 0; d <= 2; d++) {

				// sort index list with centroids along dimension d
				concurrency::parallel_sort(list.begin() + start_idx, list.begin() + end_idx, [this, d](unsigned int x, unsigned int y) -> bool {
					return (*triangles)[x].centroid()[d] < (*triangles)[y].centroid()[d]; });

				// calculate and store all split areas:
				// AABB for first and last triangle
				AABB box_l((*triangles)[list[start_idx]].min(), (*triangles)[list[start_idx]].max());
				box_l_areas[0] = box_l.area();

				AABB box_r((*triangles)[list[end_idx]].min(), (*triangles)[list[end_idx]].max());
				box_r_areas[0] = box_r.area();

				// increase existing bounding box with next triangle and calculate area
				for (unsigned int i = 1; i < tris-1; i++) {
					box_l = Union(box_l, AABB((*triangles)[list[start_idx + i]].min(), (*triangles)[list[start_idx + i]].max()));
					box_l_areas[i] = box_l.area();

					box_r = Union(box_r, AABB((*triangles)[list[end_idx-i]].min(), (*triangles)[list[end_idx-i]].max()));
					box_r_areas[i] = box_r.area();
				}

				// calculate cost
				for (unsigned int i = 0; i < tris-1; i++) {
					unsigned int n_l = i + 1;
					unsigned int n_r = tris - 1 - i;
					
					float cost = 1.0f / max_tris + (n_l * box_l_areas[i] + n_r * box_r_areas[n_r - 1]) * area;
			
					if (cost < minCost) {
						minCost = cost;
						dim     = d;
						mid     = start_idx + i + 1;
					}
				}
			}

			// Split at chosen point if cost is smaller than testing all triangles
			if (minCost < tris) {

				node->axis = dim; 
				total_splits += 1;
				depth += 1;

				// If we get a bad split, use object median
				if (mid == start_idx || mid == end_idx) {
					bad_splits += 1;
					mid = (start_idx + end_idx) / 2;
				}
				
				// rearrange list 
				std::nth_element(list.begin() + start_idx, list.begin() + mid, list.begin() + end_idx,
					[this, dim](unsigned int x, unsigned int y) -> bool {
					return (*triangles)[x].centroid()[dim] < (*triangles)[y].centroid()[dim]; });

				// construct left child node, recurse
				node->leftChild = std::make_unique<Node>();
				construct(node->leftChild, start_idx, mid - 1);

				// construct right child node, recurse
				node->rightChild = std::make_unique<Node>();
				construct(node->rightChild, mid, end_idx);

			} 
			else {
				node->leaf = true;
				return;
			}
		}
		else { 
			// try spatial median first, else use object median

			// Bounding box of centroids
			AABB centroidBounds((*triangles)[list[start_idx]].centroid(), (*triangles)[list[start_idx]].centroid());
			for (unsigned int i = start_idx + 1; i <= end_idx; ++i) {
				centroidBounds = Union(centroidBounds, (*triangles)[list[i]].centroid());
			}
			// longest axis
			dim = centroidBounds.MaximumExtent();
			node->axis = dim; 
			
			// Spatial median: split on the center of the longest axis
			float split_coord = 0.5f * (centroidBounds.min[dim] + centroidBounds.max[dim]);

			// partition
			auto it = std::partition(list.begin() + start_idx, list.begin() + end_idx, [this, dim, split_coord](unsigned int i) {
						return (*triangles)[i].centroid()[dim] < split_coord;});
			mid = it - list.begin();

			// if spatial median fails, use object median
			if (mid == start_idx || mid == end_idx) {
				bad_splits += 1;
				mid = (start_idx + end_idx) / 2;
				std::nth_element(list.begin() + start_idx, list.begin() + mid, list.begin() + end_idx,
					[this, dim](auto x, auto y) -> bool {
					return (*triangles)[x].centroid()[dim] < (*triangles)[y].centroid()[dim];});
			}
			total_splits += 1;
			depth += 1;

			// construct left child node, recurse
			node->leftChild = std::make_unique<Node>();
			construct(node->leftChild, start_idx, mid-1);

			// construct right child node, recurse
			node->rightChild = std::make_unique<Node>();
			construct(node->rightChild, mid, end_idx);
		}
	}
	else { 
		node->leaf = true;
		return;
	}
}

AABB BVH::computeBB(unsigned int start_idx, unsigned int end_idx)
{
	// compute axis-aligned bounding box for triangles in list

	// initialize bounds to min and max vertex of first triangle
	Vec3f p1 = (*triangles)[list[start_idx]].m_vertices[0].p;
	Vec3f p2 = (*triangles)[list[start_idx]].m_vertices[1].p;
	Vec3f p3 = (*triangles)[list[start_idx]].m_vertices[2].p;

	Vec3f minV = Vec3f(min(p1.x, p2.x, p3.x), min(p1.y, p2.y, p3.y), min(p1.z, p2.z, p3.z));
	Vec3f maxV = Vec3f(max(p1.x, p2.x, p3.x), max(p1.y, p2.y, p3.y), max(p1.z, p2.z, p3.z));

	// loop over all triangles in list between given indices
	Vec3f minT, maxT;
	for (unsigned int i = start_idx + 1; i <= end_idx; i++) {
		// update bounds
		p1 = (*triangles)[list[i]].m_vertices[0].p;
		p2 = (*triangles)[list[i]].m_vertices[1].p;
		p3 = (*triangles)[list[i]].m_vertices[2].p;

		minT = Vec3f(min(p1.x, p2.x, p3.x), min(p1.y, p2.y, p3.y), min(p1.z, p2.z, p3.z));
		maxT = Vec3f(max(p1.x, p2.x, p3.x), max(p1.y, p2.y, p3.y), max(p1.z, p2.z, p3.z));

		minV = Vec3f(min(minV.x, minT.x), min(minV.y, minT.y), min(minV.z, minT.z));
		maxV = Vec3f(max(maxV.x, maxT.x), max(maxV.y, maxT.y), max(maxV.z, maxT.z));
	}
	return AABB(minV, maxV);
}

// union of AABB & AABB
AABB BVH::Union(AABB b1, AABB b2) 
{
	return AABB(Vec3f(min(b1.min.x, b2.min.x), min(b1.min.y, b2.min.y), min(b1.min.z, b2.min.z)),
				Vec3f(max(b1.max.x, b2.max.x), max(b1.max.y, b2.max.y), max(b1.max.z, b2.max.z)));
}
// union of AABB & point
AABB BVH::Union(AABB b, Vec3f p) 
{
	return AABB(Vec3f(min(b.min.x, p.x), min(b.min.y, p.y), min(b.min.z, p.z)),
				Vec3f(max(b.max.x, p.x), max(b.max.y, p.y), max(b.max.z, p.z)));
}

} // namespace FW