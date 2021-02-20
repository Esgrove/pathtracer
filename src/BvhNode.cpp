#include "BvhNode.hpp"

#include <float.h>
#include <limits>
#include <math.h>

namespace FW {

bool FlatNode::intersect(const FW::Vec3f& orig, const FW::Vec3f& invDir, const int sign[3], const float& t1) const {
    /*
    Williams, Amy, et al.
    "An Efficient and Robust Ray-Box Intersection Algorithm."
    Journal of Graphics, GPU, and Game Tools 10.1 (2005): 49-54.
    */

    Vec3f bounds[2] = {box.min, box.max};

    // x
    float tmin = (bounds[sign[0]].x - orig.x) * invDir.x;
    float tmax = (bounds[1 - sign[0]].x - orig.x) * invDir.x;

    // y
    float tymin = (bounds[sign[1]].y - orig.y) * invDir.y;
    float tymax = (bounds[1 - sign[1]].y - orig.y) * invDir.y;

    // no intersection
    if ((tmin > tymax) || (tymin > tmax)) {
        return false;
    }

    // update min and max for y
    if (tymin > tmin) {
        tmin = tymin;
    }
    if (tymax < tmax) {
        tmax = tymax;
    }

    // z
    float tzmin = (bounds[sign[2]].z - orig.z) * invDir.z;
    float tzmax = (bounds[1 - sign[2]].z - orig.z) * invDir.z;

    // no intersection
    if ((tmin > tzmax) || (tzmin > tmax)) {
        return false;
    }

    // update min and max for z
    if (tzmin > tmin) {
        tmin = tzmin;
    }
    if (tzmax < tmax) {
        tmax = tzmax;
    }

    return ((tmin < t1) && (tmax > 0.0f));
}
}  // namespace FW