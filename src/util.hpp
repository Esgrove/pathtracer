#pragma once

#include "base/Math.hpp"

#include <string>

class noncopyable {
protected:
    noncopyable() {}

private:
    noncopyable(const noncopyable&);
    const noncopyable& operator=(const noncopyable&);
};

// print percentage status to terminal sometimes when updating
class Statusbar {
public:
    Statusbar(const std::string& descr, size_t max, float dispInterval = 0.01f);
    void update(size_t val);

private:
    size_t max, last;
    float  imax;
    float  interval;
};

namespace FW {

void mapToDisk(const float a, const float b, float& x, float& y);

// return a reference to a coordinate that matches some criterion, e.g. min or min of abs value
// i guess this would go easily also with .getPtr()
template<class Comp> inline F32& filtcoord(Vec3f& v, Comp comp) {
    if (comp(v.x, v.y)) {
        if (comp(v.x, v.z)) {
            return v.x;
        } else {
            return v.z;
        }
    } else {
        if (comp(v.y, v.z)) {
            return v.y;
        } else {
            return v.z;
        }
    }
}

inline F32& mincoord(Vec3f& v) {
    return filtcoord(v, [](float a, float b) { return a < b; });
}

inline F32& maxcoord(Vec3f& v) {
    return filtcoord(v, [](float a, float b) { return a >= b; });
}
}  // namespace FW
