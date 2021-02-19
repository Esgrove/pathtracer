#include "util.hpp"

#include <iostream>

Statusbar::Statusbar(const std::string& descr, size_t max, float dispInterval)
    : max(max)
    , imax(1.0f / max)
    , interval(dispInterval)
    , last(0) {
    std::cout << descr << " ... (" << max << ")" << std::endl;
}

void Statusbar::update(size_t val) {
    if ((val - last) * imax >= interval || val == max) {
        std::cout << (100.0f * val * imax) << " %\t\t\t\r" << std::flush;
        if (val == max)
            std::cout << std::endl;
        last = val;
    }
}

void FW::mapToDisk(const float a, const float b, float& x, float& y) {
    // Shirley & Chiu: "A Low Distortion Map Between Disk and Square"
    float phi, r;
    if (a > -b) {     // region 1 or 2
        if (a > b) {  // region 1, also |a| > |b|
            r = a;
            phi = (FW_PI / 4.0f) * (b / a);
        } else {  // region 2, also |b| > |a|
            r = b;
            phi = (FW_PI / 4.0f) * (2 - (a / b));
        }
    } else {          // region 3 or 4
        if (a < b) {  // region 3, also |a| >= |b|, a != 0
            r = -a;
            phi = (FW_PI / 4.0f) * (4.0f + (b / a));
        } else {  // region 4, |b| >= |a|, but a==0 and b==0 could occur.
            r = -b;
            if (b != 0)
                phi = (FW_PI / 4.0f) * (6.0f - (a / b));
            else
                phi = 0;
        }
    }
    x = r * cos(phi);
    y = r * sin(phi);
}