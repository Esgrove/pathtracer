#pragma once

#include "util.hpp"

/*
 * Utilities for value transport to hierarchy files as binary data
 */

#include <iostream>

template<class T> std::ostream& filesave(std::ostream& os, const T& x) {
    return os.write(reinterpret_cast<const char*>(&x), sizeof(x));
}

template<class T> std::istream& fileload(std::istream& os, T& x) {
    return os.read(reinterpret_cast<char*>(&x), sizeof(x));
}

class Saver : noncopyable {
public:
    Saver(std::ostream& os, Statusbar& sbar) : os(os), sbar(sbar) {}
    template<class T> Saver& operator()(const T& x) {
        os.write(reinterpret_cast<const char*>(&x), sizeof(x));
        return *this;
    }
    void statusbar(size_t counter) { sbar.update(counter); }

private:
    std::ostream& os;
    Statusbar&    sbar;
};

class Loader : noncopyable {
public:
    Loader(std::istream& is, Statusbar& sbar) : is(is), sbar(sbar) {}
    template<class T> Loader& operator()(T& x) {
        is.read(reinterpret_cast<char*>(&x), sizeof(x));
        return *this;
    }
    void statusbar(size_t counter) { sbar.update(counter); }

private:
    std::istream& is;
    Statusbar&    sbar;
};
