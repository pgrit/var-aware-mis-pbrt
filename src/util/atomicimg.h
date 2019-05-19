#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_UTIL_ATOMICIMG_H
#define PBRT_UTIL_ATOMICIMG_H

#include <atomic>
#include <vector>

namespace pbrt {

// A datatype that allows copying std::atomic
// The copy procedure itself is not atomic
template <typename T>
struct CopyableAtomic : public std::atomic<T> {
    CopyableAtomic() { }
    CopyableAtomic(T v) { this->store(v); }
    CopyableAtomic& operator= (const CopyableAtomic& a) { this->store(a.load()); return *this; }
    CopyableAtomic(const CopyableAtomic& a) { this->store(a.load()); }
    CopyableAtomic& operator= (T i) { this->store(i); return *this; }
};

template <typename T>
static T AtomicAdd(std::atomic<T>& a, T b) {
    T old_val = a.load();
    T desired_val = old_val + b;
    while(!a.compare_exchange_weak(old_val, desired_val))
        desired_val = old_val + b;
    return desired_val;
}

using AtomicImage = std::vector<CopyableAtomic<float>>;


} // namespace pbrt

#endif // PBRT_UTIL_ATOMICIMG_H