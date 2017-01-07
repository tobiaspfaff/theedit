#ifndef UTIL_HPP
#define UTIL_HPP

#include <string>
#include "vectors.hpp"

// common string ops
std::string stringf (std::string format, ...);
bool ends_with(const std::string& text, const std::string& ending);
std::string replace(std::string text, const std::string& from, const std::string& to);


// Math utilities
extern const double infinity;
template<typename T> inline T square(T v) { return v*v; }
inline bool is_finite(double v) { return v < infinity && v > -infinity; }
inline bool is_nan(double v) { return v != v; }
inline bool is_nan(const Vec2& v) { return is_nan(v[0]) || is_nan(v[1]); }
inline bool is_nan(const Vec3& v) { return is_nan(v[0]) || is_nan(v[1]) || is_nan(v[2]); }
inline bool is_nan(const Vec4& v) { return is_nan(v[0]) || is_nan(v[1]) || is_nan(v[2]) || is_nan(v[3]); }

template <typename T> T clamp (const T &x, const T &a, const T &b) { return std::min(std::max(x, a), b); }

// Debugging
extern void segfault();
void ERROR(const std::string& msg);

#endif
