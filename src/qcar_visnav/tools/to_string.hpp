#ifndef TO_STRING_HPP
#define TO_STRING_HPP

#include <sstream>
#include <string>

// Minimal, C++14-friendly stream-to-string helper
template<typename T>
inline std::string to_string(const T& obj) {
    std::ostringstream oss; oss << obj; return oss.str();
}

#endif
