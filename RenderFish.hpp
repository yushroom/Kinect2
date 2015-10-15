#ifndef RENDERFISH_H
#define RENDERFISH_H

#define RENDERFISH_VERSION "0.0.1"

#if defined(_WIN32) || defined(_WIN64)
#define RENDERFISH_PLATFORM_IS_WINDOWS
#elif defined(__linux__)
#define RENDERFISH_PLATFORM_IS_LINUX
#elif defined(__APPLE__)
#define RENDERFISH_PLATFORM_IS_APPLE
#endif

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstdint>

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <iostream>

#include "Error.hpp"

#endif // RENDERFISH_H