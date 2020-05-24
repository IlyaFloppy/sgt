#pragma once

#include <cmath>

namespace helper {
    const double WGS84_BIG_RAD = 6378137;
    const double WGS84_SMALL_RAD = 6356752.3142;

    const double WGS84_BIG_LENGTH =  WGS84_BIG_RAD * 2 * M_PI;
    const double WGS84_SMALL_LENGTH =  WGS84_SMALL_RAD * 2 * M_PI;
}