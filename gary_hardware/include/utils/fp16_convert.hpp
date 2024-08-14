#pragma once

namespace utils {

    typedef unsigned short ushort;
    typedef unsigned int uint;

    float half_to_float(ushort x);
    ushort float_to_half(float x);
}