#pragma once

#include <cmath>

namespace cyclone {
    /*defines precision of sqrt operator*/
    #define real_sqrt sqrtf
    #define real_pow powf
    #define REAL_MAX __DBL_MAX__

    // defines precision of abs magnitude operator.
    #define real_abs fabsf


    #define real_sin sinf

    #define real_cos cosf
    #define real_exp expf
    #define real_pi 3.14159
    //defines precision for real numbers. float is used originally

    typedef float real;
    
}