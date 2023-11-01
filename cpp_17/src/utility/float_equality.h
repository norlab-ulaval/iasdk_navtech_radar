// ---------------------------------------------------------------------------------------------------------------------
// Copyright 2023 Navtech Radar Limited
// This file is part of IASDK which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//
// Disclaimer:
// Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
// any warranty of the item whatsoever, whether express, implied, or statutory,
// including, but not limited to, any warranty of merchantability or fitness
// for a particular purpose or any warranty that the contents of the item will
// be error-free.
// In no respect shall Navtech Radar incur any liability for any damages, including,
// but limited to, direct, indirect, special, or consequential damages arising
// out of, resulting from, or any way connected to the use of the item, whether
// or not based upon warranty, contract, tort, or otherwise; whether or not
// injury was sustained by persons or property or otherwise; and whether or not
// loss was sustained from, or arose out of, the results of, the item, or any
// services that may be provided by Navtech Radar.
// ---------------------------------------------------------------------------------------------------------------------

#ifndef FLOAT_EQUALITY_H
#define FLOAT_EQUALITY_H

#include <cstdlib>
#include <limits>

namespace Navtech::Utility {

    // The following definitions are taken from "The Art of Computer Programming" by Knuth

    // approximately_equal() gives whether the difference between a and b is smaller than the 
    // acceptable error (epsilon), determined by the larger of a or b. This means that the 
    // two values are "close enough", and we can say that they're approximately equal.
    // For example:
    //
    // approximately_equal(95.1, 100.0, 0.05) => true
    // 
    // That is, with epsilon being 5%, 95.1 is approximately 100, as it falls within 
    // the 5% margin of the largest value (100.0).
    // 
    // Approximately-equal is a 'weaker' check of equality than essentially-equal, for any
    // given value of epsilon.
    //
    inline bool approximately_equal(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return std::fabs(a - b) <= ( (std::fabs(a) < std::fabs(b) ? std::fabs(b) : std::fabs(a)) * epsilon);
    }


    // essentially_equal() gives whether the difference between a and b is smaller than the 
    // acceptable error (epsilon), determined by the smaller of a or b. 
    // This means that the values differ less than the acceptable difference in any calculation, 
    // so that perhaps they're not actually equal, but they're "essentially equal" (given the epsilon).
    // For example:
    // 
    // essentially_equal(95.1, 100.0, 0.05) => false
    //
    // That is, 95.1 is NOT essentially 100, as 100 is not within a 5% difference 
    // from 95.1 (smallest value).
    //
    // Essentially-equal is a 'stronger' check of equality than approxiamtely-equal, for any given
    // value of epsilon.
    //
    inline bool essentially_equal(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return std::fabs(a - b) <= ( (std::fabs(a) > std::fabs(b) ? std::fabs(b) : std::fabs(a)) * epsilon);
    }


    
    inline bool definitely_greater_than(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return (a - b) > ( (std::fabs(a) < std::fabs(b) ? std::fabs(b) : std::fabs(a)) * epsilon);
    }


    inline bool definitely_less_than(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return (b - a) > ( (std::fabs(a) < std::fabs(b) ? std::fabs(b) : std::fabs(a)) * epsilon);
    }

} // namespace Navtech::Utility


#endif // FLOAT_EQUALITY_H