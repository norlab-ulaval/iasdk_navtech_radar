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

#ifndef VECTOR_MATHS_H
#define VECTOR_MATHS_H

#include <cstring>
#include <vector>

namespace Navtech {

    namespace Vector_maths {

        template<typename T>
        inline void scalar_sum(std::vector<T>& data, std::size_t length, T& result)
        {
            for (auto index = 0u; index < length; index++)
                result += data[index];
        }

        template<typename T>
        inline void scalar_sum(T* data, std::size_t length, T& result)
        {
            for (auto index = 0u; index < length; index++)
                result += data[index];
        }

        template<typename T>
        inline void scalar_square(std::vector<T>& data, std::size_t length, T& result)
        {
            for (auto index = 0u; index < length; index++)
                result += data[index] * data[index];
        }

        template<typename T>
        inline void scalar_square(T* data, std::size_t length, T& result)
        {
            for (auto index = 0u; index < length; index++)
                result += data[index] * data[index];
        }

        template<typename T>
        inline void vector_square(std::vector<T>& data, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data[index] * data[index];
        }

        template<typename T>
        inline void vector_square(T* data, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data[index] * data[index];
        }

        template<typename T>
        inline void vector_cube(T* data, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data[index] * data[index] * data[index];
        }

        template<typename T>
        inline void vector_cube(std::vector<T>& data, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data[index] * data[index] * data[index];
        }

        template<typename T>
        inline void vector_multiply(T* data1, T* data2, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data1[index] * data2[index];
        }

        template<typename T>
        inline void vector_multiply(std::vector<T>& data1, std::vector<T>& data2, std::size_t length, T* result)
        {
            for (auto index = 0u; index < length; index++)
                result[index] += data1[index] * data2[index];
        }
    } // namespace Vector_maths

} // namespace Navtech

#endif // VECTOR_MATHS_H
