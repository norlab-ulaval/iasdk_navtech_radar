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
v
#ifndef HEAP_ARRAY_H
#define HEAP_ARRAY_H

#include <array>
#include <cstddef>

namespace Navtech::Utility {

    // ----------------------------------------------------------------------------
    // Heap_array is a fixed-sized array allocated on the heap.
    // It supports the basic features of std::array, but does
    // not support intialization lists.
    // 
    template <typename T, std::size_t sz>
    class Heap_array {
    public:
        using Iterator = typename std::array<T, sz>::iterator;

        Iterator begin()
        {
            return arr->begin();
        }

        Iterator end()
        {
            return arr->end();
        }

        std::size_t size() const
        {
            return sz;
        }

        T& operator[](int index)
        {
            return (*arr)[index];
        }

        const T& operator[](int index) const
        {
            return (*arr)[index];
        }

        T* data()
        {
            return arr->data();
        }

    private:
        std::unique_ptr<std::array<T, sz>> arr { new std::array<T, sz> { } };
    };


} // namespace Navtech::Utility

#endif