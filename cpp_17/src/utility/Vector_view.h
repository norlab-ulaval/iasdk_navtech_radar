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

#ifndef VECTOR_VIEW_H
#define VECTOR_VIEW_H

namespace Navtech::Utility {

    // Vector_view presents a lightweight, immutable view on a pre-existing vector.
    // It acts as a (feature-poor) vector equivalent of  
    //
    template <typename Vec_Ty>
    class Vector_view {
    public:
        // NOTE - Iterator and Const_iterator are the
        // same, as a view is always immutable - that is,
        // the referenced vector cannot be modified via
        // the view.
        //
        using const_iterator  = typename Vec_Ty::const_iterator;
        using iterator        = const_iterator;
        using size_type       = typename Vec_Ty::size_type;
        using value_type      = typename Vec_Ty::value_type;
        using const_reference = const value_type&;
        using reference       = const_reference;

        Vector_view(const Vec_Ty& v) :
            subject { &v }
        {
        }

        size_type size() const
        {
            return subject->size();
        }

        iterator begin() const
        {
            return subject->begin();
        }

        iterator end() const
        {
            return subject->end();
        } 

        const_iterator cbegin() const
        {
            return subject->begin();
        }

        const_iterator cend() const
        {
            return subject->end();
        }

        reference operator[](int idx)
        {
            return subject->operator[](idx);
        }


        const_reference operator[](int idx) const
        {
            return subject->operator[](idx);
        }

        operator const Vec_Ty&() const
        {
            return *subject;
        }

        const value_type* data() const
        {
            return subject->data();
        }
        
    private:
        const Vec_Ty* subject;
    };


    template <typename T>
    Vector_view(T) -> Vector_view<T>;

} // namespace Navtech::Utility

#endif // VECTOR_VIEW_H