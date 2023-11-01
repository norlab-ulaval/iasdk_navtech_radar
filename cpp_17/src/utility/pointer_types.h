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

#ifndef POINTER_TYPES_H
#define POINTER_TYPES_H

#include <memory>

namespace Navtech {

    // ------------------------------------------------------
    // These type aliases provide a more specific declaration
    // for the use of the pointer; particularly with regards
    // to lifetime management of objects.
    //

    // Declares that the lifetime of the object is managed
    // by the class declaring ownership.  Ownership of the
    // object may be transferred (via move()) but cannot be
    // shared.
    // Owner_of should only be used in one of two situations:
    // - The object may be lazy-instantiated -that is, its
    //   lifetime may be significantly shorter than that of
    //   its parent.
    // - The object's may be replaced at run-time; for example
    //   with a sub-type.
    //
    template <typename T>
    using owner_of = std::unique_ptr<T>;

    // Dynamically allocate a T object and construct it
    // with the supplied arguments.
    //
    template <typename T, typename... Arg_Ty>
    inline owner_of<T> allocate_owned(Arg_Ty&&... args)
    {
        return owner_of<T> { new T { std::forward<Arg_Ty>(args)... } };
    }


    // --------------------------------------------------------------------------------------------
    // Declares that the lifetime of the object is shared
    // by several objects. The last owner to go out of scope
    // destroys the shared object.
    //
    template <typename T>
    using shared_owner = std::shared_ptr<T>;

    // Dynamically allocate a T object and construct it
    // with the supplied arguments.
    //
    template <typename T, typename... Arg_Ty>
    inline auto allocate_shared(Arg_Ty&&... args)
    {
        return std::make_shared<T>(std::forward<Arg_Ty>(args)...);
    }

    // #define allocate_shared std::make_shared


    // --------------------------------------------------------------------------------------------
    // Declares that the class does NOT manage the lifetime
    // of the object, it simply _uses_ (that is, calls the
    // methods of) the object.
    //
    template <typename T>
    using association_to = T*;

    // Create an association (that is, a pointer-to)
    // the supplied argument.  Prefer this function
    // to explicitly taking the address of the object
    //
    template <typename T>
    inline auto associate_with(T& from)
    {
        return &from;
    }

    // Create an association, given a supplied
    // pointer/association.  This function
    // does no transformation of the argument, but
    // should be used to maintain the semantics of
    // forming associations
    //
    template <typename T>
    inline auto associate_with(T* from)
    {
        return from;
    }

    // Create an association to a shared object.
    // Note - the returned association does NOT participate
    // in the lifetime management of the argument.  You
    // should NEVER delete the association returned from this
    // function.
    //
    template <typename T>
    inline auto associate_with(shared_owner<T>& from)
    {
        return from.get();
    }

    // Create an association to an owned object.
    // Note - the returned association does NOT participate
    // in the lifetime management of the argument.  You
    // should NEVER delete the association returned from this
    // function.
    //
    template <typename T>
    inline auto associate_with(owner_of<T>& from)
    {
        return from.get();
    }

    // Create an association to a sub-type of the supplied argument.
    // This function performs a dynamic downcast on the supplied
    // pointer/association type.  If the downcast fails, the returned
    // association will be set to nullptr.
    //
    template <typename To_Ty, typename From_Ty>
    association_to<To_Ty> association_cast(const From_Ty& from)
    {
        return dynamic_cast<To_Ty*>(from);
    }

    // Create an association to a sub-type of the owned object.  This 
    // function performs a dynamic downcast on the supplied
    // pointer/association type.  
    // If the downcast fails, the returned association will be set to nullptr.
    // Note - the returned association does NOT participate
    // in the lifetime management of the argument.  You
    // should NEVER delete the association returned from this
    // function.
    //
    template <typename To_Ty, typename From_Ty>
    association_to<To_Ty> association_cast(const owner_of<From_Ty>& from)
    {
        return dynamic_cast<To_Ty*>(from.get());
    }

    // Create an association to a sub-type of the shared object.  This 
    // function performs a dynamic downcast on the supplied
    // pointer/association type.  
    // If the downcast fails, the returned association will be set to nullptr.
    // Note - the returned association does NOT participate
    // in the lifetime management of the argument.  You
    // should NEVER delete the association returned from this
    // function.
    //
    template <typename To_Ty, typename From_Ty>
    association_to<To_Ty> association_cast(const shared_owner<From_Ty>& from)
    {
        return dynamic_cast<To_Ty*>(from.get());
    }
    
} // namespace Navtech

#endif // POINTER_TYPES_H