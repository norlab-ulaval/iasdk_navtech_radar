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

#ifndef FLAG_SET_H
#define FLAG_SET_H

#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "Bit_ops.h"

namespace Navtech::Utility {

    // ------------------------------------------------------------------------------------------------------- 
    // Flag_set API
    //
    // Flag_set represents a set of N manually-reset signals.
    // Clients may set/test one or more flags.  The class allows 
    // either a conjunctive (check all flags to be set) test, or 
    // disjunctive test (check for *any* flag to be set).
    //
    // any_of(Bitmask bits_to_check)        - Disjunctive test.
    // all_of(Bitmask bits_to_check)        - Conjunctive test.
    //
    // ----------------------------------------------------------
    // Set one or more flags
    // 
    // void set(Bitmask bits_to_set);
    //
    // ----------------------------------------------------------
    // Read current state of flags
    //
    // operator Bitmask() const;
    //
    // ----------------------------------------------------------
    // Reset any signalled flags
    //
    // void clear(Bitmask bits_to_clear);
    // void clear_all();
    //
    //
    // Return the state of a single flag.  These operators
    // allow the Flag_set object to be accessed like an array;
    // each bit is indexed (from zero).
    // This allows code like the following:
    //
    // Utility::Flag_set<8> flags { };
    // flags[0] = 1;         // set
    // flags[0] = 0;         // clear
    // flags[1] = flags[0];  // copy state
    //
    // inline int  operator[](unsigned int flag) const;
    // inline Flag operator[](unsigned int flag);
    //
    // -------------------------------------------------------------------------------------------------------

    // -------------------------------------------------
    // Event flag exceptions
    //
    class invalid_flag : public std::out_of_range {
    public:
        invalid_flag(const char* str) : std::out_of_range(str) {}
        invalid_flag() : std::out_of_range("Invalid flag index") {}
    };


    // -------------------------------------------------
    // As the number of flags need not be a multiple of
    // 8, this compile-time function rounds up to the nearest
    // word length.
    //
    constexpr inline std::size_t to_nearest_byte(unsigned int num_bits)
    {
        if (num_bits <= 8)  return 8;
        if (num_bits <= 16) return 16;
        if (num_bits <= 32) return 32;
        return 0;
    }


    // -------------------------------------------------
    // Traits class for determining the underlying
    // type used to store the event flags.  Note there
    // is a maximum of 32 flags per event group.
    //
    template <std::size_t num_bits>
    struct Flag_set_traits {  };

    template <>
    struct Flag_set_traits<8>  { using type = std::uint8_t; };

    template <>
    struct Flag_set_traits<16> { using type = std::uint16_t; };

    template <>
    struct Flag_set_traits<32> { using type = std::uint32_t; };


    // -------------------------------------------------
    // Event_flags represents a set of N manually-reset
    // signals.
    //
    template <std::size_t num_flags>
    class Flag_set {
    public:
        using Bitmask = typename Flag_set_traits<to_nearest_byte(num_flags)>::type;

        Flag_set() = default;

        bool any_of(const Bitmask bits_to_check) const;
        bool all_of(const Bitmask bits_to_check) const;

        void set  (const Bitmask bits_to_set);
        void clear(const Bitmask bits_to_clear);
        void clear_all();
        operator Bitmask() const;


        // ----------------------------------------
        // Proxy for setting/clearing individual
        // event flags.
        //
        class Flag {
        public:
            // Signal / clear the flag
            //
            inline Flag& operator=(int val);
            inline Flag& operator=(const Flag& rhs);

            // Return the flag's state
            //
            inline operator int() const;

        private:
            friend class Flag_set;
            Flag(Flag_set& owner, unsigned num) : parent(&owner), flag(num) { }

            Flag_set* parent;
            unsigned     flag;
        };

        // Return the state of a single event flag
        //
        inline bool operator[](unsigned int flag) const;
        inline Flag operator[](unsigned int flag);
        
    private:
        Bitmask state { };
    };


    template <std::size_t num_flags>
    void Flag_set<num_flags>::set(const Flag_set<num_flags>::Bitmask bits_to_set)
    {
        state |= bits_to_set;
    }


    template <std::size_t num_flags>
    void Flag_set<num_flags>::clear(const Flag_set<num_flags>::Bitmask bits_to_clear)
    {
        state &= ~(bits_to_clear);
    }


    template <std::size_t num_flags>
    void Flag_set<num_flags>::clear_all()
    {
        clear(bit_range(0, num_flags));
    }


    template <std::size_t num_flags>
    bool Flag_set<num_flags>::operator[](unsigned int flag) const
    {
        if ((flag < 0) || (flag >= num_flags)) throw invalid_flag { };
        return (is_set(state, flag));
    }


    template <std::size_t num_flags>
    typename Flag_set<num_flags>::Flag
    Flag_set<num_flags>::operator[](unsigned int flag)
    {
        if (flag >= num_flags) throw invalid_flag { };
        return Flag { *this, flag };
    }


    template <std::size_t num_flags>
    Flag_set<num_flags>::operator Flag_set<num_flags>::Bitmask() const
    {
        return state;
    }


    template <std::size_t num_flags>
    bool Flag_set<num_flags>::any_of(const Flag_set<num_flags>::Bitmask flags_to_check) const
    {
        Bitmask bit_mask = (flags_to_check & bit_range(0, num_flags));

        // AND the current bit pattern with the bits to check.
        // If any match this will yield a non-zero result.
        //
        return ((state & bit_mask) != 0);
    }


    template <std::size_t num_flags>
    bool Flag_set<num_flags>::all_of(const Flag_set<num_flags>::Bitmask flags_to_check) const
    {
        Bitmask bit_mask = (flags_to_check & bit_range(0, num_flags));

        // If all the flags are set then then ANDing the bit_mask
        // with the state should yield itself.
        //
        return ((state & bit_mask) == bit_mask);
    }


    template <std::size_t num_flags>
    typename Flag_set<num_flags>::Flag&
    Flag_set<num_flags>::Flag::operator=(int val)
    {
        if (val == 0) parent->clear(bit(flag));
        else          parent->set  (bit(flag));
        return *this;
    }


    template <std::size_t num_flags>
    Flag_set<num_flags>::Flag::operator int() const
    {
        return is_set(parent->state, flag) ? 1 : 0;
    }


    template <std::size_t num_flags>
    typename Flag_set<num_flags>::Flag&
    Flag_set<num_flags>::Flag::operator=(const Flag& rhs)
    {
        *this = static_cast<int>(rhs);
        return *this;
    }


} // namespace Navtech::Utility


#endif // FLAG_SET_H