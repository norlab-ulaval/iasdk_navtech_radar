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

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <cstddef>
#include <array>
#include <vector>
#include <optional>
#include <typeinfo>
#include <memory>

#include "Heap_array.h"



namespace Navtech::Utility {

    // -------------------------------------------------------------------------------------------------------
    // Circular buffer API
    //
    // In this implementation, insertion-on-full does not fail - oldest
    // data will be over-written
    //
    // template <
    //      typename T                      // The type stored in the buffer
    //      std::size_t sz                  // Buffer size (fixed)
    // >
    // class Circular_buffer<T, sz> {
    // public:
    //     Insert a single element
    //     -----------------------
    //     - Insert by copy
    //     - Insert by move
    //
    //     void push(const T& in_val);      
    //     void push(T&& in_val);
    //
    //
    //     Insert a range of elements
    //     --------------------------
    //     - Range specified by two iterators.
    //     - Range specified by a span.
    //     If the number of elements in the input range exceeds the capacity
    //     of the buffer older elements will be overwritten
    //     
    //     template <typename Iterator_Ty> void push(Iterator_Ty begin, Iterator_Ty end);
    //     template <typename Iterator_Ty> void push(Iterator_Ty begin, std::size_t sz);
    //    
    //
    //     Insert from a container
    //     -----------------------
    //     - Containers passed as lvalues will have their elements copied.
    //     - Containers passed as rvalues will have their elements moved.
    //     If the number of elements in the input container exceeds the capacity
    //     of the buffer older elements will be overwritten
    //     
    //     template <template <typename> class Container_Ty> void push(const Container_Ty<T>& in_vals);
    //     template <template <typename> class Container_Ty> void push(Container_Ty<T>&& in_vals);
    //    
    //
    //     Insertion operator overloads
    //     ----------------------------
    //     The output stream operator has been overloaded so that individual elements
    //     and containers of elements can be inserted into the buffer as if it were
    //     an output stream.
    //     These functions behave exactly as their non-operator-overload equivalents.
    //
    //     Circular_buffer& operator<<(const T& in_val);      
    //     Circular_buffer& operator<<(T&& in_val);
    //     template <template <typename> class Container_Ty> Circular_buffer& operator<<(const Container_Ty<T>& in_vals);
    //     template <template <typename> class Container_Ty> Circular_buffer& operator<<(Container_Ty<T>&& in_vals)
    //  
    //   
    //     Retrieve n elements
    //     --------------------
    //     - Retrieve 1 element.  If the buffer is empty a nullopt value
    //       is returned.
    //
    //     - Retreive n elements into an output vector.  Elements will be moved from
    //       the container. 
    //       If n exceeds the size of the buffer, only 'size' elements will be returned.
    //
    //     - Retrieve from the buffer into a range specified by the two iterators
    //       If the number of elements specified by the input range is greater than
    //       the size of the buffer, only 'size' elements will be returned.
    //
    //     - Retrieve up to n elements into the receiver, starting at the supplied
    //       iterator.
    //       If n exceeds the size of the buffer, only 'size' elements will be returned.
    //    
    //     std::optional<T>                     pop();
    //     std::vector<T>                       pop_n(std::size_t n);
    //     template <typename Iterator_Ty> void pop_into(Iterator_Ty begin, Iterator_Ty end);
    //     template <typename Iterator_Ty> void pop_n_into(Iterator_Ty begin, std::size_t n);
    //    
    //     Buffer statistics
    //     -----------------
    //     - Number of elements in container.
    //     - Number of elements before overwriting of old elements occurs.
    //     - Does the buffer hold any data?
    //
    //     std::size_t size()     const;        
    //     std::size_t capacity() const;        
    //     bool        empty()    const;        
    // };


    // -------------------------------------------------------------------------------------------------------
    // Basic circular buffer (FIFO).
    //
    template <typename T, std::size_t sz>
    class Circular_buffer {
    public:
        void push(T&& in_val);
        void push(const T& in_val);
        template <template <typename... U> class Container_Ty> void push(const Container_Ty<T>& in_vals);
        template <template <typename... U> class Container_Ty> void push(Container_Ty<T>&& in_vals);
        template <typename Iterator_Ty> void push(Iterator_Ty first, Iterator_Ty last);
        template <typename Iterator_Ty> void push(Iterator_Ty first, std::size_t n);

        Circular_buffer& operator<<(const T& in_val);      
        Circular_buffer& operator<<(T&& in_val);
        template <template <typename...> class Container_Ty> Circular_buffer& operator<<(const Container_Ty<T>& in_vals);
        template <template <typename...> class Container_Ty> Circular_buffer& operator<<(Container_Ty<T>&& in_vals);

        std::optional<T> pop();
        std::vector<T>   pop_n(std::size_t n);
        template <typename Iterator_Ty> std::size_t pop_into(Iterator_Ty first, Iterator_Ty last);
        template <typename Iterator_Ty> std::size_t pop_n_into(Iterator_Ty first, std::size_t n);

        void clear();
        
        std::size_t size() const;
        std::size_t capacity() const;
        bool  empty() const;

    private:
        // using Container  = std::array<T, sz>;
        // using Iterator   = typename Container::iterator;
        // using Owning_ptr = std::unique_ptr<Container>;

        // Owning_ptr  buffer    { new Container { } };
        // Iterator    read      { std::begin(*buffer) };
        // Iterator    write     { std::begin(*buffer) };
        // std::size_t num_items { 0 };

        using Container = Utility::Heap_array<T, sz>;
        using Iterator  = typename Container::Iterator;

        Container   buffer    { };
        Iterator    read      { std::begin(buffer) };
        Iterator    write     { std::begin(buffer) };
        std::size_t num_items { 0 };
    };

 
 
    template <typename T, std::size_t sz>
    void Circular_buffer<T, sz>::push(T&& in_val)
    {
        using std::begin;
        using std::end;
        using std::move;
        
        *write = move(in_val);
        ++num_items;
        ++write;
        if (write == end(buffer)) write = begin(buffer);

        // If the capacity of the buffer has been
        // exceeded, the oldest data will have be over-written.
        // Reset the read iterator to point to the current-oldest
        // element.
        //
        if (num_items > sz) {
            num_items = sz;
            read = write;
        }
    }


    template <typename T, std::size_t sz>
    void Circular_buffer<T, sz>::push(const T& in_val)
    {
        using std::begin;
        using std::end;
        using std::move;
        
        *write = in_val;
        ++num_items;
        ++write;
        if (write == end(buffer)) write = begin(buffer);

        // If the capacity of the buffer has been
        // exceeded, the oldest data will have be over-written.
        // Reset the read iterator to point to the current-oldest
        // element.
        //
        if (num_items > sz) {
            num_items = sz;
            read = write;
        }
    }


    template <typename T, std::size_t sz>
    template <template <typename...> class Container_Ty>
    void Circular_buffer<T, sz>::push(const Container_Ty<T>& in_vals)
    {
        // Because the circular buffer may wrap we can't just do
        // a linear copy; so we have to do n insertions
        //
        for (const auto& elem : in_vals) {
            push(elem);
        }
    }


    template <typename T, std::size_t sz>
    template <template <typename...> class Container_Ty>
    void Circular_buffer<T, sz>::push(Container_Ty<T>&& in_vals)
    {
        using std::move;

        // Moving into a local container will 
        // 'empty' in_vals, giving the correct
        // move semantics.
        //
        Container_Ty local { move(in_vals) };

        // Because the circular buffer may wrap we can't just do
        // a linear copy; so we have to do n insertions
        //
        for (auto& elem : local) {
            push(move(elem));
        }
    }


    template <typename T, std::size_t sz>
    template <typename Iterator_Ty> 
    void Circular_buffer<T, sz>::push(Iterator_Ty first, Iterator_Ty last)
    {
        auto it = first;
        while (it != last) {
            push(*it);
            ++it;
        }
    }
    

    template <typename T, std::size_t sz>
    template <typename Iterator_Ty> 
    void Circular_buffer<T, sz>::push(Iterator_Ty first, std::size_t n)
    {
        using std::size_t;

        auto it = first;

        for (size_t i { 0 }; i < n; ++i) {
            push(*it);
            ++it;
        }
    }


    template <typename T, std::size_t sz>
    Circular_buffer<T, sz>& Circular_buffer<T, sz>::operator<<(const T& in_val)
    {
        push(in_val);
        return *this;
    }


    template <typename T, std::size_t sz>   
    Circular_buffer<T, sz>& Circular_buffer<T, sz>::operator<<(T&& in_val)
    {
        using std::move;

        push(move(in_val));
        return *this;
    }


    template <typename T, std::size_t sz>
    template <template <typename...> class Container_Ty> 
    Circular_buffer<T, sz>& Circular_buffer<T, sz>::operator<<(const Container_Ty<T>& in_vals)
    {
        push(in_vals);
        return *this;
    }


    template <typename T, std::size_t sz>
    template <template <typename...> class Container_Ty> 
    Circular_buffer<T, sz>& Circular_buffer<T, sz>::operator<<(Container_Ty<T>&& in_vals)
    {
        using std::move;

        push(move(in_vals));
        return *this;
    }


    template <typename T, std::size_t sz>
    std::optional<T> Circular_buffer<T, sz>::pop()
    {
        using std::nullopt;
        using std::begin;
        using std::end;
        using std::move;
      
        if (num_items == 0) return nullopt;  // Empty

        auto return_val_iter = read;

        --num_items;
        ++read;
        if (read == end(buffer)) read = begin(buffer);

        return move(*return_val_iter);
    }


    template <typename T, std::size_t sz>
    std::vector<T> Circular_buffer<T, sz>::pop_n(std::size_t n)
    {
        using std::vector;
        using std::move;
        using std::size_t;

        vector<T> elements { };
        elements.reserve(n);

        for (; n > 0; --n) {
            if (auto elem = pop(); elem.has_value()) {
                elements.push_back(move(elem.value()));
            }
            else {
                break;
            }
        }

        return elements;
    }


    template <typename T, std::size_t sz>
    template <typename Iterator_Ty> 
    std::size_t Circular_buffer<T, sz>::pop_into(Iterator_Ty first, Iterator_Ty last)
    {
        using std::move;
        using std::size_t;

        auto it { first };
        size_t num_popped { 0 };

        // Keep moving elements from the buffer
        // until you've filled the range, or the
        // buffer is empty
        //
        while (it != last) {
            if (auto elem = pop(); elem.has_value()) {
                *it = move(elem.value());
                ++it;
                ++num_popped;
            }
            else {
                break;
            }
        }
        
        return num_popped;
    }


    template <typename T, std::size_t sz>
    template <typename Iterator_Ty> 
    std::size_t Circular_buffer<T, sz>::pop_n_into(Iterator_Ty first, std::size_t n)
    {
        using std::move;
        using std::size_t;

        Iterator_Ty it { first };
        size_t num_popped { 0 };

        // Keep moving elements from the buffer
        // until you've filled the span, or the
        // buffer is empty
        //
        for (; n > 0; --n) {
            if (auto elem = pop(); elem.has_value()) {
                *it = move(elem.value());
                ++it;
                ++num_popped;
            }
            else {
                break;
            }
        }

        return num_popped;
    }
    

    template <typename T, std::size_t sz>
    void Circular_buffer<T, sz>::clear()
    {
        read      = std::begin(buffer);
        write     = std::begin(buffer);
        num_items = 0;
    }


    template <typename T, std::size_t sz>
    std::size_t Circular_buffer<T, sz>::size() const
    {
        return num_items;
    }


    template <typename T, std::size_t sz>
    std::size_t Circular_buffer<T, sz>::capacity() const
    {
        auto current_capacity = (sz - num_items);
        return current_capacity;
    }


    template <typename T, std::size_t sz>
    bool Circular_buffer<T, sz>::empty() const
    {
        bool is_empty = (num_items == 0);
        return is_empty;
    }


} // namespace Navtech::Utility


#endif // CIRCULAR_BUFFER_H