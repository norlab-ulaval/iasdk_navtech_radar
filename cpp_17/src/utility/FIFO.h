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

#ifndef FIFO_H
#define FIFO_H

#include <queue>
#include <optional>
#include <stdexcept>

namespace Navtech::Utility {

    // ----------------------------------------------------------------------------
    // FIFO provides a new interface for std::queue which is compatible
    // with the interface of Utility::Threadsafe_queue. This allows a
    // FIFO and a Threadsafe_queue to be interchanged without changing the
    // client application.
    //
    template <typename T>
    class FIFO : private std::queue<T> {
    public:
        template <typename U> void push(U&& in_val);
        T pop();
        std::optional<T> try_pop();
        void clear();
        
        using std::queue<T>::empty;
        using std::queue<T>::size;
    
    private:
        using Queue = std::queue<T>;
    };


    template <typename T>
    template <typename U> 
    void FIFO<T>::push(U&& in_val)
    {
        Queue::push(std::forward<U>(in_val));
    }


    template <typename T>
    T FIFO<T>::pop()
    {
        if (Queue::empty()) throw std::runtime_error { "Empty FIFO" };
        T out_val { Queue::front() };
        Queue::pop();
        return out_val;
    }


    template <typename T>
    std::optional<T> FIFO<T>::try_pop()
    {
        if (empty()) return std::nullopt;

        std::optional<T> out_val { std::move(Queue::front()) };
        Queue::pop();
        return out_val;
    }


    template <typename T>
    void FIFO<T>::clear()
    {
        while (!Queue::empty()) {
            Queue::pop();
        }
    }


}  // namespace Navtech::Utility

#endif // FIFO_H