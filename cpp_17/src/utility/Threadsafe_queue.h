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

#ifndef THREADSAFE_QUEUE_H
#define THREADSAFE_QUEUE_H


#include <queue>
#include <optional>
#include <mutex>
#include <condition_variable>

namespace Navtech::Utility {
    
    template <typename T>
    class alignas(alignof(std::condition_variable)) Threadsafe_queue : private std::queue<T> {
    public:
        template <typename U> void push(U&& in_val);
        std::optional<T> pop();
        std::optional<T> try_pop();
        void clear();
        bool empty() const;
        std::size_t size() const;

    private:
        using Queue = std::queue<T>;

        mutable std::mutex      mtx { };
        std::condition_variable has_data { };

        bool cancelled { false };
    };


    template <typename T>
    template <typename U> 
    void Threadsafe_queue<T>::push(U&& in_val)
    {
        std::lock_guard lock { mtx };

        Queue::push(std::forward<U>(in_val));
        has_data.notify_all();
    }


    template <typename T>
    std::optional<T> Threadsafe_queue<T>::pop()
    {
        std::unique_lock lock { mtx };
        
        while (Queue::empty() && !cancelled) {
            has_data.wait(lock);
        }

        if (cancelled) return std::nullopt;

        T out_val { std::move(Queue::front()) };
        Queue::pop();
        return out_val;
    }


    template <typename T>
    std::optional<T> Threadsafe_queue<T>::try_pop()
    {
        std::unique_lock lock { mtx };

        if (Queue::empty()) return std::nullopt;

        std::optional<T> out_val { move(Queue::front()) };
        Queue::pop();
        return out_val;
    }


    template <typename T>
    bool Threadsafe_queue<T>::empty() const
    {
        std::lock_guard lock { mtx };

        return Queue::empty();
    }


    template <typename T>
    std::size_t Threadsafe_queue<T>::size() const
    {
        std::lock_guard lock { mtx };
    
        return Queue::size();
    }


    template <typename T>
    void Threadsafe_queue<T>::clear()
    {
        std::lock_guard lock { mtx };
    
        while (!Queue::empty()) {
            Queue::pop();
        }
        cancelled = true;
        has_data.notify_all();
    }

} // namespace Navtech::Utility

#endif // THREADSAFE_QUEUE_H 