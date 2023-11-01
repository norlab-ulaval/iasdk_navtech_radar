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

#ifndef STATISTICAL_VALUE_H
#define STATISTICAL_VALUE_H

#include <cstddef>
#include <algorithm>
#include <numeric>
#include <cmath>


namespace Navtech::Utility {
    // ----------------------------------------------------------------------------
    // A Statistical_value represent a data item that maintains statistics about
    // its value.
    // As the object is written to, previous values are maintained in a rolling
    // buffer, the size of which is determined by the template parameter, sample_sz
    //
    // Basic statistics - mean, variance and standard deviation - are available.
    // An interface is provided to access the latest update value and the current
    // history.
    //
    // Note - The base template is used for sample_sz 2+; even though the default
    // is sample_sz = 1.  A simple_size of 1 invokes the template specialisation
    // (below).
    //
    // ----------------------------------------------------------------------------
    // Base template.
    //
    template <typename T, std::size_t sample_sz = 1>
    class Statistical_value {
    public:
        Statistical_value() = default;
        
        template <typename U = T>
        Statistical_value(U&& in_val);

        template <typename U>
        void update(U&& in_val);

        template <typename U = T>
        Statistical_value& operator=(U&& in_val);

        const T& latest() const;
        operator const T&() const;

        const std::array<T, sample_sz>& dataset() const;

        // Computed stats, based on the current
        // data set
        //
        T mean() const;
        T variance() const;
        T std_deviation() const;
        T delta() const;

        // Highest/lowest values observed by
        // this object
        //
        T min() const;
        T max() const;

        std::size_t size() const;
        bool empty() const;

    private:
        std::array<T, sample_sz> values { };
        std::size_t num_elems { };
        
        T min_val { };
        T max_val { };
    };


    template <typename T, std::size_t sample_sz>
    template <typename U>
    Statistical_value<T, sample_sz>::Statistical_value(U&& in_val)
    {
        update(std::forward<U>(in_val));
    }


    template <typename T, std::size_t sample_sz>
    template <typename U>
    void Statistical_value<T, sample_sz>::update(U&& in_val)
    {
        if (++num_elems > sample_sz) num_elems = sample_sz;
        std::rotate(values.rbegin(), values.rbegin() + 1, values.rend());
        values[0] = std::forward<U>(in_val);

        if (num_elems == 1) {
            min_val = values[0];
            max_val = values[0];
        }

        if (values[0] > max_val) max_val = in_val;
        if (values[0] < min_val) min_val = in_val;
    }


    template <typename T, std::size_t sample_sz>
    template <typename U>
    Statistical_value<T, sample_sz>& Statistical_value<T, sample_sz>::operator=(U&& in_val)
    {
        update(std::forward<U>(in_val));
        return *this;
    }


    template <typename T, std::size_t sample_sz>
    const T& Statistical_value<T, sample_sz>::latest() const
    {
        // Return the latest value
        //
        return values[0];
    }


    template <typename T, std::size_t sample_sz>
    Statistical_value<T, sample_sz>::operator const T&() const
    {
        // Return the latest value
        //
        return latest();
    }


    template <typename T, std::size_t sample_sz>
    const std::array<T, sample_sz>& Statistical_value<T, sample_sz>::dataset() const
    {
        return values;
    }


    template <typename T, std::size_t sample_sz>
    T Statistical_value<T, sample_sz>::mean() const
    {
        if (num_elems == 0) return { };

        T total   = std::accumulate(values.begin(), values.end(), T { });
        T average = total / num_elems;
        return average;
    }


    template <typename T, std::size_t sample_sz>
    T Statistical_value<T, sample_sz>::variance() const
    {
        // Since variance is computed by dividing by
        // n, for the case where n == 0, this would 
        // cause a divide-by-zero; so return NaN
        //
        if (num_elems == 0) return std::numeric_limits<T>::quiet_NaN();

        T var { };
        T mu  { mean() };

        for (unsigned i { 0 }; i < num_elems; ++i) {
            var += static_cast<T>(pow(values[i] - mu, 2));
        }
        return var / num_elems;
    }


    template <typename T, std::size_t sample_sz>
    T Statistical_value<T, sample_sz>::std_deviation() const
    {
        return std::sqrt(variance());
    }


    template <typename T, std::size_t sample_sz>
    T Statistical_value<T, sample_sz>::delta() const
    {
        return values[0] - values[1];
    }


    template <typename T, std::size_t sample_sz>
    std::size_t Statistical_value<T, sample_sz>::size() const
    {
        return num_elems;
    }


    template <typename T, std::size_t sample_sz>
    bool Statistical_value<T, sample_sz>::empty() const
    {
        return size() == 0;
    }


    template <typename T, std::size_t sample_sz>
    T Statistical_value<T, sample_sz>::min() const
    {
        return min_val;
    }


    template <typename T, std::size_t sample_sz>
    T Statistical_value<T, sample_sz>::max() const
    {
        return max_val;
    }


    // ----------------------------------------------------------------------------
    // Specialisation for sample_sz == 1.
    //
    template <typename T>
    class Statistical_value<T, 1> {
    public:
        Statistical_value() = default;
        
        template <typename U = T>
        Statistical_value(U&& in_val);

        template <typename U>
        void update(U&& in_val);

        template <typename U = T>
        Statistical_value& operator=(U&& in_val);

        const T& latest() const;
        operator const T&() const;
        std::array<T, 1> dataset() const;

        // Rolling average
        //
        T mean() const;

        // For a sample size of 1, these measures may
        // be of limited value; but are supplied for
        // interface compatibility/ 
        //
        T variance() const;
        T std_deviation() const;
        T delta() const;

        // Highest/lowest values observed by
        // this object
        //
        T min() const;
        T max() const;

        std::size_t size() const;
        bool empty() const;

    private:
        bool updated { };
        T current    { };
        T previous   { };
        T min_val    { };
        T max_val    { };
    };


    template <typename T>
    template <typename U>
    Statistical_value<T, 1>::Statistical_value(U&& in_val)
    {
        update(std::forward<U>(in_val));
    }


    template <typename T>
    template <typename U>
    void Statistical_value<T, 1>::update(U&& in_val)
    {
        if (!updated) {
            current  = std::forward<U>(in_val);
            previous = current;
            min_val  = current;
            max_val  = current;
            updated  = true;
        }
        else {
            previous = std::move(current);
            current  = std::forward<U>(in_val);
           
            if (current > max_val) max_val = current;
            if (current < min_val) min_val = current;
        }
    }


    template <typename T>
    template <typename U>
    Statistical_value<T, 1>& Statistical_value<T, 1>::operator=(U&& in_val)
    {
        update(std::forward<U>(in_val));
        return *this;
    }


    template <typename T>
    const T& Statistical_value<T, 1>::latest() const
    {
        return current;
    }


    template <typename T>
    Statistical_value<T, 1>::operator const T&() const
    {
        return latest();
    }


    template <typename T>
    std::array<T, 1> Statistical_value<T, 1>::dataset() const
    {
        return std::array<T, 1> { current };
    }


    template <typename T>
    T Statistical_value<T, 1>::mean() const
    {
        return (previous + (delta() / static_cast<T>(2.0)));
    }


    template <typename T>
    T Statistical_value<T, 1>::variance() const
    {
        if (!updated) return std::numeric_limits<T>::quiet_NaN();

        return static_cast<T>(std::pow(current - mean(), 2));
    }


    template <typename T>
    T Statistical_value<T, 1>::std_deviation() const
    {
        return static_cast<T>(std::sqrt(variance()));
    }


    template <typename T>
    T Statistical_value<T, 1>::delta() const
    {
        return current - previous;
    }


    template <typename T>
    std::size_t Statistical_value<T, 1>::size() const
    {
        return (updated ? 1 : 0);
    }


    template <typename T>
    bool Statistical_value<T, 1>::empty() const
    {
        return !updated;
    }


    template <typename T>
    T Statistical_value<T, 1>::min() const
    {
        return min_val;
    }


    template <typename T>
    T Statistical_value<T, 1>::max() const
    {
        return max_val;
    }


    // ----------------------------------------------------------------------------
    // Specialisation for sample_sz = 0
    // - just in case someone does something dumb!
    //
    template <typename T>
    class Statistical_value<T, 0> {
    public:
        template <typename U>
        void update(U&&)                       {  }

        template <typename U = T>
        Statistical_value& operator=(U&&)   { return *this; }

        T mean() const                      { return std::numeric_limits<T>::quiet_NaN(); }
        T variance() const                  { return std::numeric_limits<T>::quiet_NaN(); }
        T std_deviation() const             { return std::numeric_limits<T>::quiet_NaN(); }

        T min() const                       { return { }; }
        T max() const                       { return { }; }

        std::size_t size() const            { return 0; }
        bool empty() const                  { return true; }
    };

} // namespace Navtech::Utility


#endif // STATISTICAL_VALUE_H