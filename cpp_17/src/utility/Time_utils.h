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

#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <ctime>
#include <mutex>

#include <iostream>

// A set of features for the measurement and processing of time.  Designed to replace the complex, 
// and overly verbose, std::chrono library.  As such, the Time library is a subset of the 
// std::chrono library

namespace Navtech::Time {

    // Forward references
    //
    namespace Real_time              { class Observation; }
    namespace Thread_safe            { class Duration; }
    namespace Thread_safe::Monotonic { class Observation; }
    namespace Thread_safe::Real_time { class Observsation; }

    // Tag types for operator overloading selection
    //
    class Duration_tag    { };
    class Observation_tag { };

    template <typename T>
    using is_duration = typename std::enable_if<std::is_base_of<Duration_tag, T>::value, bool>::type;

    template <typename T>
    using is_observation = typename std::enable_if<std::is_base_of<Observation_tag, T>::value, bool>::type;


    // The underlying clock representation
    //
    using Tick_type = std::int64_t;
    

    // --------------------------------------------------------------------------------
    // A Duration represents a period of time, as a number of nanoseconds.
    //
    // Durations act similar to numerical types, but with modified characteristics due 
    // to the fact that represent a physical phenomenon, rather than being purely numeric.
    // 
    // - Durations may be compared for equality/inequality and greater-than/less-than.
    // - Addition represents lengthening the duration. Two Durations may be added to yield 
    //   a new Duration whose value is the simple sum of the two Durations.
    // - Subtraction represents shortening the duration. Subtracting a larger Duration 
    //   from a smaller Duration will yield a negative Duration (which is not physically 
    //   realizable).
    // - If the difference between two Durations is required, prefer Monotonic::abs_diff.
    // - Two Durations cannot be multiplied since this has no physical meaning.
    // - Two Durations may be divided. The result will be a positive integer, representing
    //   the number of times the divisor Duration goes into the numerator.
    // - Durations may be scaled by multiplying or dividing by a scale factor; either 
    //   integer or floating-point.
    //
    // See the unit tests for examples of Duration use cases
    //

    class Duration : public Duration_tag {
    public:
        constexpr Duration() = default;

        constexpr explicit Duration(Tick_type init) : duration { init }
        {
        }

        constexpr explicit Duration(const std::timespec& init) :
            duration { std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds { init.tv_sec } + std::chrono::nanoseconds { init.tv_nsec }) }
        {
        }

        constexpr Tick_type ticks() const
        {
            return duration.count();
        }

        constexpr std::uint64_t to_nsec() const
        {
            return ticks();
        }

        // Fractional value
        //
        float to_usec() const
        {
            return (duration.count() / 1000.0f);
        }

        float to_msec() const
        {
            return (to_usec() / 1000.0f);
        }

        float to_sec() const
        {
            return (to_msec() / 1000.0f);
        }

        // Integer part only
        //
        std::uint64_t nanoseconds() const
        {
            return ticks();
        }

        std::uint64_t microseconds() const
        {
            return static_cast<std::uint64_t>(std::round(to_usec()));
        }

        std::uint64_t milliseconds() const
        {
            return static_cast<std::uint64_t>(std::round(to_msec()));
        }

        std::uint64_t seconds() const
        {
            return static_cast<std::uint64_t>(std::round(to_sec()));
        }

        Duration to_nearest_microsecond() const
        {
            float num_usec = std::round(duration.count() / 1000.0f);
            return Duration { static_cast<Tick_type>(num_usec * 1000)  };
        }

        Duration to_nearest_millisecond() const
        {
            float num_msec = std::round(duration.count() / 1'000'000.0f);
            return Duration { static_cast<Tick_type>(num_msec * 1000'000)  };
        }

        Duration to_nearest_second() const
        {
            float num_sec = std::round(duration.count() / 1'000'000'000.0f);
            return Duration { static_cast<Tick_type>(num_sec * 1'000'000'000)  };
        }

        constexpr static Duration forever()
        {
            return Duration { static_cast<Tick_type>(Duration_type::max().count()) };
        }

        template <typename Chrono_Ty>
        Chrono_Ty to_chrono() const
        {
            return std::chrono::duration_cast<Chrono_Ty>(duration);
        }

        std::string to_string() const;

        Duration& operator+=(const Duration& rhs);
        Duration& operator-=(const Duration& rhs);
        Duration& operator*=(double rhs);
        Duration& operator/=(double rhs);

        Duration& operator+=(const Thread_safe::Duration& rhs);
        Duration& operator-=(const Thread_safe::Duration& rhs);

    protected:
        using Duration_type = std::chrono::nanoseconds;

        Duration_type duration { };
    };
    
    inline Duration to_nearest_microsecond(const Duration& d)
    {
        return d.to_nearest_microsecond();
    }

    inline Duration to_nearest_millisecond(const Duration& d)
    {
        return d.to_nearest_millisecond();
    }

    inline Duration to_nearest_second(const Duration& d)
    {
        return d.to_nearest_second();
    }

    constexpr Duration operator""_nsec(unsigned long long val)
    {
        return Duration { static_cast<Tick_type>(val) };
    }

    constexpr Duration operator""_usec(unsigned long long val)
    {
        return operator""_nsec(val * 1000ULL);
    }


    constexpr Duration operator""_msec(unsigned long long val)
    {
        return operator""_usec(val * 1000ULL);
    }


    constexpr Duration operator""_msec(long double val)
    {
        return operator""_usec(static_cast<Tick_type>(val * 1000.0L));
    }


    constexpr Duration operator""_sec(unsigned long long val)
    {
        return operator""_msec(val * 1000ULL);
    }


    constexpr Duration operator""_sec(long double val)
    {
        return operator""_msec(val * 1000.0L);
    }


    constexpr Duration operator""_min(unsigned long long val)
    {
        return operator""_sec(val * 60ULL);
    }


    constexpr Duration operator""_min(long double val)
    {
        return operator""_sec(val * 60.0L);
    }


    constexpr Duration operator""_hour(unsigned long long val)
    {
        return operator""_min(val * 60ULL);
    }


    constexpr Duration operator""_hour(long double val)
    {
        return operator""_min(val * 60.0L);
    }


    constexpr Duration operator""_day(unsigned long long val)
    {
        return operator""_hour(val * 24ULL);
    }


    constexpr Duration operator""_day(long double val)
    {
        return operator""_hour(val * 24.0L);
    }
    

    constexpr inline Duration to_nsec_duration(unsigned long long val)
    {
        return Duration { static_cast<Tick_type>(val) };
    }
    

    constexpr inline Duration to_usec_duration(unsigned long long val)
    {
        return operator""_nsec(static_cast<Tick_type>(val * 1000.0L));
    }


    constexpr inline Duration to_msec_duration(long double val)
    {
        return operator""_usec(static_cast<Tick_type>(val * 1000.0L));
    }


    constexpr inline Duration to_sec_duration(long double val)
    {
        return operator""_msec(val * 1000.0L);
    }


    inline std::ostream& operator<<(std::ostream& os, const Duration& d)
    {
        os << d.to_string();
        return os;
    }


    // ====================================================================================
    //
    // The Monotonic namespace represents an independent time frame. That is, this time 
    // is not related to wall clock time (for example, it can be time since last reboot), 
    // and is most suitable for measuring intervals.
    //
    namespace Monotonic {

        // --------------------------------------------------------------------------------
        // An Observation represents a point in time; more specifically, it represents a 
        // duration since the start of the Clock’s epoch.  An Observation is used to 
        // mark/record an event occurrence.
        // Observations are usually generated by the Clock (via the now() function); but may 
        // be computed from other Observations using Durations as offsets.
        // Since Monotonic::Clocks do not align with a real-time (wall) clock there is no 
        // interface for extracting the day/date/month/etc. from a Monotonic::Observation.
        //
        // Observations support some operator overloads, as follows:
        // - Observations may be compared for equality/inequality.  Equality indicates that 
        //   both Observations mark the same moment in time.
        // - Greater-than indicates that one Observation is later the other; less-than indicates 
        //   that one Observation is earlier than the other.
        // - Adding two Observations has no meaning.
        // - A Duration may be added to, or subtracted from, an Observation.  The result is 
        //   a new Observation.
        // - Subtracting two Observations yields the Duration between them.  
        // - If a later Observation (t1) is subtracted from an earlier Observation (t0) the 
        //   result will yield a negative Duration.  That is, t0 – t1 => -d. 
        //
        class Observation : public Observation_tag {
        public:
            Observation() = default;
            explicit Observation(const Duration& init);
            
            Duration since_epoch()  const;
            Real_time::Observation to_real_time() const;

            std::timespec to_ntp() const;

            Observation& operator+=(const Duration& rhs);
            Observation& operator-=(const Duration& rhs);

        protected:
            std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> time { };
        };

        // Non-member API
        //
        Duration abs_diff(const Observation& lhs, const Observation& rhs);


        // --------------------------------------------------------------------------------
        // Monotonic::Clock represents a steady clock.  Observations of this clock’s time 
        // will only ever increase; never decrease.  The time between ticks of the clock are 
        // constant.
        // The tick period for the clock is 1 microsecond.
        //
        class Clock {
        public:
            static Observation now();
            static Real_time::Observation started_at();
        };

        // Non-member API
        //
        Observation now();

        // A null_time represents an unassigned time observation.
        //
        extern Observation null_time;


        // --------------------------------------------------------------------------------
        //
        void sleep_for(const Duration& sleep_period);
        void sleep_until(const Observation& wakeup_time);

        void sleep_for(const Thread_safe::Duration& sleep_period);
        void sleep_until(const Thread_safe::Monotonic::Observation& wakeup_time);

    } // namespace Monotonic


    // ====================================================================================
    // The Real_time namespace represents the time as represented by the system’s real-time 
    // clock.
    //
    namespace Real_time {

        // --------------------------------------------------------------------------------
        // An Observation represents a point in time; more specifically, it represents a 
        // duration since the start of the Clock’s epoch.  An Observation is used to 
        // mark/record an event occurrence.
        // Observations are usually generated by the Clock (via the now() function); but may 
        // be computed from other Observations using Durations as offsets.
        // Real_time::Observation provides an interface for providing day/date/month/etc. 
        // and providing formatted output.
        //
        // Observations support some operator overloads, as follows:
        // - Observations may be compared for equality/inequality.  Equality indicates that 
        //   both Observations mark the same moment in time.
        // - Greater-than indicates that one Observation is later the other; less-than 
        //   indicates that one Observation is earlier than the other.
        // - Adding two Observations has no meaning.
        // - A Duration may be added to, or subtracted from, an Observation.  The result is 
        //   a new Observation.
        // - Since it cannot be guaranteed that two subsequent Observations will yield increasing 
        //   time points, subtracting two Observations may lead to a negative Duration.
        // - To find the absolute difference between two Observations use abs_diff.
        //
        class Observation : public Observation_tag {
        public:
            Observation() = default;
            explicit Observation(const Duration& init);
            explicit Observation(const std::time_t& init);
            explicit Observation(const std::timespec& init);

            Duration since_epoch()  const;
            int year()              const;
            int month()             const;
            int day()               const;
            int hour()              const;
            int minute()            const;
            int second()            const;
            int milliseconds()      const;
            int microseconds()      const;

            Observation& format_as(std::string_view fmt);
            std::string to_string() const;

            std::time_t   to_time_t() const;
            std::timespec to_ntp()    const;
            Monotonic::Observation to_monotonic() const;

            Observation& operator+=(const Duration& rhs);
            Observation& operator-=(const Duration& rhs);

        protected:
            std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> time { };
            std::string format { "%FT%T.%msZ" };

            Duration get_subseconds() const;
        };

        // Non-member API
        //
        std::ostream& operator<<(std::ostream& os, const Observation& obs);
    
        Duration abs_diff(const Observation& lhs, const Observation& rhs);


        // --------------------------------------------------------------------------------
        // Real_time::Clock represents the system-wide real time (wall) clock, in UTC(GMT).
        // It may not be monotonic: on most systems, the system time can be adjusted at any 
        // moment. 
        // The epoch of the Clock is unspecified, but most implementations use Unix Time 
        // (that is, time since 00:00:00 Coordinated Universal Time (UTC), Thursday, 1 January 1970).
        // The tick period for the clock is 1 microsecond.
        //
        class Clock {
        public:
            static Observation now();
        };

        // Non-member API
        //
        Observation now(); 

        // A null_time represents an unassigned time observation.
        //
        extern Observation null_time;

    } // namespace Real_time


    // ====================================================================================
    // The Thread_safe namespace contains thread-safe versions of the Time types.
    // For the most part, these types can be used interchangeably; although the thread-safe
    // versions are likely to be less performant, due to the thread-locking mechanisms
    // required.  In general, prefer the non-thread-safe versions.
    //
    namespace Thread_safe {

        // --------------------------------------------------------------------------------
        // Thread_safe::Duration
        //
        class Duration : private Time::Duration {
        public:
            using Time::Duration::Duration;
            using Time::Duration::forever;

            Duration(const Time::Duration& d);
            Duration(const Duration& other);
            Duration& operator=(Duration rhs);
            friend void swap(Duration& lhs, Duration& rhs);

            Tick_type    ticks()   const;
            float to_nsec() const;
            float to_usec() const;
            float to_msec() const;
            float to_sec()  const;

            std::int64_t nanoseconds()  const;
            std::int64_t microseconds() const;
            std::int64_t milliseconds() const;
            std::int64_t seconds()      const;

            Duration     to_nearest_millisecond() const;
            Duration     to_nearest_second() const;
            
            template <typename Chrono_Ty>
            Chrono_Ty to_chrono() const
            {
                std::lock_guard lock { mtx };
                return Time::Duration::to_chrono<Chrono_Ty>();
            }

            std::string to_string() const;

            Time::Duration unlocked() const;

            Duration& operator+=(const Duration& rhs);
            Duration& operator-=(const Duration& rhs);
            Duration& operator*=(double rhs);
            Duration& operator/=(double rhs);

        private:
            mutable std::mutex mtx { };
        };


        inline Duration to_nearest_millisecond(const Duration& d)
        {
            return d.to_nearest_millisecond();
        }


        inline Duration to_nearest_second(const Duration& d)
        {
            return d.to_nearest_second();
        }


        inline std::ostream& operator<<(std::ostream& os, const Duration& d)
        {
            os << d.to_string();
            return os;
        }

        // ====================================================================================
        //
        namespace Monotonic {

            // --------------------------------------------------------------------------------
            // Thread-safe::Monotonic::Observation
            //
            class Observation : private Time::Monotonic::Observation {
            public:
                using Time::Monotonic::Observation::Observation;

                Observation(const Time::Monotonic::Observation& other);

                // Copy-move policy
                //
                Observation(const Observation& other);
                Observation& operator=(Observation rhs);
                friend void swap(Observation& lhs, Observation& rhs);
                
                Time::Duration since_epoch()  const;
                std::timespec to_ntp() const;
                Time::Real_time::Observation to_real_time() const;

                Observation& operator+=(const Time::Duration& rhs);
                Observation& operator-=(const Time::Duration& rhs);
                Observation& operator+=(const Time::Thread_safe::Duration& rhs);
                Observation& operator-=(const Time::Thread_safe::Duration& rhs);

            private:
                mutable std::mutex mtx { };
            };
        
        } // namespace Monotonic


        // ====================================================================================
        //
        namespace Real_time {

            // --------------------------------------------------------------------------------
            // Thread-safe::Real_time::Observation
            //
            class Observation : private Time::Real_time::Observation {
            public:
                using Time::Real_time::Observation::Observation;

                Observation(const Time::Real_time::Observation& other);

                // Copy-move policy
                //
                Observation(const Observation& other);
                Observation& operator=(Observation rhs);
                friend void swap(Observation& lhs, Observation& rhs);

                Time::Duration since_epoch() const;
                int year()              const;
                int month()             const;
                int day()               const;
                int hour()              const;
                int minute()            const;
                int second()            const;
                int milliseconds()      const;
                int microseconds()      const;

                Observation& format_as(std::string_view fmt);
                std::string to_string() const;

                std::time_t   to_time_t() const;
                std::timespec to_ntp()    const;
                Monotonic::Observation to_monotonic() const;

                Observation& operator+=(const Time::Duration& rhs);
                Observation& operator-=(const Time::Duration& rhs);
                Observation& operator+=(const Time::Thread_safe::Duration& rhs);
                Observation& operator-=(const Time::Thread_safe::Duration& rhs);

            private:
                mutable std::mutex mtx { };
            };


            // Non-member API
            //
            std::ostream& operator<<(std::ostream& os, const Observation& obs);


        } // namespace Real_time

    } // namespace Thread_safe


    // --------------------------------------------------------------------------------
    // Template operator overloads.
    //
    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr Duration operator+(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        // NOTE: Overflow behaviour is undefined

        return Duration { lhs.ticks() + rhs.ticks() };
    }


    template <typename Obs_Ty, typename Dur_Ty, 
        is_observation<Obs_Ty> = true,
        is_duration<Dur_Ty>    = true>
    constexpr Obs_Ty operator+(const Obs_Ty& lhs, const Dur_Ty& rhs)
    {
        auto result = lhs.since_epoch().ticks() + rhs.ticks();

        // Cannot have a time observation before the epoch (zero)
        //
        if (result < 0) return Obs_Ty { Time::Duration { 0 } };
        else            return Obs_Ty { Time::Duration { result } };
    }


    template <typename Obs_Ty, typename Dur_Ty, 
        is_observation<Obs_Ty> = true,
        is_duration<Dur_Ty>    = true>
    constexpr Obs_Ty operator+(const Dur_Ty&  lhs, const Obs_Ty&rhs)
    {
        auto result = rhs.since_epoch().ticks() + lhs.ticks();

        // Cannot have a time observation before the epoch (zero)
        //
        if (result < 0) return Obs_Ty { Time::Duration { 0 } };
        else            return Obs_Ty { Time::Duration { result } };
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr Duration operator-(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        // NOTE: Underflow behaviour is undefined

        return Duration { lhs.ticks() - rhs.ticks() };
    }


    template <typename Obs_Ty, typename Dur_Ty, 
        is_observation<Obs_Ty> = true,
        is_duration<Dur_Ty>    = true>
    constexpr Obs_Ty operator-(const Obs_Ty& lhs, const Dur_Ty& rhs)
    {
        auto result = lhs.since_epoch().ticks() - rhs.ticks();

        // Cannot have a time observation before the epoch (zero)
        //
        if (result < 0) return Obs_Ty { Time::Duration { 0 } };
        else            return Obs_Ty { Time::Duration { result } };
    }


    template <typename Obs_Ty1, typename Obs_Ty2, 
        is_observation<Obs_Ty1> = true,
        is_observation<Obs_Ty2> = true>
    constexpr Duration operator-(const Obs_Ty1& lhs, const Obs_Ty2& rhs)
    {
        return Duration { lhs.since_epoch().ticks() - rhs.since_epoch().ticks() };
    }


    template <typename Dur_Ty,
        is_duration<Dur_Ty> = true>
    constexpr Duration operator*(const Dur_Ty& lhs, double rhs)
    {
        // NOTE:
        // -Ofast or -ffast-math optimisation disables NaN
        //
        if (std::isnan(rhs)) throw std::invalid_argument { "RHS operand is not a number" };

        return Duration { static_cast<Tick_type>(lhs.ticks() * rhs) };
    }
    

    template <typename Dur_Ty,
        is_duration<Dur_Ty> = true>
    constexpr Duration operator*(double lhs, const Dur_Ty& rhs)
    {
        // NOTE:
        // -Ofast or -ffast-math optimisation disables NaN
        //
        if (std::isnan(lhs)) throw std::invalid_argument { "RHS operand is not a number" };

        return Duration { static_cast<Tick_type>(lhs * rhs.ticks()) };
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr int operator/(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        if (rhs.ticks() == 0) throw std::overflow_error { "divide by zero!" };

        return static_cast<int>(lhs.ticks() / rhs.ticks());
    }


    template <typename Dur_Ty,
        is_duration<Dur_Ty> = true>
    constexpr Duration operator/(const Dur_Ty& lhs, double rhs)
    {
        if (rhs == 0.0) throw std::overflow_error { "divide by zero!" };

        return Duration { static_cast<Tick_type>(lhs.ticks() / rhs) };
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr bool operator==(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        return (lhs.ticks() == rhs.ticks());
    }


    template <typename Obs_Ty1, typename Obs_Ty2, 
        is_observation<Obs_Ty1> = true,
        is_observation<Obs_Ty2> = true>
    constexpr bool operator==(const Obs_Ty1& lhs, const Obs_Ty2& rhs)
    {
        return (lhs.since_epoch() == rhs.since_epoch());
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr bool operator!=(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        return (lhs.ticks() != rhs.ticks());
    }


    template <typename Obs_Ty1, typename Obs_Ty2, 
        is_observation<Obs_Ty1> = true,
        is_observation<Obs_Ty2> = true>
    constexpr bool operator!=(const Obs_Ty1& lhs, const Obs_Ty2& rhs)
    {
        return (lhs.since_epoch() != rhs.since_epoch());
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr bool operator>(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        return (lhs.ticks() > rhs.ticks());
    }


    template <typename Obs_Ty1, typename Obs_Ty2, 
        is_observation<Obs_Ty1> = true,
        is_observation<Obs_Ty2> = true>
    constexpr bool operator>(const Obs_Ty1& lhs, const Obs_Ty2& rhs)
    {
        return (lhs.since_epoch() > rhs.since_epoch());
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr bool operator<(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        return (lhs.ticks() < rhs.ticks());
    }


    template <typename Obs_Ty1, typename Obs_Ty2, 
        is_observation<Obs_Ty1> = true,
        is_observation<Obs_Ty2> = true>
    constexpr bool operator<(const Obs_Ty1& lhs, const Obs_Ty2& rhs)
    {
        return (lhs.since_epoch() < rhs.since_epoch());
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr bool operator>=(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        return (lhs.ticks() >= rhs.ticks());
    }


    template <typename Obs_Ty1, typename Obs_Ty2, 
        is_observation<Obs_Ty1> = true,
        is_observation<Obs_Ty2> = true>
    constexpr bool operator>=(const Obs_Ty1& lhs, const Obs_Ty2& rhs)
    {
        return (lhs.since_epoch() >= rhs.since_epoch());
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr bool operator<=(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        return (lhs.ticks() <= rhs.ticks());
    }


    template <typename Obs_Ty1, typename Obs_Ty2, 
        is_observation<Obs_Ty1> = true,
        is_observation<Obs_Ty2> = true>
    constexpr bool operator<=(const Obs_Ty1& lhs, const Obs_Ty2& rhs)
    {
        return (lhs.since_epoch() <= rhs.since_epoch());
    }


    template <typename Dur_Ty1, typename Dur_Ty2, 
        is_duration<Dur_Ty1> = true, 
        is_duration<Dur_Ty2> = true>
    constexpr Duration abs_diff(const Dur_Ty1& lhs, const Dur_Ty2& rhs)
    {
        using std::abs;

        auto lhs_ticks = lhs.ticks();
        auto rhs_ticks = rhs.ticks();

        if (lhs_ticks >= rhs_ticks) return Duration { abs(lhs_ticks - rhs_ticks) };
        else                        return Duration { abs(rhs_ticks - lhs_ticks) };
    }


    template <typename Obs_Ty, typename Dur_Ty, 
        is_observation<Obs_Ty> = true,
        is_duration<Dur_Ty>    = true>
    constexpr Duration abs_diff(const Obs_Ty& lhs, const Dur_Ty& rhs)
    {
        using std::abs;

        std::uint64_t lhs_ticks = lhs.since_epoch().ticks();
        std::uint64_t rhs_ticks = rhs.ticks();

        if (lhs_ticks >= rhs_ticks) return Duration { abs(static_cast<Tick_type>(lhs_ticks - rhs_ticks)) };
        else                        return Duration { abs(static_cast<Tick_type>(rhs_ticks - lhs_ticks)) };
    }


    template <typename Obs_Ty1, typename Obs_Ty2, 
        is_observation<Obs_Ty1> = true,
        is_observation<Obs_Ty2> = true>
    constexpr Duration abs_diff(const Obs_Ty1& lhs, const Obs_Ty2& rhs)
    {
        using std::abs;

        std::uint64_t lhs_ticks = lhs.since_epoch().ticks();
        std::uint64_t rhs_ticks = rhs.since_epoch().ticks();

        if (lhs_ticks >= rhs_ticks) return Duration { abs(static_cast<Tick_type>(lhs_ticks - rhs_ticks)) };
        else                        return Duration { abs(static_cast<Tick_type>(rhs_ticks - lhs_ticks)) };
    }
    
} // namespace Navtech::Time

#endif // TIME_UTILS_H