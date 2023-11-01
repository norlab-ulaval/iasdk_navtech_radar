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

#include <sstream>
#include <iomanip>
#include <thread>

#include <iostream> // FOR DEBUG ONLY

#include "Time_utils.h"

namespace Navtech::Time {
    
    // ------------------------------------------------------------------------------------------------------------
    //
    Duration& Duration::operator+=(const Duration& rhs)
    {
        this->duration += rhs.duration;
        return *this;
    }

    Duration& Duration::operator-=(const Duration& rhs)
    {
        this->duration -= rhs.duration;
        return *this;
    }

    Duration&Duration:: operator*=(double rhs)
    {
        duration = Duration_type { static_cast<Tick_type>(std::round(this->duration.count() * rhs)) };
        return *this;
    }

    Duration& Duration::operator/=(double rhs)
    {
        if (rhs == 0.0) throw std::underflow_error { "Divide by zero!" };

        duration = Duration_type { static_cast<Tick_type>(std::round(this->duration.count() / rhs)) };
        return *this;
    }


    Duration& Duration::operator+=(const Thread_safe::Duration& rhs)
    {
        this->duration += rhs.to_chrono<std::chrono::nanoseconds>();
        return *this;
    }


    Duration& Duration::operator-=(const Thread_safe::Duration& rhs)
    {
        this->duration -=rhs.to_chrono<std::chrono::nanoseconds>();
        return *this;
    }


    std::string Duration::to_string() const
    {
        // The duration is displayed in the most appropriate (that is, in the most 
        // human-readable) units, given the value of the duration; so:
        // - Sub-millisecond values are displayed as whole microseconds
        // - Values greated than 1msec are displayed as (fractional) millisecond values
        // - Durations greater than 1sec are displayed as (fractional) second values
        //
        auto sub_secs = [this]       { return (ticks() - ((ticks() / 1'000'000'000) * 1'000'000'000)); };
        auto msec = [sub_secs]       { return (sub_secs() / 1'000'000); };
        auto usec = [msec, sub_secs] { return ((sub_secs() / 1000) - (msec() * 1000)); };

        std::ostringstream os { };

        std::string  units     { "\u03BCs" };  // \u03BC => greek letter mu
        unsigned int precision { 0 };
        double       divisor   { 1'000.0 };

        if (*this > 1_msec) { 
            divisor   = 1'000'000.0; 
            units     = "ms";
            precision = (usec() != 0 ? 3 : 0);
        }

        if (*this > 1_sec)  { 
            divisor = 1'000'000'000.0; 
            units   = "s";
            if (msec() != 0) precision = 3;
            if (usec() != 0) precision = 6;
        }

        os << std::fixed;
        os << std::setprecision(precision);
        os << ticks() / divisor;
        os << units;

        return os.str();
    }

    // ================================================================================================================
    //
    namespace Monotonic {

        Observation null_time { };

        // ------------------------------------------------------------------------------------------------------------
        //
        Observation::Observation(const Duration& init) : 
            time { std::chrono::nanoseconds { init.ticks() } }
        {
        }


        Duration Observation::since_epoch() const
        {
            return Duration {
                static_cast<Tick_type>(
                    time
                    .time_since_epoch()
                    .count()
                )
            };
        }


        std::timespec Observation::to_ntp() const
        {
            auto ticks = time.time_since_epoch().count();
            auto seconds     = static_cast<std::time_t>(ticks / 1'000'000'000);
            auto sub_seconds = static_cast<long>(ticks - (seconds * 1'000'000'000));

            return std::timespec { seconds, sub_seconds };
        }


        Real_time::Observation Observation::to_real_time() const
        {
            return Real_time::Observation { Monotonic::Clock::started_at() + since_epoch() };
        }


        Observation& Observation::operator+=(const Duration& d)
        {
            std::chrono::nanoseconds offset { d.ticks() };
            time += offset;
            return *this;
        }


        Observation& Observation::operator-=(const Duration& d)
        {
            std::chrono::nanoseconds offset { d.ticks() };
            time -= offset;
            return *this;
        }


        Duration abs_diff(const Observation& lhs, const Observation& rhs)
        {
            if (lhs >= rhs) return Duration { lhs.since_epoch() - rhs.since_epoch() };
            else            return Duration { rhs.since_epoch() - lhs.since_epoch() };
        }


        // ------------------------------------------------------------------------------------------------------------
        //
        Observation Clock::now()
        {
            return Observation { 
                Duration {
                    static_cast<Tick_type>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()
                        )
                        .count()
                    )
                }
            };
        }


        Real_time::Observation Clock::started_at()
        {
            auto t_real = Real_time::Clock::now();
            auto t_mono = Monotonic::Clock::now();

            return Real_time::Observation { t_real - t_mono };
        }


        Observation now()
        {
            return Clock::now();            
        }


        // ------------------------------------------------------------------------------------------------------------
        //
        void sleep_for(const Duration& sleep_period)
        {
            std::chrono::nanoseconds duration { sleep_period.ticks() };
            std::this_thread::sleep_for(duration);
        }


        void sleep_until(const Observation& wakeup_time)
        {
            std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> time { 
                std::chrono::nanoseconds { wakeup_time.since_epoch().ticks() }
            };

            std::this_thread::sleep_until(time);
        }


        void sleep_for(const Thread_safe::Duration& sleep_period)
        {
            std::chrono::nanoseconds duration { sleep_period.ticks() };
            std::this_thread::sleep_for(duration);
        }


        void sleep_until(const Thread_safe::Monotonic::Observation& wakeup_time)
        {
            std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> time { 
                std::chrono::nanoseconds { wakeup_time.since_epoch().ticks() }
            };

            std::this_thread::sleep_until(time);
        }

    } // namespace Monotonic


    // ================================================================================================================
    //
    namespace Real_time {

        Observation null_time { };

        // ------------------------------------------------------------------------------------------------------------
        //
        Observation::Observation(const Duration& init) : 
            time { std::chrono::nanoseconds { init.ticks() } }
        {
        }


        Observation::Observation(const std::time_t& init) : 
            time { 
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::from_time_t(init).time_since_epoch()
                )
            }
        {
        }


        Observation::Observation(const std::timespec& init) : 
            time {
                std::chrono::nanoseconds { (init.tv_sec * 1'000'000'000) + init.tv_nsec }
            }
        {
        }


        Duration Observation::since_epoch() const
        {
            return Duration {
                static_cast<Tick_type>(
                    time
                    .time_since_epoch()
                    .count()
                )
            };
        }


        int Observation::year() const
        {
            auto time = to_time_t();
            auto tm = std::gmtime(&time);

            return tm->tm_year + 1900;
        }


        int Observation::month() const
        {
            auto time = to_time_t();
            auto tm = std::gmtime(&time);

            return tm->tm_mon + 1;
        }


        int Observation::day() const
        {
            auto time = to_time_t();
            auto tm = std::gmtime(&time);

            return tm->tm_mday;
        }


        int Observation::hour() const
        {
            auto time = to_time_t();
            auto tm = std::gmtime(&time);

            return tm->tm_hour;
        }


        int Observation::minute() const
        {
            auto time = to_time_t();
            auto tm = std::gmtime(&time);

            return tm->tm_min;
        }


        int Observation::second() const
        {
            auto time = to_time_t();
            auto tm = std::gmtime(&time);

            return tm->tm_sec;
        }


        Duration Observation::get_subseconds() const
        {
            using std::chrono::time_point;
            using std::chrono::duration;
            using std::ratio_multiply;
            using std::ratio;
            using std::chrono::hours;
            using std::chrono::minutes;
            using std::chrono::seconds;
            using std::chrono::duration_cast;
            using days = duration<int, ratio_multiply<hours::period, ratio<24>>::type>;

            auto since_epoch = time.time_since_epoch();

            days d = duration_cast<days>(since_epoch);
            since_epoch -= d;
            hours h = duration_cast<hours>(since_epoch);
            since_epoch -= h;
            minutes m = duration_cast<minutes>(since_epoch);
            since_epoch -= m;
            seconds s = duration_cast<seconds>(since_epoch);
            since_epoch -= s;

            return Duration { static_cast<Tick_type>(since_epoch.count()) };
        }


        int Observation::milliseconds() const
        {
            return get_subseconds().ticks() / 1'000'000;
        }


        int Observation::microseconds() const
        {
            return (get_subseconds().ticks() / 1000 - (milliseconds() * 1000));
        }


        Observation& Observation::format_as(std::string_view fmt)
        {
            format = fmt;
            return *this;
        }


        std::string Observation::to_string() const
        {
            using std::chrono::system_clock;
            using std::ostringstream;
            using std::string_view;
            using std::string;
            using std::time_t;
            using std::put_time;
            using std::gmtime;
            using std::fixed;
            using std::setprecision;
            using std::setfill;
            using std::setw;
            using std::round;

            ostringstream os { };
            const time_t c_time  { system_clock::to_time_t(time) };
            string_view msec_fmt { R"(%ms)" };
            string_view usec_fmt { R"(%us)" };
            string output_format { format };
            string postscript    { };

            // Parse and remove special markers for msec and usec.
            // It is expected the %ms/&us markers will be the last
            // formatters in the string.
            // If there are any 'postscript' characters after the %ms/%us
            // formatter strip them out to a temporary holder, to be added at
            // the end of the output string.
            //
            bool show_msec { false };
            bool show_usec { false };

            if (auto pos = output_format.find(msec_fmt); pos != string::npos) {
                auto postscript_sz = output_format.size() - (pos + msec_fmt.length());
                
                postscript = output_format.substr(
                    pos + msec_fmt.length(), 
                    postscript_sz
                );
                
                output_format.replace(
                    pos, 
                    msec_fmt.length() + postscript_sz, 
                    ""
                );
                show_msec = true;
            }
    
            if (auto pos = output_format.find(usec_fmt); pos != string::npos) {
                auto postscript_sz = output_format.size() - (pos + usec_fmt.length());
                
                postscript = output_format.substr(
                    pos + usec_fmt.length(), 
                    postscript_sz
                );
                
                output_format.replace(
                    pos, 
                    usec_fmt.length() + postscript_sz, 
                    ""
                );
                show_usec = true;
            }

            // Output the 'normal' formatted string
            //
            os << put_time(gmtime(&c_time), output_format.c_str());

            // Add milliseconds, or nanoseconds, as required
            //
            if (show_msec) {
                os << fixed << setprecision(3);
                os << setfill('0') << setw(3);
                os << static_cast<unsigned int>(round(get_subseconds().to_usec() / 1000.0f));
            }

            if (show_usec) {
                os << fixed << setprecision(6);
                os << static_cast<unsigned long>(get_subseconds().to_usec());
            }

            // Finish off with the postscript text
            //
            os << postscript;

            return os.str();
        }


        std::time_t Observation::to_time_t() const
        {
            return std::chrono::system_clock::to_time_t(time);
        }


        std::timespec Observation::to_ntp() const
        {
            return std::timespec {
                to_time_t(),
                static_cast<long>(get_subseconds().to_nsec())
            };
        }


        Monotonic::Observation Observation::to_monotonic() const
        {
            return Monotonic::Observation { this->since_epoch() };
        }


        Observation& Observation::operator+=(const Duration& d)
        {
            std::chrono::nanoseconds offset { d.ticks() };
            time += offset;
            return *this;
        }


        Observation& Observation::operator-=(const Duration& d)
        {
            std::chrono::nanoseconds offset { d.ticks() };
            time -= offset;
            return *this;
        }


        std::ostream& operator<<(std::ostream& os, const Observation& obs)
        {
            os << obs.to_string();
            return os;
        }


        Duration abs_diff(const Observation& lhs, const Observation& rhs)
        {
            if (lhs >= rhs) return Duration { lhs.since_epoch() - rhs.since_epoch() };
            else            return Duration { rhs.since_epoch() - lhs.since_epoch() };
        }


        // ------------------------------------------------------------------------------------------------------------
        //
        Observation Clock::now()
        {
            return Observation { 
                Duration {
                    static_cast<Tick_type>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch()
                        )
                        .count()
                    )
                }
            };
        }


        Observation now()
        {
            return Clock::now();            
        }

        
    } // namespace Real_time


    // ================================================================================================================
    //
    namespace Thread_safe {

        Duration::Duration(const Time::Duration& d) : Time::Duration { d }
        {
        }


        Duration::Duration(const Duration& other) : Time::Duration { other }
        {
            std::lock_guard other_lock { other.mtx };
            std::lock_guard this_lock  { this->mtx };

            this->duration = other.duration;
        }


        Duration& Duration::operator=(Duration rhs)
        {
            swap(*this, rhs);
            return *this;
        }


        void swap(Duration& lhs, Duration& rhs)
        {
            using std::swap;
            swap(lhs.duration, rhs.duration);
        }


        Tick_type Duration::ticks() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::ticks();
        }


        float Duration::to_nsec() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::to_nsec();
        }


        float Duration::to_usec() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::to_usec();
        }


        float Duration::to_msec() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::to_msec();
        }


        float Duration::to_sec() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::to_sec();
        }

        std::int64_t Duration::nanoseconds() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::nanoseconds();
        }


        std::int64_t Duration::microseconds() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::microseconds();
        }


        std::int64_t Duration::milliseconds() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::milliseconds();
        }


        std::int64_t Duration::seconds() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::seconds();
        }


        Duration Duration::to_nearest_millisecond() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::to_nearest_millisecond();
        }


        Duration Duration::to_nearest_second() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::to_nearest_second();
        }


        std::string Duration::to_string() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration::to_string();
        }


        Time::Duration Duration::unlocked() const
        {
            std::lock_guard lock { mtx };
            return Time::Duration { Time::Duration::ticks() };
        }


        Duration& Duration::operator+=(const Duration& rhs)
        {
            std::lock_guard lock { mtx };
            Time::Duration::operator+=(rhs);
            return *this;
        }


        Duration& Duration::operator-=(const Duration& rhs)
        {
            std::lock_guard lock { mtx };
            Time::Duration::operator-=(rhs);
            return *this;
        }
        

        Duration& Duration::operator*=(double rhs)
        {
            std::lock_guard lock { mtx };
            Time::Duration::operator*=(rhs);
            return *this;
        }
        

        Duration& Duration::operator/=(double rhs)
        {
            std::lock_guard lock { mtx };
            Time::Duration::operator/=(rhs);
            return *this;
        }


        // ================================================================================================================
        //
        namespace Monotonic {

            Observation::Observation(const Time::Monotonic::Observation& init) :
                Time::Monotonic::Observation { init }
            {
            }


            Observation::Observation(const Observation& other) : 
                Time::Monotonic::Observation { other }
            {
                std::lock_guard other_lock { other.mtx };
                std::lock_guard this_lock  { this->mtx };

                this->time = other.time;
            }
            

            Observation& Observation::operator=(Observation rhs)
            {
                swap(*this, rhs);
                return *this;
            }
            
            
            void swap(Observation& lhs, Observation& rhs)
            {
                using std::swap;
                swap(lhs.time, rhs.time);
            }


            Time::Duration Observation::since_epoch() const
            {
                std::lock_guard lock { mtx };
                return Time::Monotonic::Observation::since_epoch();
            }


            std::timespec Observation::to_ntp() const
            {
                std::lock_guard lock { mtx };
                return Time::Monotonic::Observation::to_ntp();
            }


            Time::Real_time::Observation Observation::to_real_time() const
            {
                std::lock_guard lock { mtx };
                return Time::Monotonic::Observation::to_real_time();
            }


            Observation& Observation::operator+=(const Time::Duration& rhs)
            {
                std::lock_guard lock { mtx };
                Time::Monotonic::Observation::operator+=(rhs);
                return *this;
            }


            Observation& Observation::operator-=(const Time::Duration& rhs)
            {
                std::lock_guard lock { mtx };
                Time::Monotonic::Observation::operator-=(rhs);
                return *this;
            }


            Observation& Observation::operator+=(const Time::Thread_safe::Duration& rhs)
            {
                std::lock_guard lock { mtx };
                Time::Monotonic::Observation::operator+=(Time::Duration { rhs.ticks() });
                return *this;
            }


            Observation& Observation::operator-=(const Time::Thread_safe::Duration& rhs)
            {
                std::lock_guard lock { mtx };
                Time::Monotonic::Observation::operator-=(Time::Duration { rhs.ticks() });
                return *this;
            }

        } // namespace Monotonic


        // ================================================================================================================
        //
        namespace Real_time {

            // ------------------------------------------------------------------------------------------------------------
            // Thread_safe::Observation
            //
            Observation::Observation(const Time::Real_time::Observation&  other) :
                Time::Real_time::Observation { other }
            {
            }


            Observation::Observation(const Observation& other) :
                Time::Real_time::Observation { other }
            {
                std::lock_guard other_lock { other.mtx };
                std::lock_guard this_lock  { this->mtx };

                this->time   = other.time;
                this->format = other.format;
            }


            Observation& Observation::operator=(Observation rhs)
            {
                swap(*this, rhs);
                return *this;
            }


            void swap(Observation& lhs, Observation& rhs)
            {
                using std::swap;

                swap(lhs.time, rhs.time);
                swap(lhs.format, rhs.format);
            }


            Time::Duration Observation::since_epoch() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::since_epoch();
            }


            int Observation::year() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::year();
            }


            int Observation::month() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::month();
            }


            int Observation::day() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::day();
            }


            int Observation::hour() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::hour();
            }


            int Observation::minute() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::minute();
            }


            int Observation::second() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::second();
            }


            int Observation::milliseconds() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::milliseconds();
            }


            int Observation::microseconds() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::microseconds();
            }

            Observation& Observation::format_as(std::string_view fmt)
            {
                std::lock_guard lock { mtx };
                Time::Real_time::Observation::format_as(fmt);
                return *this;
            }


            std::string Observation::to_string() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::to_string();
            }


            std::time_t Observation::to_time_t() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::to_time_t();
            }


            std::timespec Observation::to_ntp() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::to_ntp();
            }


            Monotonic::Observation Observation::to_monotonic() const
            {
                std::lock_guard lock { mtx };
                return Time::Real_time::Observation::to_monotonic();
            }


            Observation& Observation::operator+=(const Time::Duration& rhs)
            {
                std::lock_guard lock { mtx };
                Time::Real_time::Observation::operator+=(rhs);
                return *this;
            }


            Observation& Observation::operator-=(const Time::Duration& rhs)
            {
                std::lock_guard lock { mtx };
                Time::Real_time::Observation::operator-=(rhs);
                return *this;
            }


            Observation& Observation::operator+=(const Time::Thread_safe::Duration& rhs)
            {
                std::lock_guard lock { mtx };
                Time::Real_time::Observation::operator+=(Time::Duration { rhs.ticks() });
                return *this;
            }


            Observation& Observation::operator-=(const Time::Thread_safe::Duration& rhs)
            {
                std::lock_guard lock { mtx };
                Time::Real_time::Observation::operator-=(Time::Duration { rhs.ticks() });
                return *this;
            }


            std::ostream& operator<<(std::ostream& os, const Observation& obs)
            {
                os << obs.to_string();
                return os;
            }

        } // namespace Real_time

    } // namespace Thread_safe

} // namespace Navtech::Time