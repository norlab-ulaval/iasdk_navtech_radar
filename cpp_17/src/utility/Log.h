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

#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <iomanip>
#include <string>
#include <string_view>

#include "Time_utils.h"
#include "pointer_types.h"

namespace Navtech::Utility {

    // -----------------------------------------------------------------------------------------
    //
    enum class Logging_level { 
        debug,      // Debug output is reverted off after an endl;
        info,       // Default ouput
        warning,    
        error 
    };

    inline std::string to_string(Logging_level lev)
    {
        static const std::string strings[] {
            "[DEBUG]  ",
            "[INFO]   ",
            "[WARNING]",
            "[ERROR]  "
        };
        return strings[static_cast<int>(lev)];
    }

    inline bool operator==(Logging_level lhs, Logging_level rhs)
    {
        return (static_cast<int>(lhs) == static_cast<int>(rhs));
    }

    inline bool operator!=(Logging_level lhs, Logging_level rhs)
    {
        return !(lhs == rhs);
    }

    inline bool operator>(Logging_level lhs, Logging_level rhs)
    {
        return (static_cast<int>(lhs) > static_cast<int>(rhs));
    }

    inline bool operator<(Logging_level lhs, Logging_level rhs)
    {
        return (static_cast<int>(lhs) < static_cast<int>(rhs));
    }

    inline bool operator>=(Logging_level lhs, Logging_level rhs)
    {
        return ((lhs == rhs) || (lhs > rhs));
    }

    inline bool operator<=(Logging_level lhs, Logging_level rhs)
    {
        return ((lhs == rhs) || (lhs < rhs));
    }

    // -----------------------------------------------------------------------------------------
    //
    template <typename Stream_Ty = std::ostream>
    class Log {
    public:
        Log() = default;
        Log(Stream_Ty& output);
        
        void set_output(Stream_Ty& output);
        void time_format(std::string_view fmt);
        void level(Logging_level lev);
        void pop_level();
        void show_only(Logging_level lev);
        void show_all();
        void min_level(Logging_level lev);

        void flush();

        template <typename T> Log& operator<<(const T& rhs);
        Log& operator<<(Logging_level lev);
        Log& operator<<(Stream_Ty& (*modifier)(Stream_Ty&));
        Log& operator<<(Log& (*modifier)(Log&));
    
    private:
        association_to<Stream_Ty> stream                    { };
        Logging_level             log_level                 { Logging_level::info };
        Logging_level             prev_level                { log_level };
        Logging_level             fxd_log_level             { log_level };
        Logging_level             min_log_level             { log_level };
        bool                      is_fixed_level_logging    { false };
        bool                      is_min_level_logging      { false };
        std::string               time_fmt                  { "%FT%T.%msZ" };
        bool                      flushed                   { true };
    };


    template <typename Stream_Ty>
    Log<Stream_Ty>::Log(Stream_Ty& output) :
        stream { associate_with(output) }
    {
    }


    template <typename Stream_Ty>
    void Log<Stream_Ty>::set_output(Stream_Ty& output)
    {
        stream = associate_with(output);
    }

    template <typename Stream_Ty>
    void Log<Stream_Ty>::time_format(std::string_view fmt)
    {
        time_fmt = fmt;
    }


    template <typename Stream_Ty>
    template <typename T>
    Log<Stream_Ty>& Log<Stream_Ty>::operator<<(const T& rhs)
    {
        using Time::Real_time::Clock;

        if (!stream)                                              return *this;
        if (is_fixed_level_logging && fxd_log_level != log_level) return *this;
        if (is_min_level_logging && log_level < min_log_level)    return *this;

        // Only output the date and logging level after a flush
        // (in most cases, this is from an endl)
        //
        if (flushed) {
            *stream << Clock::now().format_as(time_fmt).to_string() << " ";
            *stream << to_string(log_level) << " - ";
        }
        *stream << rhs;
        flushed = false;
        return *this;
    }


    template <typename Stream_Ty>
    Log<Stream_Ty>& Log<Stream_Ty>::operator<<(Logging_level lev)
    {
        level(lev);
        return *this;
    }


    template <typename Stream_Ty>
    Log<Stream_Ty>& Log<Stream_Ty>::operator<<(Stream_Ty& (*modifier)(Stream_Ty&))
    {
        if (stream) modifier(*stream);
        return *this;
    }


    template <typename Stream_Ty>
    Log<Stream_Ty>& Log<Stream_Ty>::operator<<(Log<Stream_Ty>& (*modifier)(Log<Stream_Ty>&))
    {
        return modifier(*this);
    }


    template <typename Stream_Ty>
    void Log<Stream_Ty>::level(Logging_level lev)
    {
        prev_level = log_level;
        log_level  = lev;
    }


    template <typename Stream_Ty>
    void Log<Stream_Ty>::pop_level()
    {
        log_level = prev_level;
    }


    template <typename Stream_Ty>
    void Log<Stream_Ty>::show_only(Logging_level lev)
    {
        fxd_log_level          = lev;
        is_fixed_level_logging = true;
        is_min_level_logging   = false;
    }
    

    template <typename Stream_Ty>
    void Log<Stream_Ty>::show_all()
    {
        is_fixed_level_logging = false;
        is_min_level_logging   = false;   
    }
        

    template <typename Stream_Ty>    
    void Log<Stream_Ty>::min_level(Logging_level lev)
    {
        min_log_level          = lev;
        is_min_level_logging   = true;
        is_fixed_level_logging = false;
    }   


    template <typename Stream_Ty>
    void Log<Stream_Ty>::flush()
    {
        flushed = true;

        // Debug output is usually for single-line
        // output.  Therefore the previous logging level 
        // is restored after an endl
        //
        if (log_level == Logging_level::debug) pop_level();
    }


    // -----------------------------------------------------------------------------------------
    // Log modifier functions
    //
    inline Log<std::ostream>& endl(Log<std::ostream>& log)
    {
        log << "\n";
        log.flush();
        return log;
    }


    inline Log<std::ostream>& pop_level(Log<std::ostream>& log)
    {
        log.pop_level();
        return log;
    }


    // -----------------------------------------------------------------------------------------
    // Global logging object declarations
    //
    extern Log<std::ostream> stdout_log;

} // namespace Navtech::Utility

#endif // LOG_H