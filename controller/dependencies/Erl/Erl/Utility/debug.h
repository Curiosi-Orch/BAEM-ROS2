/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_DEBUG_H
#define ERL_DEBUG_H

#include <locale>
#include <cassert>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <stdio.h>


#include "Erl/Utility/util.h"
#include "Erl/Realtime/spinlock.h"
#include "Erl/Realtime/timers.h"
#include "Erl/Utility/singleton.h"
#include "Erl/Utility/static.h"


#define ERL_DEBUG_COUT_FILE_SWITCH(DBG_L,N) namespace Erl { namespace details { template<> struct debug_cout_switch<DBG_L,N>{ static const bool m_print_2_file = true; };} }
#ifdef ERL_DEBUG_OFF
    #ifndef ERL_DEBUG
        #define ERL_DEBUG(DBG_MSG)
    #endif
#else
    #ifndef ERL_DEBUG
        #define ERL_DEBUG(DBG_MSG) DBG_MSG
    #endif
#endif
#ifndef ERL_DEBUG_COLOR
    #if (defined(_WIN32) || defined(_WIN64) || defined(_MSC_VER) ) && !defined(ERL_DEBUG_COLOR_FORCE)
        #define ERL_DEBUG_COLOR ERL_FALSE
    #else
        #define ERL_DEBUG_COLOR ERL_TRUE
    #endif
#endif

namespace Erl
{
enum class debug_level
{
    FATAL_   =  -4,
    ERROR_   =  -2,
    WARNING_ =  -1,
    DEFAULT_ =   0,
    GOOD_    =   1,
    NONE_    = 100
};

namespace details
{
    namespace colors
    {
        #if ERL_DEBUG_COLOR==ERL_TRUE
        static constexpr const char * const reset         = "\033[0m"     ;
        static constexpr const char * const green         = "\033[0;32m"  ;
        static constexpr const char * const yellow        = "\033[0;33m"  ;
        static constexpr const char * const red           = "\033[0;31m"  ;
        static constexpr const char * const red_underline = "\033[1;4;31m";
        #else
        static constexpr const char * const reset         = "";
        static constexpr const char * const green         = "";
        static constexpr const char * const yellow        = "";
        static constexpr const char * const red           = "";
        static constexpr const char * const red_underline = "";
        #endif

    }
template <int N, class SC= std::ofstream, debug_level level = debug_level::NONE_>
class debug_file_t
{
    public:
        debug_file_t()
            : m_Timer(double(N))
        {

        }
        ~debug_file_t()
        {
            close();
        }
        bool open(const std::string& _filename, std::ios_base::openmode _mode = std::ios_base::out, bool _add_prefix=true)
        {

            std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
            if(m_Output.is_open())
            { m_Output.close(); }
            if(_add_prefix)
                m_Output.open(std::string("dbg")+std::to_string(N)+_filename.c_str(), _mode);
            else
                m_Output.open(_filename.c_str(), _mode);

            return m_Output.good();
        }
        bool is_open() const
        {
            std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
            return m_Output.is_open();
        }
        void close()
        {
            std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
            if(m_Output.is_open())
            { m_Output<<std::flush; m_Output.flush(); m_Output.close(); }
        }
        void setTimer(const double& _time_ms)
        {
            ERL_ASSERT(N>-1);
            Erl::static_if<(N>(-1))>()([&]{ // N==-1: Direct Printing;
                std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
                m_Timer.setTimer(_time_ms);
            });
        }
        bool writeable() const
        {
            return Erl::static_if_else<(N>(-1))>()([&]
            {
                std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
                return m_Timer.expired();

            },[&]
            {
                return true;
            });
        }
        typedef std::basic_ostream<char, std::char_traits<char> > cout_t;
        typedef cout_t& (*std_endl)(cout_t&);

        debug_file_t& operator<<(std_endl manip)
        {
            Erl::static_if_else<(N>(-1))>()([&](){
                std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
                if(m_Timer.expired())
                {
                   m_Timer.incrementDeadline();
                   manip(m_Output);
                }
            }, // N==-1
                [&](){
                std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
                manip(m_Output);
            });
            return (*this);
        }

        template<class IP>
        debug_file_t& operator << (const IP & _ip)
        {
            Erl::static_if_else<(N>(-1))>()([&]
            {
                std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
                if(m_Timer.expired())
                {
                   m_Timer.incrementDeadline();
                   m_Output<<_ip;
                }
            },[&]
            {
                std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
                m_Output<<_ip;
            });
            return (*this);
        }
        SC& handle()
        {
            return m_Output;
        }

    private:
        mutable Erl::RT::Spinlock m_mx;
        SC                        m_Output;
        Erl::Timer_ms             m_Timer ;
};

struct debug_mx_debug
{
    Erl::RT::Spinlock m_mx_debug;
    std::stringstream m_debug_classes_used;
};
template <debug_level _l=debug_level::DEFAULT_, int N=0>
struct debug_cout_switch
{
    static const bool m_print_2_file = false;
};

template <debug_level _l=debug_level::DEFAULT_, int N=0>
class debug_cout_t
{
public:
    debug_cout_t()
        : m_Timer(double(N))
    {
        // Stream Available Strings
        Erl::RT::Spinlock & c_mx_dbg
        = Erl::Singleton<debug_mx_debug>::getInstance().m_mx_debug;
        std::lock_guard<Erl::RT::Spinlock> c_lockT(c_mx_dbg);
        std::stringstream & c_dbg_classes_used
        = Erl::Singleton<debug_mx_debug>::getInstance().m_debug_classes_used;

        std::string c_Type(std::to_string(int(_l)));
        Erl::static_if<_l==debug_level::GOOD_>()([&]{
          c_Type = "GOOD";
        });
        Erl::static_if<_l==debug_level::DEFAULT_>()([&]{
          c_Type ="DEFAULT";
        });
        Erl::static_if<_l==debug_level::WARNING_>()([&]{
          c_Type ="WARNING";
        });
        Erl::static_if<_l==debug_level::ERROR_>()([&]{
          c_Type ="ERROR";
        });
        Erl::static_if<_l==debug_level::FATAL_>()([&]{
          c_Type ="FATAL";
        });
        c_dbg_classes_used<<"Type: debug_level::"<<c_Type;
        c_dbg_classes_used<<", N: "<<N<<std::endl;

        //

        Erl::static_if<debug_cout_switch<_l,N>::m_print_2_file>()([&]
        {
            std::stringstream c_filename;
            c_filename<<"DBG_"<<c_Type<<"_N_"<<std::to_string(N)<<".dbg";
            Erl::Singleton<debug_file_t<N,std::ofstream,_l> > ::getInstance().open(c_filename.str(),std::ios_base::out,false);

            Erl::static_if<(N>(-1))>()([&]
            {
                Erl::Singleton<debug_file_t<N,std::ofstream,_l> > ::getInstance().setTimer(-1.);
            });
        });


    }
    void setTimer(const double& _time_ms)
    {
        ERL_ASSERT(N!=-1);
        Erl::static_if<N!=(-1)>()([&]{ // N==-1: Direct Printing;
            std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
            m_Timer.setTimer(_time_ms);
        });
    }
    bool writeable() const
    {
        return Erl::static_if_else<N!=(-1)>()([&]
        {
            std::lock_guard<Erl::RT::Spinlock> c_lock(m_mx);
            return m_Timer.expired();

        },[&]
        {
            return true;
        });
    }
    void flush()
    {
        Erl::RT::Spinlock & c_mx_dbg = Erl::Singleton<debug_mx_debug>::getInstance().m_mx_debug;
        std::lock(m_mx,c_mx_dbg);
        std::lock_guard<Erl::RT::Spinlock> c_lockT(m_mx    ,std::adopt_lock);
        std::lock_guard<Erl::RT::Spinlock> c_lockG(c_mx_dbg,std::adopt_lock);
        std::cout<<std::flush;
    }
    typedef std::basic_ostream<char, std::char_traits<char> > cout_t;
    typedef cout_t& (*std_endl)(cout_t&);

    debug_cout_t& operator<<(std_endl manip)
    {
        Erl::static_if_else<!debug_cout_switch<_l,N>::m_print_2_file>()([&]()
        {
        Erl::static_if_else<N!=(-1)>()([&](){
            Erl::RT::Spinlock & c_mx_dbg = Erl::Singleton<debug_mx_debug>::getInstance().m_mx_debug;
            std::lock(m_mx,c_mx_dbg);
            std::lock_guard<Erl::RT::Spinlock> c_lockT(m_mx    ,std::adopt_lock);
            std::lock_guard<Erl::RT::Spinlock> c_lockG(c_mx_dbg,std::adopt_lock);
            if(m_Timer.expired())
            {
               m_Timer.incrementDeadline();
               manip(std::cout);
            }
        }, // N==-1
            [&](){
            Erl::RT::Spinlock & c_mx_dbg = Erl::Singleton<debug_mx_debug>::getInstance().m_mx_debug;
            std::lock_guard<Erl::RT::Spinlock> c_lockG(c_mx_dbg);
            manip(std::cout);
        });},
        [&]()
        {
            Erl::Singleton<debug_file_t<N,std::ofstream,_l> > ::getInstance()<<std::endl;
        });
        return (*this);
    }


    template<class IP>
    debug_cout_t& operator << (const IP & _ip)
    {
        Erl::static_if_else<!debug_cout_switch<_l,N>::m_print_2_file>()([&]()
        {
        Erl::static_if_else<N!=(-1)>()([&](){
            Erl::RT::Spinlock & c_mx_dbg = Erl::Singleton<debug_mx_debug>::getInstance().m_mx_debug;
            std::lock(m_mx,c_mx_dbg);
            std::lock_guard<Erl::RT::Spinlock> c_lockT(m_mx    ,std::adopt_lock);
            std::lock_guard<Erl::RT::Spinlock> c_lockG(c_mx_dbg,std::adopt_lock);
            if(m_Timer.expired())
            {
               m_Timer.incrementDeadline();
               Erl::static_if<_l==debug_level::GOOD_>()([&_ip]{
                   std::cout<<colors::green<<_ip<<colors::reset;
               });
               Erl::static_if<_l==debug_level::DEFAULT_>()([&_ip]{
                   std::cout<<_ip;
               });
               Erl::static_if<_l==debug_level::WARNING_>()([&_ip]{
                   std::cout<<colors::yellow<<_ip<<colors::reset;
               });
               Erl::static_if<_l==debug_level::ERROR_>()([&_ip]{
                   std::cerr<<colors::red<<_ip<<colors::reset;
               });
               Erl::static_if<_l==debug_level::FATAL_>()([&_ip]{
                   std::cerr<<colors::red_underline<<_ip<<colors::reset;
               });
            }
        }, // N==-1
            [&](){
            Erl::RT::Spinlock & c_mx_dbg = Erl::Singleton<debug_mx_debug>::getInstance().m_mx_debug;
            std::lock_guard<Erl::RT::Spinlock> c_lockG(c_mx_dbg);
           Erl::static_if<_l==debug_level::GOOD_>()([&_ip]{
               std::cout<<colors::green<<_ip<<colors::reset;
           });
           Erl::static_if<_l==debug_level::DEFAULT_>()([&_ip]{
               std::cout<<_ip;
           });
           Erl::static_if<_l==debug_level::WARNING_>()([&_ip]{
               std::cout<<colors::yellow<<_ip<<colors::reset;
           });
           Erl::static_if<_l==debug_level::ERROR_>()([&_ip]{
               std::cout<<colors::red<<_ip<<colors::reset;
           });
           Erl::static_if<_l==debug_level::FATAL_>()([&_ip]{
               std::cout<<colors::red_underline<<_ip<<colors::reset;
           });
        });},
        [&]()
        {
            Erl::Singleton<debug_file_t<N,std::ofstream,_l> > ::getInstance()<<_ip;
        });
        return (*this);
    }
    static Erl::RT::Spinlock& mx_debug()
    {
        return Erl::Singleton<debug_mx_debug>::getInstance().m_mx_debug;
    }
    private:
        mutable Erl::RT::Spinlock m_mx    ;
        Erl::Timer_ms             m_Timer ;

};
}


template <debug_level _l=debug_level::DEFAULT_, int N=0>
struct debug_cout_t
{
    details::debug_cout_t<_l,N>&
    operator()()
    {return Erl::Singleton<details::debug_cout_t<_l,N> >::getInstance(); }
    template<class IP>
    details::debug_cout_t<_l,N>& operator << (const IP & _ip)
    {
        return (Erl::Singleton<details::debug_cout_t<_l,N> >::getInstance()<<_ip);
    }
    static inline Erl::RT::Spinlock& mx_debug()
    {
        return details::debug_cout_t<_l,N>::mx_debug();
    }
};
template <int N=0>
struct debug_cout_t_d
{
    details::debug_cout_t<debug_level::DEFAULT_,N>&
    operator()()
    {return Erl::Singleton<details::debug_cout_t<debug_level::DEFAULT_,N> >::getInstance(); }
    template<class IP>
    details::debug_cout_t<debug_level::DEFAULT_,N>& operator << (const IP & _ip)
    {
        return (Erl::Singleton<details::debug_cout_t<debug_level::DEFAULT_,N> >::getInstance()<<_ip);
    }
};
template <int N=0>
struct debug_cout_t_g
{
    details::debug_cout_t<debug_level::GOOD_,N>&
    operator()()
    {return Erl::Singleton<details::debug_cout_t<debug_level::GOOD_,N> >::getInstance(); }
    template<class IP>
    details::debug_cout_t<debug_level::GOOD_,N>& operator << (const IP & _ip)
    {
        return (Erl::Singleton<details::debug_cout_t<debug_level::GOOD_,N> >::getInstance()<<_ip);
    }
};
template <int N=0>
struct debug_cout_t_w
{
    details::debug_cout_t<debug_level::WARNING_,N>&
    operator()()
    {return Erl::Singleton<details::debug_cout_t<debug_level::WARNING_,N> >::getInstance(); }
    template<class IP>
    details::debug_cout_t<debug_level::WARNING_,N>& operator << (const IP & _ip)
    {
        return (Erl::Singleton<details::debug_cout_t<debug_level::WARNING_,N> >::getInstance()<<_ip);
    }
};
template <int N=0>
struct debug_cout_t_e
{
    details::debug_cout_t<debug_level::ERROR_,N>&
    operator()()
    {return Erl::Singleton<details::debug_cout_t<debug_level::ERROR_,N> >::getInstance(); }
    template<class IP>
    details::debug_cout_t<debug_level::ERROR_,N>& operator << (const IP & _ip)
    {
        return (Erl::Singleton<details::debug_cout_t<debug_level::ERROR_,N> >::getInstance()<<_ip);
    }
};
template <int N=0>
struct debug_cout_t_f
{
    details::debug_cout_t<debug_level::FATAL_,N>&
    operator()()
    {return Erl::Singleton<details::debug_cout_t<debug_level::FATAL_,N> >::getInstance(); }
    template<class IP>
    details::debug_cout_t<debug_level::FATAL_,N>& operator << (const IP & _ip)
    {
        return (Erl::Singleton<details::debug_cout_t<debug_level::FATAL_,N> >::getInstance()<<_ip);
    }
};

template <int N, class SC= std::ofstream>
struct debug_file_t
{
    details::debug_file_t<N,SC>&
    operator()()
    {return Erl::Singleton<details::debug_file_t<N,SC> >::getInstance(); }
    template<class IP>
    details::debug_file_t<N,SC>& operator << (const IP & _ip)
    {
        return (Erl::Singleton<details::debug_file_t<N,SC> >::getInstance()<<_ip);
    }
};

template<class io>
void debug_set_io_format(io& _ios, int _precision, int _width, const std::ios_base::fmtflags& _format , bool _sign)
{
    _ios.precision(_precision);
    _ios<< std::setw(_width);
    _ios.setf(_format); // std::ios_base::fixed, std::ios_base::scientific, std::ios_base::hexfloat, std::ios_base::defaultfloat
    if(_sign){_ios.setf  (std::ios_base::showpos);}
    else     {_ios.unsetf(std::ios_base::showpos);}
}
template<class io>
void debug_set_io_format(io& _ios, int _precision, int _width, const std::ios_base::fmtflags & _format )
{
    _ios.precision(_precision);
    _ios<< std::setw(_width);
    _ios.setf(_format); // std::ios_base::fixed, std::ios_base::scientific, std::ios_base::hexfloat, std::ios_base::defaultfloat
}
template<class io>
void debug_set_io_format(io& _ios, int _precision, int _width)
{
    _ios.precision(_precision);
    _ios<< std::setw(_width);
}
template<class io>
void debug_set_io_format(io& _ios, int _precision)
{
    _ios.precision(_precision);
}

typedef debug_cout_t<debug_level::DEFAULT_ ,-1> debug_cout  ;
typedef debug_cout_t<debug_level::GOOD_    ,-1> debug_cout_g;
typedef debug_cout_t<debug_level::WARNING_ ,-1> debug_cout_w;
typedef debug_cout_t<debug_level::ERROR_   ,-1> debug_cout_e;
typedef debug_cout_t<debug_level::FATAL_   ,-1> debug_cout_f;

typedef debug_file_t<-1>                       debug_file;

}
#endif // ERL_DEBUG_H
