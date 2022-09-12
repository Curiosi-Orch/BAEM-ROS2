/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_TIMERS_H
#define ERL_TIMERS_H

#include <thread>
#include <chrono>
#include "Erl/Utility/util.h"
#include "Erl/Utility/singleton.h"

namespace Erl
{

template <typename TimePrecision, typename TimeScale = std::ratio<1,1>, typename ClockType = std::chrono::high_resolution_clock>
class Spinner
{
public:

    inline Spinner(TimePrecision _spintime)
        : spintime(_spintime)
        , missedSpins(0)
        , totalCalls(0)
        , end(ClockType::now() + std::chrono::duration<TimePrecision, TimeScale>(_spintime))
    {
    }

    inline void reset(const TimePrecision& _spintime = TimePrecision(-1.0) )
    {
        end = ClockType::now()+std::chrono::duration<TimePrecision, TimeScale>(spintime);
        if (_spintime > TimePrecision(0))
        {
            end += std::chrono::duration<TimePrecision, TimeScale>(_spintime-spintime);
            spintime = _spintime;
        }
    }

    inline void reset(const std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> >& _end)
    {
        spintime  = std::chrono::duration<TimePrecision,TimeScale>(_end - ClockType::now()).count();
        end       = _end;
    }

    inline void spinUp()
    {
        typename ClockType::time_point curr = ClockType::now();
        while(curr<end)
        {
            curr = ClockType::now();
        }
        end = curr + std::chrono::duration<TimePrecision, TimeScale>(spintime);
    }

    inline bool spinUpWithInfo()
    {

        totalCalls++;
        typename ClockType::time_point curr
                = ClockType::now();

        if (curr>end)
        {
            missedSpins++;
            end = curr + std::chrono::duration<TimePrecision, TimeScale>(spintime);
            return false;
        }

        while(curr<end)
        {
            curr = ClockType::now();
        }
        end = curr + std::chrono::duration<TimePrecision, TimeScale>(spintime);
        return true;
    }

    inline unsigned int getMissedSpins()
    {
        return missedSpins;
    }

    inline unsigned int getTotalCalls()
    {
        return totalCalls;
    }

private:

    TimePrecision spintime;
    unsigned int missedSpins;
    unsigned int totalCalls;
    std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > end;

};

template <typename TimePrecision, typename TimeScale, typename ClockType = std::chrono::high_resolution_clock>
inline void spin(TimePrecision _spintime)
{
    typename std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > end
            = ClockType::now()
            + std::chrono::duration<TimePrecision, TimeScale>(_spintime);
     while(ClockType::now() < end);
}

template <typename TimePrecision, typename TimeScale, typename ClockType = std::chrono::high_resolution_clock>
inline void spin_until(const typename std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> >& _end)
{
     while(ClockType::now() < _end);
}

template <typename TimePrecision,typename TimeScale, typename ClockType = std::chrono::high_resolution_clock>
class Timer
{

public:

    inline Timer()
        : m_TP(ClockType::now())
        , m_LastDuration(0)
    {}

    inline Timer(const std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> >& _TP)
        : m_TP(_TP)
        , m_LastDuration(ClockType::now() - _TP)
    {}

    inline Timer(const TimePrecision & _duration)
        : m_TP(ClockType::now() + std::chrono::duration<TimePrecision, TimeScale>(_duration))
        , m_LastDuration(std::chrono::duration<TimePrecision, TimeScale>(_duration))
    {}

    inline Timer(const std::chrono::duration<TimePrecision, TimeScale> & _duration)
        : m_TP(ClockType::now() + _duration)
        , m_LastDuration(_duration)
    {}

    inline void setTimer(const TimePrecision & _duration)
    {
        m_LastDuration = std::chrono::duration<TimePrecision, TimeScale>(_duration);
        m_TP = ClockType::now()+m_LastDuration;
    }

    inline void setTimer(const std::chrono::duration<TimePrecision, TimeScale> & _duration)
    {
        m_TP = ClockType::now() + _duration;
        m_LastDuration = _duration;
    }

    inline void setTimer(const std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > & _tp)
    {
        m_LastDuration = _tp - m_TP;
        m_TP = _tp;
    }

    inline void resetTimer()
    {
        m_TP = ClockType::now();
    }

    inline void resetDeadline()
    {
        m_TP = ClockType::now()+m_LastDuration;
    }

    inline void incrementDeadline()
    {
        m_TP = m_TP + m_LastDuration;
    }

    inline TimePrecision remainingTime() const
    {
        return std::chrono::duration<TimePrecision, TimeScale>(m_TP-ClockType::now()).count();
    }

    inline TimePrecision elapsedRatio() const
    {
        return TimePrecision(1)-(remainingTime()/m_LastDuration.count());
    }

    inline TimePrecision elapsedBoundRatio() const
    {
        return Erl::clamp<TimePrecision>(elapsedRatio(),TimePrecision(0),TimePrecision(1));
    }

    inline TimePrecision elapsedTime() const
    {
        return  std::chrono::duration<TimePrecision, TimeScale>(ClockType::now()-m_TP+m_LastDuration).count();
    }

    inline std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > deadline() const
    {
        return m_TP;
    }

    inline std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > deadline_offset(const TimePrecision& _offset) const
    {
        return (m_TP+std::chrono::duration<TimePrecision,TimeScale> (_offset));
    }

    inline std::chrono::duration<TimePrecision,TimeScale> duration() const
    {
        return m_LastDuration;
    }

    inline bool expired() const
    {
        return m_TP < ClockType::now();
    }

    inline bool valid() const
    {
        return m_TP > ClockType::now();
    }

private:

    std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > m_TP;
    std::chrono::duration<TimePrecision,TimeScale>                                     m_LastDuration;
};

template <typename TimePrecision,typename TimeScale, typename ClockType = std::chrono::high_resolution_clock>
class TimerPeriod
{

private:

    std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > m_TPMin;
    std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > m_TPMax;

public:

    inline TimerPeriod()
        : m_TPMin(ClockType::now()),
          m_TPMax(m_TPMin)
    {}

    inline TimerPeriod(const TimePrecision & _MinDur, const TimePrecision & _MaxDur)
        : m_TPMin(ClockType::now() + std::chrono::duration<TimePrecision, TimeScale>(_MinDur)),
          m_TPMax(m_TPMin + std::chrono::duration<TimePrecision, TimeScale>(_MaxDur - _MinDur))
    {}

    inline void setPeriod(const TimePrecision & _MinDur, const TimePrecision & _MaxDur)
    {
        m_TPMin = ClockType::now() + std::chrono::duration<TimePrecision, TimeScale>(_MinDur);
        m_TPMax = m_TPMin + std::chrono::duration<TimePrecision, TimeScale>(_MaxDur - _MinDur);
    }

    inline TimePrecision timeDist() const
    {
        std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > c_TPCurr(
                                                                 ClockType::now());
        if(c_TPCurr < m_TPMin)
        {
            return std::chrono::duration<TimePrecision, TimeScale>(m_TPMin - c_TPCurr).count();
        }
        else if(c_TPCurr > m_TPMax)
        {
            return std::chrono::duration<TimePrecision, TimeScale>(m_TPMax - c_TPCurr).count();
        }

        return TimePrecision(0);
    }

    inline std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > timePointMin() const
    {
        return m_TPMin;
    }

    inline std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> > timePointMax() const
    {
        return m_TPMax;
    }

    inline bool valid() const
    {
        return ( (ClockType::now() - m_TPMin) <= (m_TPMax - m_TPMin) );
    }
};


template <typename TimePrecision,typename TimeScale, typename DefaultState, typename ClockType = std::chrono::high_resolution_clock>
class TimedState
{
    typedef typename DefaultState::value_type                                                            state_type    ;
    typedef          Timer<TimePrecision,TimeScale, ClockType>                                           timer_type    ;
    typedef          std::chrono::time_point<ClockType,std::chrono::duration<TimePrecision,TimeScale> >  timepoint_type;
    typedef          std::chrono::duration<TimePrecision, TimeScale>                                     duration_type ;

private:

    timer_type m_Timer;
    state_type m_State;

public:

    inline explicit TimedState()
        : m_Timer(),
          m_State(DefaultState::value)
    {}

    inline explicit TimedState(const TimePrecision & _duration)
        : m_Timer(_duration),
          m_State(DefaultState::value)
    {}

    inline explicit TimedState(const state_type& _state)
        : m_Timer(),
          m_State(_state)
    {}

    inline explicit TimedState(const TimePrecision & _duration, const state_type& _state)
        : m_Timer(_duration),
          m_State(_state)
    {}

    inline explicit TimedState(const timepoint_type& _TP, const state_type& _state = DefaultState::value)
        : m_Timer(_TP)
        , m_State(_state)
    {}

    inline explicit TimedState(const duration_type & _duration, const state_type& _state = DefaultState::value)
        : m_Timer(_duration)
        , m_State(_state)
    {}

    inline void set_timeout(const TimePrecision& _timeout)
    {
        m_Timer.setTimer(_timeout);
    }

    inline void set_timeout(const duration_type & _timeout)
    {
        m_Timer.setTimer(_timeout);
    }

    inline void set_state(const state_type& _state, const TimePrecision& _timeout)
    {
        m_State  = _state;
        m_Timer.setTimer(_timeout);
    }

    inline void set_state(const state_type& _state, const duration_type & _timeout)
    {
        m_State = _state;
        m_Timer.setTimer(_timeout);
    }

    inline void set_state(const state_type& _state)
    {
        m_State = _state;
        m_Timer.resetDeadline();
    }

    inline state_type get_state()
    {
        if(m_Timer.expired())
            m_State = DefaultState::value;

        return m_State;
    }

    inline const timer_type& get_timer_ref() const
    {
       return m_Timer;
    }

    inline const timer_type get_timer_ref()
    {
       return m_Timer;
    }
};
template <typename TimePrecision,typename TimeScale, typename ClockType = std::chrono::high_resolution_clock>
class GStopwatch
{
    typedef Timer<TimePrecision,TimeScale,ClockType> SWTimer;
    public:
        inline GStopwatch()
            : m_Timer(TimePrecision(0.))
        {}
        inline static const SWTimer& sRef()
        { return Erl::Singleton<GStopwatch>::getInstance().m_Timer;}
    private:
        SWTimer m_Timer;
};


template<typename TimePrecision>
inline void sleep_s (const TimePrecision &  _s) { std::this_thread::sleep_for(std::chrono::duration<TimePrecision             >( _s)); }
template<typename TimePrecision>
inline void sleep_ms(const TimePrecision & _ms) { std::this_thread::sleep_for(std::chrono::duration<TimePrecision , std::milli>(_ms)); }
template<typename TimePrecision>
inline void sleep_us(const TimePrecision & _us) { std::this_thread::sleep_for(std::chrono::duration<TimePrecision , std::micro>(_us)); }
template<typename TimePoint>
inline void sleep_until(const TimePoint & _t)   { std::this_thread::sleep_until(_t); }
template<typename Timer>
inline void sleep_untildeadline(const Timer & _t){ std::this_thread::sleep_until(_t.deadline()); }

template<typename TimePrecision>
inline void spin_s (const TimePrecision &   _s) { Erl::spin<TimePrecision            >( _s); }
template<typename TimePrecision>
inline void spin_ms (const TimePrecision & _ms) { Erl::spin<TimePrecision, std::milli>(_ms); }
template<typename TimePrecision>
inline void spin_us (const TimePrecision & _us) { Erl::spin<TimePrecision, std::micro>(_us); }
template<typename TimePrecision>
inline void spin_ns (const TimePrecision & _ns) { Erl::spin<TimePrecision, std::nano>( _ns); }

typedef Spinner<double, std::ratio<1> >     Spinner_s     ;
typedef Spinner<double, std::milli    >     Spinner_ms    ;
typedef Spinner<double, std::micro    >     Spinner_us    ;

typedef Timer<double, std::ratio<1> >       Timer_s       ;
typedef Timer<double, std::milli    >       Timer_ms      ;
typedef Timer<double, std::micro    >       Timer_us      ;

typedef GStopwatch<double, std::ratio<1> >   GStopwatch_s ;
typedef GStopwatch<double, std::milli    >   GStopwatch_ms;
typedef GStopwatch<double, std::micro    >   GStopwatch_us;

typedef TimerPeriod<double, std::ratio<1> > TimerPeriod_s ;
typedef TimerPeriod<double, std::milli    > TimerPeriod_ms;
typedef TimerPeriod<double, std::micro    > TimerPeriod_us;

typedef TimedState<double, std::ratio<1> , std::integral_constant<bool,true> >  TimedBoolT_s ;
typedef TimedState<double, std::milli    , std::integral_constant<bool,true> >  TimedBoolT_ms;
typedef TimedState<double, std::micro    , std::integral_constant<bool,true> >  TimedBoolT_us;

typedef TimedState<double, std::ratio<1> , std::integral_constant<bool,false> > TimedBoolF_s ;
typedef TimedState<double, std::milli    , std::integral_constant<bool,false> > TimedBoolF_ms;
typedef TimedState<double, std::micro    , std::integral_constant<bool,false> > TimedBoolF_us;

}

#endif
