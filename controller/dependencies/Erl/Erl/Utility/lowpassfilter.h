/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_LOWPASSFILTER_H
#define ERL_LOWPASSFILTER_H

#include "Erl/Utility/util.h"
#include "Erl/Realtime/timers.h"

namespace Erl
{
template<class T> class LowpassFilter
{
public:
    typedef typename std::decay<typename Erl::extract_value_type<T,0>::value_type>::type  BaseT;

    inline LowpassFilter()
    {}

    inline LowpassFilter(const BaseT& _CutOffFrequency, const BaseT& _SampleTime)
        : m_FilterValue()
        , m_FilterCoeff(calcFilterCoeff(_CutOffFrequency,_SampleTime))
    {}

    inline LowpassFilter(const T& _InitValue, const BaseT& _CutOffFrequency, const BaseT& _SampleTime)
        : m_FilterValue(_InitValue)
        , m_FilterCoeff(calcFilterCoeff(_CutOffFrequency,_SampleTime))
    {}

    inline LowpassFilter(const T& _InitValue)
        : m_FilterValue(_InitValue)
        , m_FilterCoeff()
    {}

    inline T add(const T& _NewValue)
    {
        m_FilterValue = (BaseT(1)-m_FilterCoeff) * m_FilterValue + m_FilterCoeff*_NewValue;
        return m_FilterValue;
    }

    inline T get() const
    {
        return m_FilterValue;
    }
    inline BaseT getFilterCoeff() const
    {
        return m_FilterCoeff;
    }

    inline void setFilterCoeff(const T& _FilterCoeff)
    {
        m_FilterCoeff = _FilterCoeff;
    }

    inline void setFilterCoeff(const BaseT& _CutOffFrequency, const BaseT& _SampleTime)
    {
        m_FilterCoeff = calcFilterCoeff(_CutOffFrequency,_SampleTime);
    }

    inline void setFilterValue(const T& _FilterValue)
    {
        m_FilterValue = _FilterValue;
    }

    inline static BaseT calcCutOffFrequency(const BaseT& _FilterCoeff    , const BaseT& _SampleTime     )
    {
        return (_FilterCoeff/((BaseT(1)-_FilterCoeff)*BaseT(2)*ERL_PI*_SampleTime));
    }

    inline static BaseT calcFilterCoeff    (const BaseT& _CutOffFrequency, const BaseT& _SampleTime     )
    {
        return (BaseT(1)/(BaseT(1) + BaseT(1)/(_CutOffFrequency*BaseT(2)*ERL_PI*_SampleTime)));
    }

    inline static BaseT calcSampleTime     (const BaseT& _FilterCoeff    , const BaseT& _CutOffFrequency)
    {
        return  (_FilterCoeff/((BaseT(1)-_FilterCoeff)*BaseT(2)*ERL_PI*_CutOffFrequency));
    }

private:
    T      m_FilterValue;
    BaseT  m_FilterCoeff  ;
};

template<class T> class LowpassFilterAuto : public LowpassFilter<T>
{
public :
    typedef typename std::decay<typename Erl::extract_value_type<T,0>::value_type>::type  BaseT;
    typedef LowpassFilter<T>                                                              BaseClass;

    inline LowpassFilterAuto()
        : BaseClass()
        , m_Timer(BaseT(0))
    {}

    inline LowpassFilterAuto(const T& _InitValue)
        : BaseClass(_InitValue)
        , m_Timer(BaseT(0))
    {}

    inline LowpassFilterAuto(const T& _InitValue, const BaseT& _CutOffFrequency)
        : BaseClass(_InitValue)
        , m_CutOffFrequency(_CutOffFrequency)
        , m_Timer(BaseT(0))
    {}

    inline T add(const T& _NewValue)
    {
        this->setFilterCoeff(m_CutOffFrequency,m_Timer.elapsedTime());
        m_Timer.resetTimer();
        return BaseClass::add(_NewValue);
    }

    inline BaseT getCutOffFrequency() const
    {
        return m_CutOffFrequency;
    }

    inline void setCutOffFrequency(const BaseT& _CutOffFrequency)
    {
        m_CutOffFrequency = _CutOffFrequency;
    }



private:
    BaseT        m_CutOffFrequency;
    Erl::Timer_s m_Timer;

};
}

#endif // LOWPASSFILTER_H
