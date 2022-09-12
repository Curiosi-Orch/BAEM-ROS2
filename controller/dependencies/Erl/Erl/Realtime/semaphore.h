/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_SEMAPHORE_H
#define ERL_SEMAPHORE_H

#include <mutex>
#include <thread>
#include <condition_variable>
#include "Erl/_forwards.h"
#include "Erl/Realtime/spinlock.h"

namespace Erl
{
// *** BASE CLASSES
template<class mutex_type, class cv_type>
class SemaphoreBase
{
public:
    inline explicit SemaphoreBase(size_t n = 0): m_iter{n}{}
    SemaphoreBase(const SemaphoreBase&) = delete;
    SemaphoreBase& operator=(const SemaphoreBase&) = delete;
    inline void notify_add(const size_t& n)
    {
         ERL_ASSERT(n>0 && "n has to be larger zero otherwise undefined behaviour");
         std::lock_guard<mutex_type > lock{m_mx};
         m_iter += n;
         m_cv.notify_one();
    }
    inline void notify_set(const size_t& n)
    {
         ERL_ASSERT(n>0 && "n has to be larger zero otherwise undefined behaviour");
         std::lock_guard<mutex_type> lock{m_mx};
         m_iter = n;
         m_cv.notify_one();
    }
    inline void notify_one()
    {    std::lock_guard<mutex_type> lock{m_mx};
         m_iter = 1;
         m_cv.notify_one();
    }
    inline void notify()
    {    std::lock_guard<mutex_type> lock{m_mx};
         ++m_iter;
         m_cv.notify_one();
    }
    inline void wait(const size_t& n)
    {
        for(size_t i(0);i<n;i++)
            wait();
    }
    inline void wait()
    {
        std::unique_lock<mutex_type> lock{m_mx};
        m_cv.wait(lock, [&]{ return m_iter > 0; });
        --m_iter;
    }
    template<class Rep, class Period>
    inline bool wait_for(const std::chrono::duration<Rep, Period>& d)
    {
        std::unique_lock<mutex_type> lock{m_mx};
        auto finished = m_cv.wait_for(lock, d, [&]{ return m_iter > 0; });
        if (finished)
            --m_iter;
        return finished;
    }
    template<class Clock, class Duration>
    inline bool wait_until(const std::chrono::time_point<Clock, Duration>& t)
    {
        std::unique_lock<mutex_type> lock{m_mx};
        auto finished = m_cv.wait_until(lock, t, [&]{ return m_iter > 0; });
        if (finished)
            --m_iter;
        return finished;
    }
protected:
    size_t     m_iter;
    mutex_type m_mx;
    cv_type    m_cv;
};
template<class mutex_type, class cv_type>
class SemaphoreBinaryBase
{
public:
    inline explicit SemaphoreBinaryBase(bool _state = false): m_state{_state}{}
    SemaphoreBinaryBase(const SemaphoreBinaryBase&) = delete;
    SemaphoreBinaryBase& operator=(const SemaphoreBinaryBase&) = delete;
    inline void notify()
    {    std::lock_guard<mutex_type> lock{m_mx};
         m_state = true;
         m_cv.notify_one();
    }
    inline void wait()
    {
        std::unique_lock<mutex_type > lock{m_mx};
        m_cv.wait(lock, [&]{ return m_state; });
        m_state = false;
    }
    template<class Rep, class Period>
    inline bool wait_for(const std::chrono::duration<Rep, Period>& d)
    {
        std::unique_lock<mutex_type> lock{m_mx};
        auto finished = m_cv.wait_for(lock, d, [&]{ return m_state; });
        if (finished)
            m_state = false;
        return finished;
    }
    template<class Clock, class Duration>
    inline bool wait_until(const std::chrono::time_point<Clock, Duration>& t)
    {
        std::unique_lock<mutex_type> lock{m_mx};
        auto finished = m_cv.wait_until(lock, t, [&]{ return m_state; });
        if (finished)
            m_state=false;
        return finished;
    }
protected:
    bool        m_state;
    mutex_type  m_mx;
    cv_type     m_cv;
};
// *** SPECIALISATIONS
class Semaphore : public SemaphoreBase<std::mutex,std::condition_variable>
{
    using native_handle_type = std::condition_variable::native_handle_type;
public:
    inline explicit Semaphore(bool n = false): SemaphoreBase(n){}
    inline native_handle_type native_handle()
    { return m_cv.native_handle(); }
};
class SemaphoreBinary : public SemaphoreBinaryBase<std::mutex,std::condition_variable>
{
    using native_handle_type = std::condition_variable::native_handle_type;
public:
    inline explicit SemaphoreBinary(bool n = false): SemaphoreBinaryBase(n){}
    inline native_handle_type native_handle()
    { return m_cv.native_handle(); }
};

typedef SemaphoreBase      <RT::Spinlock,std::condition_variable_any> SemaphoreSpin       ;
typedef SemaphoreBinaryBase<RT::Spinlock,std::condition_variable_any> SemaphoreBinarySpin ;
typedef SemaphoreBase      <std::mutex  ,std::condition_variable_any> SemaphoreMutex      ;
typedef SemaphoreBinaryBase<std::mutex  ,std::condition_variable_any> SemaphoreBinaryMutex;

}

#endif // ERL_SEMAPHORE_H

