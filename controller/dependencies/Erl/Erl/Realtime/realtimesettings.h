/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_REALTIMESETTINGS_H
#define ERL_REALTIMESETTINGS_H

#include <Erl/_platform/_platform.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <future>
#include <vector>
#include <cstring>
#include <cmath>

#ifdef __linux__
#include <sys/resource.h>
#include <sys/mman.h>
#include <pthread.h>
#endif

namespace Erl
{
namespace RT
{

#if defined(__linux__) || defined(__APPLE__)
enum class POLICY
{
    OTHER = SCHED_OTHER,
    FIFO  = SCHED_FIFO,
    RR    = SCHED_RR
};
#elif _WIN32
typedef enum
{
    POLICY_OTHER = 0,
    POLICY_FIFO  = 1,
    POLICY_RR    = 2
}POLICY;
#endif

enum
{
#ifdef __linux__
    PRIORITY_MAX = 99,
    PRIORITY_MIN = 0
#elif defined(__APPLE__)
    PRIORITY_MAX = 47,
    PRIORITY_MIN = 15
#endif
};

struct RTparams
{
    POLICY policy;
    int priority;
    std::vector<int> cores;

    #if defined(__linux__) || defined(__APPLE__)
    RTparams(const POLICY &_policy = POLICY::OTHER,
    #elif _WIN32
    RTparams(const POLICY &_policy = POLICY_OTHER,
    #endif
             const int &_priority = 0,
             const std::vector<int> &_cores = std::vector<int>())
        : policy(_policy)
        , priority(_priority)
        , cores(_cores)
    {}

    RTparams(const POLICY &_policy,
             const int &_priority,
             const int &core)
        : policy(_policy)
        , priority(_priority)
    {
        cores.push_back(core);
    }
};

namespace details
{

#if defined(__linux__)

inline void _setAffinity(const pthread_t &threadID, const int &core)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(core, &mask);
    int ret = pthread_setaffinity_np(threadID, sizeof(cpu_set_t), &mask);
    if (ret!=0)
    {
        #ifdef ERL_REALTIME_THROW_EXCEPTION
        throw std::runtime_error("ERROR: Could not set CPU affinity");
        #endif
        std::cerr << std::endl << "ERROR: Could not set CPU affinity" << std::endl;
    }
}

inline void _setAffinity(const pthread_t &threadID, const std::vector<int> &cores)
{
    cpu_set_t mask, temp;
    CPU_ZERO(&mask);
    CPU_ZERO(&temp);

    for (unsigned int i = 0; i < cores.size(); i++)
    {
        CPU_SET(cores[i], &temp);
        CPU_OR(&mask, &mask, &temp);
    }
    int ret = pthread_setaffinity_np(threadID, sizeof(cpu_set_t), &mask);
    if (ret!=0)
    {
        #ifdef ERL_REALTIME_THROW_EXCEPTION
        throw std::runtime_error("ERROR: Could not set CPU affinity");
        #endif
        std::cerr << std::endl << "ERROR: Could not set CPU affinity" << std::endl;
    }
}

#endif

#if defined(__linux__) || defined(__APPLE__)

inline int _setPolicy(const pthread_t &threadID, const POLICY &policy, const int &priority = 0)
{
    if (int(priority) < sched_get_priority_min(int(policy)) || int(priority) > sched_get_priority_max(int(policy)))
    {
        int min = sched_get_priority_min(int(policy));
        int max = sched_get_priority_max(int(policy));
        std::stringstream message;
        message << "ERROR: Requested priority out of bounds. Boundaries are: " << min << " and " << max;
        #ifdef ERL_REALTIME_THROW_EXCEPTION
        throw std::out_of_range(message.str());
        #endif
        std::cerr << std::endl << message.str() << std::endl;
        return EINVAL;
    }
    struct sched_param sp;
    sp.sched_priority = int(priority);
    int ret = pthread_setschedparam(threadID, int(policy), &sp);
    if (ret!=0)
    {
        #ifdef ERL_REALTIME_THROW_EXCEPTION
        throw std::runtime_error("ERROR: Could not set policy");
        #endif
        switch(ret)
        {
            case ESRCH :
                std::cerr << "ERROR: Could not set policy, No thread with the ID thread: "<<threadID<<" could be found."<<std::endl;
                break;
            case EINVAL :
                std::cerr << "ERROR: Could not set policy, policy is not a recognized policy ("<<int(policy)<<"), or param ("<<int(priority)
                          <<"_ does not make sense for the policy"<<std::endl;
                break;
            case EPERM :
                std::cerr << "ERROR: Could not set policy, The caller does not have appropriate privileges"
                             " to set the specified scheduling policy and parameters."<<std::endl;
                break;
            default:
                std::cerr << "ERROR: Could not set policy, unknown reason: "<<ret<<std::endl;
        }
    }
    return ret;
}

inline void _setPriority(const pthread_t &threadID, const int &priority)
{
    int policy;
    struct sched_param sp;
    int ret = pthread_getschedparam(threadID, &policy, &sp);
    if (ret!=0)
    {
        #ifdef ERL_REALTIME_THROW_EXCEPTION
        throw std::runtime_error("WARNING: Could not get policy to check priority boundaries" );
        #endif
        std::cerr << std::endl << "WARNING: Could not get policy to check priority boundaries" << std::endl;
    }

    if (ret == 0 && (priority < sched_get_priority_min(policy) || priority > sched_get_priority_max(policy)))
    {
        int min = sched_get_priority_min(policy);
        int max = sched_get_priority_max(policy);
        std::stringstream message;
        message << "ERROR: Requested priority out of bounds. Boundaries are: " << min << " and " << max;
        #ifdef ERL_REALTIME_THROW_EXCEPTION
        throw std::out_of_range(message.str());
        #endif
        std::cerr << std::endl << message.str() << std::endl;
        return;
    }
    sp.sched_priority = int(priority);
    ret = pthread_setschedparam(threadID, int(policy), &sp);
    if (ret!=0)
    {
        #ifdef ERL_REALTIME_THROW_EXCEPTION
        throw std::runtime_error("ERROR: Could not set policy");
        #endif
        std::cerr << std::endl << "ERROR: Could not set priority" << std::endl;
    }
}

#endif

}

#ifdef _WIN32
ERL_RMV_UNPARA_BEGIN
#endif

template <typename TheadType>
inline void setAffinity(TheadType &thread, const std::vector<int> &cpus)
{
#if defined(__linux__)
    pthread_t threadID = (pthread_t) thread.native_handle();
    details::_setAffinity(threadID, cpus);
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error("CPU affinity not implemented on this platform");
    #endif
    std::cerr << std::endl << "CPU affinity not implemented on this platform" << std::endl;
#endif
}

template <typename TheadType>
inline void setAffinity(TheadType &thread, const int &cpu)
{
#if defined(__linux__)
    pthread_t threadID = (pthread_t) thread.native_handle();
    details::_setAffinity(threadID, cpu);
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error("CPU affinity not implemented on this platform");
    #endif
    std::cerr << std::endl << "CPU affinity not implemented on this platform" << std::endl;
#endif
}

inline void setSelfAffinity(const std::vector<int> &cpus)
{
#if defined(__linux__)
    pthread_t threadID = pthread_self();
    details::_setAffinity(threadID, cpus);
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error("CPU affinity not implemented on this platform");
    #endif
    std::cerr << std::endl << "CPU affinity not implemented on this platform" << std::endl;
#endif
}

inline void setSelfAffinity(const int &cpu)
{
#if defined(__linux__)
    pthread_t threadID = pthread_self();
    details::_setAffinity(threadID, cpu);
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error("CPU affinity not implemented on this platform");
    #endif
    std::cerr << std::endl << "CPU affinity not implemented on this platform" << std::endl;
#endif
}

template <typename TheadType>
inline void setPolicy(TheadType &thread, const POLICY &policy, const int &priority = 0)
{
#if defined(__linux__) || defined(__APPLE__)
    pthread_t threadID = (pthread_t) thread.native_handle();
    details::_setPolicy(threadID, policy, priority);
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error("Scheduler policy not defined on this platform");
    #endif
    std::cerr << std::endl << "Scheduler policy not defined on this platform" << std::endl;
#endif
}

inline void setSelfPolicy(const POLICY &policy, const int &priority = 0)
{
#if defined(__linux__) || defined(__APPLE__)
    pthread_t threadID = pthread_self();
    details::_setPolicy(threadID, policy, priority);
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error("Scheduler policy not defined on this platform");
    #endif
    std::cerr << std::endl << "Scheduler policy not defined on this platform" << std::endl;
#endif
}

template <typename TheadType>
inline void setPriority(TheadType &thread, const int &priority)
{
#if defined(__linux__) || defined(__APPLE__)
    pthread_t threadID = (pthread_t) thread.native_handle();
    details::_setPriority(threadID, priority);
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error("Set priority not implemented on this platform");
    #endif
    std::cerr << std::endl << "Set priority not implemented on this platform" << std::endl;
#endif
}

inline void setSelfPriority(const int &priority)
{
#if defined(__linux__) || defined(__APPLE__)
    pthread_t threadID = pthread_self();
    details::_setPriority(threadID, priority);
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error("Set priority not implemented on this platform");
    #endif
    std::cerr << std::endl << "Set priority not implemented on this platform" << std::endl;
#endif
}

inline int getCpuCount() // there is also pthread_num_processors_np, according to gcc hardware_concurrency should use pthread_num_processors_np on linux
{
    return int(std::thread::hardware_concurrency());
}

inline std::vector<int> getFullCpuSet() //Fills a std::vector with all availables cpus; can be used to cancel out affinity
{
    std::vector<int> c_FullCPUSet(std::thread::hardware_concurrency());
    for(unsigned i(0);i<c_FullCPUSet.size();i++)
        c_FullCPUSet[i]=int(i);
    return c_FullCPUSet;
}

inline bool setStackLimit(const size_t &bytes, bool verbose = true)
{
#ifdef __linux__
    struct rlimit rlim;
    int ret = getrlimit(RLIMIT_STACK, &rlim);
    if (ret == 0)
    {
        if (rlim.rlim_cur == RLIM_INFINITY && verbose)
            std::cout << "Current stack size limit: unlimited" << std::endl;
        else if (verbose)
            std::cout << "Current stack size limit: " << rlim.rlim_cur/(std::pow<int>(2,20)) << " MB" << std::endl;

        if (rlim.rlim_max == RLIM_INFINITY && verbose)
            std::cout << "Maximum stack size limit: unlimited" << std::endl;
        else if (verbose)
            std::cout << "Maximum stack size limit: " << rlim.rlim_max/(std::pow<int>(2,20)) << " MB" << std::endl;

        if (rlim.rlim_cur > bytes)
        {
            if (verbose)
                std::cout << "Current stack limit already high enough, nothing to do." << std::endl;
            return true;
        }
        else if (rlim.rlim_max < bytes)
        {
            if (verbose)
                std::cout << "Maximum stack size is insufficient, nothing to do." << std::endl;
            return false;
        }
        else
        {
            rlimit rlim_send = rlim;
            rlim_send.rlim_cur = bytes;
            ret = setrlimit(RLIMIT_STACK, &rlim_send);
            if (ret != 0)
            {
                if (verbose)
                {
                    #ifdef ERL_REALTIME_THROW_EXCEPTION
                    throw std::runtime_error("Could not expand allowable stack: " + std::to_string(errno));
                    #endif
                    std::cerr << "Could not expand allowable stack: " << errno << std::endl;
                }
                return false;
            }
            else
            {
                if (verbose)
                    std::cout << "Expanded stack limit to " << bytes/(size_t)(std::pow<int>(2,20)) << " MB." << std::endl;
                return true;
            }
        }
    }
    else
    {
        if (verbose)
        {
            #ifdef ERL_REALTIME_THROW_EXCEPTION
            throw std::runtime_error("Could not retrieve stack data: " + std::to_string(errno));
            #endif
            std::cerr << "Could not retrieve stack data: " << errno << std::endl;
        }
        return false;
    }
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error( "setStackLimit not implemented on this platform");
    #endif
    std::cerr << std::endl << "setStackLimit not implemented on this platform" << std::endl;
    return false;
#endif
}

inline bool setMemlockLimit(const size_t &bytes, bool verbose = true)
{
#ifdef __linux__
    struct rlimit rlim;
    int ret = getrlimit(RLIMIT_MEMLOCK, &rlim);
    if (ret == 0)
    {
        if (rlim.rlim_cur == RLIM_INFINITY && verbose)
            std::cout << "Current memlock size limit: unlimited" << std::endl;
        else if (verbose)
            std::cout << "Current memlock size limit: " << rlim.rlim_cur/(std::pow<int>(2,20)) << " MB" << std::endl;

        if (rlim.rlim_max == RLIM_INFINITY && verbose)
            std::cout << "Maximum memlock size limit: unlimited" << std::endl;
        else if (verbose)
            std::cout << "Maximum memlock size limit: " << rlim.rlim_max/(std::pow<int>(2,20)) << " MB" << std::endl;

        if (rlim.rlim_cur > bytes)
        {
            if (verbose)
                std::cout << "Current memlock limit already high enough, nothing to do." << std::endl;
            return true;
        }
        else if (rlim.rlim_max < bytes)
        {
            if (verbose)
                std::cout << "Maximum memlock size is insufficient, nothing to do." << std::endl;
            return false;
        }
        else
        {
            rlimit rlim_send = rlim;
            rlim_send.rlim_cur = bytes;
            ret = setrlimit(RLIMIT_MEMLOCK, &rlim_send);
            if (ret != 0)
            {
                if (verbose)
                {
                    #ifdef ERL_REALTIME_THROW_EXCEPTION
                    throw std::runtime_error( "Could not expand allowable memlock: "+std::to_string(errno));
                    #endif
                    std::cerr << "Could not expand allowable memlock: " << errno << std::endl;
                }
                return false;
            }
            else
            {
                if (verbose)
                    std::cout << "Expanded stack limit to " << bytes/(size_t)(std::pow<int>(2,20)) << " MB." << std::endl;
                return true;
            }
        }
    }
    else
    {
        if (verbose)
        {
            #ifdef ERL_REALTIME_THROW_EXCEPTION
            throw std::runtime_error( "Could not retrieve memlock data: "+std::to_string(errno));
            #endif
            std::cerr << "Could not retrieve memlock data: " << errno << std::endl;
        }
        return false;
    }
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error( "setMemlockLimit not implemented on this platform");
    #endif
    std::cerr << std::endl << "setMemlockLimit not implemented on this platform" << std::endl;
    return false;
#endif
}

inline bool reserveStack(const size_t &bytes, bool verbose = true)
{
#ifdef __linux__
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        if (verbose)
        {
            #ifdef ERL_REALTIME_THROW_EXCEPTION
            throw std::runtime_error( "Could not lock memory pages:"+std::to_string(errno));
            #endif
            std::cerr << "Could not lock memory pages: " << errno << std::endl;
        }
        return false;
    }
    else
    {
        if (verbose)
            std::cout << "Current and future process memory locked" << std::endl;
    }
    char *tempAlloc = (char *)alloca(bytes);
    memset(tempAlloc, 0, bytes);

    if (verbose)
        std::cout << bytes/(std::pow<int>(2,20)) << " MB of memory reserved\n" << std::endl;
    return true;
#else
    #ifdef ERL_REALTIME_THROW_EXCEPTION
    throw std::logic_error( "reserveStack not implemented on this platform");
    #endif
    std::cerr << std::endl << "reserveStack not implemented on this platform" << std::endl;
    return false;
#endif
}

#ifdef _WIN32
ERL_RMV_UNPARA_END
#endif

// Helper functions ================================================================================
// =================================================================================================

namespace details
{

template <typename T, typename...Args> inline auto _thread_func(const RTparams &params, T function, const Args&...args) -> decltype(function(args...))
{
    Erl::RT::setSelfPolicy(params.policy, params.priority);
    if (params.cores.size() > 0)
        Erl::RT::setSelfAffinity(params.cores);
    return function(args...);
}

template <typename T, class Class, typename...Args> inline auto _thread_class(const RTparams &params, T function, Class *instance, const Args&...args)
                                                           -> decltype( (instance->*function)(args...) )
{
    Erl::RT::setSelfPolicy(params.policy, params.priority);
    if (params.cores.size() > 0)
        Erl::RT::setSelfAffinity(params.cores);
    return (instance->*function)(args...);
}

}

template <typename T, typename...Args> inline std::thread thread(T function, const RTparams &params, const Args&...args)
{
    return std::thread(&details::_thread_func<T, Args...>, params, function, args...);
}

template <typename T, class Class, typename...Args> inline std::thread thread(T function, Class *instance, const RTparams &params, const Args&...args)
{
    return std::thread(&details::_thread_class<T, Class, Args...>, params, function, instance, args...);
}

template <typename T, typename...Args> inline auto thread_future(T function, const RTparams &params, const Args&...args) -> std::future<decltype(function(args...))>
{
    std::packaged_task<decltype(function(args...))(RTparams, T, const Args&...)> tsk (details::_thread_func<T, Args...>);
    auto ret = tsk.get_future();
    std::thread th(std::move(tsk), params, function, args...);
    th.detach();
    return ret;
}

template <typename T, class Class, typename...Args> inline auto thread_future(T function, Class *instance, const RTparams &params, const Args&...args)
                                                           -> std::future< decltype( (instance->*function)(args...) ) >
{
    std::packaged_task< decltype( (instance->*function)(args...) ) (RTparams, T, Class*, const Args&...)> tsk (details::_thread_class<T, Class, Args...>);
    auto ret = tsk.get_future();
    std::thread th(std::move(tsk), params, function, instance, args...);
    th.detach();
    return ret;
}

template <typename TheadType>
inline bool join(TheadType& _thread)
{
    if(_thread.joinable())
    {
        try
        {
            _thread.join();
            return true;
        }
        catch(const std::system_error& _join_except)
        {

            if(_join_except.code() == std::errc::no_such_process)
                std::cout<<"join: no such process"<<std::endl;
            else if(_join_except.code() == std::errc::invalid_argument)
                std::cout<<"join: invalid argument"<<std::endl;
            else if(_join_except.code() == std::errc::resource_deadlock_would_occur)
                std::cerr<<"join: resource deadlock would occur: "<<_join_except.what()<<std::endl;
            else
                std::cerr<<"join: unknown system error: "<<_join_except.what()<<std::endl;

            return false;
        }
        catch(...)
        {
            std::cerr<<"join: unknown error"<<std::endl;
            return false;
        }
    }

    return false;
}

}
}
#endif
