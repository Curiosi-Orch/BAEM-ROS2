/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_REQUESTMANAGER_H
#define ERL_REQUESTMANAGER_H

#include <list>
#include <tuple>
#include <mutex>
#include <algorithm>

namespace Erl
{

namespace details
{

/// Non-member function callback

template <typename F, typename Tuple, bool done, int total, int... params> struct rqm_call_impl
{
    static void call(F f, Tuple && t)
    {
        rqm_call_impl<F, Tuple, total == 1 + sizeof...(params), total, params..., sizeof...(params)>::call(f, std::forward<Tuple>(t));
    }
};

template <typename F, typename Tuple, int total, int... params> struct rqm_call_impl<F, Tuple, true, total, params...>
{
    static void call(F f, Tuple && t)
    {
        f(std::get<params>(std::forward<Tuple>(t))...);
    }
};

template <typename F, typename Tuple> void rqm_call(F f, Tuple && t)
{
    typedef typename std::decay<Tuple>::type ttype;
    rqm_call_impl<F, Tuple, 0 == std::tuple_size<ttype>::value, std::tuple_size<ttype>::value>::call(f, std::forward<Tuple>(t));
}

/// Member function callback

template <typename F, typename I, typename Tuple, bool done, int total, int... params> struct rqm_call_impl_class
{
    static void call(F f, I *i, Tuple && t)
    {
        rqm_call_impl_class<F, I, Tuple, total == 1 + sizeof...(params), total, params..., sizeof...(params)>::call(f, i, std::forward<Tuple>(t));
    }
};

template <typename F, typename I, typename Tuple, int total, int... params> struct rqm_call_impl_class<F, I, Tuple, true, total, params...>
{
    static void call(F f, I *i, Tuple && t)
    {
        (i->*f)(std::get<params>(std::forward<Tuple>(t))...);
    }
};

template <typename F, typename I, typename Tuple> void rqm_call_class(F f, I *i, Tuple && t)
{
    typedef typename std::decay<Tuple>::type ttype;
    rqm_call_impl_class<F, I, Tuple, 0 == std::tuple_size<ttype>::value, std::tuple_size<ttype>::value>::call(f, i, std::forward<Tuple>(t));
}

class RequestContainerBase
{

public:

    inline virtual ~RequestContainerBase() {}
    virtual void handleRequest() = 0;
};

template<class F, typename... Types> class RequestContainer : public RequestContainerBase
{

public:

    inline virtual ~RequestContainer() {}

    inline void createRequest(F function, Types&&... args)
    {
        func_ = function;
        params_ = std::make_tuple(std::forward<Types>(args)...);
    }

    inline void handleRequest()
    {
        rqm_call(func_, params_);
    }

private:

    std::tuple<typename std::decay<Types>::type...> params_;
    F func_;
};

template<class F, class Class, typename... Types> class RequestClassContainer : public RequestContainerBase
{

public:

    inline virtual ~RequestClassContainer() {}

    inline void createRequest(F function, Class *classInstance, Types&&... args)
    {
        func_ = function;
        params_ = std::make_tuple(std::forward<Types>(args)...);
        instance_ = classInstance;
    }

    inline void handleRequest()
    {
        rqm_call_class(func_, instance_, params_);
    }

private:

    std::tuple<typename std::decay<Types>::type...> params_;
    F func_;
    Class *instance_;
};

}

template <typename lockable>
class RequestManager_t
{

public:

     inline RequestManager_t()
        : requests_(new std::list<details::RequestContainerBase*>()) {}

    inline ~RequestManager_t()
    {
        std::list<details::RequestContainerBase*>::iterator it;
        for (it = requests_->begin(); it != requests_->end(); ++it)
            delete (*it);
        delete requests_;
    }

    template<class F, typename... Types> inline void createRequest(F function, Types&&... args)
    {
        details::RequestContainer<F, Types...> *container = new details::RequestContainer<F, Types...>;
        container->createRequest(function, std::forward<Types>(args)...);

        requestLock_.lock();
        requests_->push_back(container);
        requestLock_.unlock();
    }

    template<class F, class Class, typename... Types> inline void createRequest(F function, Class *classInstance, Types&&... args)
    {
        details::RequestClassContainer<F, Class, Types...> *container = new details::RequestClassContainer<F, Class, Types...>;
        container->createRequest(function, classInstance, std::forward<Types>(args)...);

        requestLock_.lock();
        requests_->push_back(container);
        requestLock_.unlock();
    }

    inline void handleAllRequests()
    {
        requestLock_.lock();
        if (requests_->size() == 0)
        {
            requestLock_.unlock();
            return;
        }
        std::list<details::RequestContainerBase*> *storedRequest = requests_;
        requests_ = new std::list<details::RequestContainerBase*>();
        requestLock_.unlock();


        std::list<details::RequestContainerBase*>::iterator it;
        for (it = storedRequest->begin(); it != storedRequest->end(); ++it)
        {
            (*it)->handleRequest();
            delete (*it);
        }

        delete storedRequest;
    }

    inline void handleRequest()
    {
        requestLock_.lock();
        if (requests_->size() == 0)
        {
            requestLock_.unlock();
            return;
        }
        details::RequestContainerBase *storedRequest = requests_->front();
        requests_->pop_front();
        requestLock_.unlock();

        storedRequest->handleRequest();
        delete storedRequest;
    }

    inline void handleAllRequestsBlocking()
    {
        handleLock_.lock();
        handleAllRequests();
        handleLock_.unlock();
    }

    inline void handleRequestBlocking()
    {
        handleLock_.lock();
        handleRequest();
        handleLock_.unlock();
    }

    inline int getNumberOfRequests() const
    {
        requestLock_.lock();
        int temp = requests_->size();
        requestLock_.unlock();
        return temp;
    }

    inline void deleteFirstRequests(const unsigned &numberToDelete)
    {
        requestLock_.lock();
        int toClear = std::min(numberToDelete, (unsigned)requests_->size());
        std::list<details::RequestContainerBase*>::iterator it = requests_->begin();
        for (int i = 0; i < toClear; ++i, ++it)
        {
            delete (*it);
        }
        requests_->erase(requests_->begin(), it);
        requestLock_.unlock();
    }

    inline void deleteLastRequests(const unsigned &numberToDelete)
    {
        requestLock_.lock();
        int toClear = std::min(numberToDelete, (unsigned)requests_->size());
        std::list<details::RequestContainerBase*>::iterator it = requests_->end();
        for (int i = 0; i < toClear; ++i, --it)
        {
            delete (*it);
        }
        requests_->erase(it, requests_->end());
        requestLock_.unlock();
    }

    std::list<details::RequestContainerBase*> *requests_;
    mutable lockable requestLock_;
    mutable lockable handleLock_;
};

typedef RequestManager_t<std::mutex> RequestManager;

}

#endif
