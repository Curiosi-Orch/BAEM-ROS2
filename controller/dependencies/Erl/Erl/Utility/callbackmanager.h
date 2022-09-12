/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_CALLBACKMANAGER
#define ERL_CALLBACKMANAGER

#include <list>
#include <mutex>

namespace Erl
{

namespace details
{

template <typename... Types>
class CallbackContainerBase
{

public:

    inline virtual ~CallbackContainerBase() {}
    virtual void handleCallback(Types&&... args) = 0;
};

template<class F, typename... Types>
class CallbackContainer : public CallbackContainerBase<Types...>
{

public:

    inline virtual ~CallbackContainer() {}

    inline void createCallback(F function)
    {
        func_ = function;
    }

    inline void handleCallback(Types&&... args)
    {
        *func_(std::forward<Types>(args)...);
    }

private:

    F func_;
};

template<class F, class Class, typename... Types>
class CallbackClassContainer : public CallbackContainerBase<Types...>
{

public:

    inline virtual ~CallbackClassContainer() {}

    inline void createCallback(F function, Class *classInstance)
    {
        func_ = function;
        instance_ = classInstance;
    }

    inline void handleCallback(Types&&... args)
    {
        (instance_->*func_)(std::forward<Types>(args)...);
    }

private:

    F func_;
    Class *instance_;
};

}

template <typename lockable, typename... Types>
class CallbackManager_t
{

public:

    inline CallbackManager_t()
        : callbacks_(new std::list<details::CallbackContainerBase<Types...>*>()) {}

    inline ~CallbackManager_t()
    {
        typename std::list<details::CallbackContainerBase<Types...>*>::iterator it;
        for (it = callbacks_->begin(); it != callbacks_->end(); ++it)
            delete (*it);
        delete callbacks_;
    }

    template<class F> inline int createCallback(F function)
    {
        details::CallbackContainer<F, Types...> *container = new details::CallbackContainer<F, Types...>;
        container->createCallback(function);

        handleLock_.lock();
        callbacks_->push_back(container);
        int ret = callbacks_->size() - 1;
        handleLock_.unlock();
        return ret;
    }

    template<class F, class Class> inline int createCallback(F function, Class *classInstance)
    {
        details::CallbackClassContainer<F, Class, Types...> *container = new details::CallbackClassContainer<F, Class, Types...>;
        container->createCallback(function, classInstance);

        handleLock_.lock();
        callbacks_->push_back(container);
        int ret = callbacks_->size() - 1;
        handleLock_.unlock();
        return ret;
    }

    inline void handleCallbacks(Types&&... args)
    {
        handleLock_.lock();

        typename std::list<details::CallbackContainerBase<Types...>*>::iterator it;
        for (it = callbacks_->begin(); it != callbacks_->end(); ++it)
        {
            (*it)->handleCallback(std::forward<Types>(args)...);
        }

        handleLock_.unlock();
    }

    inline int getNumberOfCallbacks() const
    {
        handleLock_.lock();
        int ret = callbacks_->size();
        handleLock_.unlock();
        return ret;
    }

    inline void deleteCallback(const unsigned &index)
    {
        handleLock_.lock();
        int toClear = std::min(index, (unsigned)callbacks_->size() - 1);
        typename std::list<details::CallbackContainerBase<Types...>*>::iterator it = callbacks_->begin();
        std::advance(it, toClear);
        delete (*it);
        callbacks_->erase(it);
        handleLock_.unlock();
    }

    inline void deleteAllCallbacks()
    {
        handleLock_.lock();
        typename std::list<details::CallbackContainerBase<Types...>*>::iterator it;
        for (it = callbacks_->begin(); it != callbacks_->end(); ++it)
        {
            delete (*it);
        }
        callbacks_->clear();
        handleLock_.unlock();
    }

    std::list<details::CallbackContainerBase<Types...>*> *callbacks_;
    mutable lockable handleLock_;
};

template<typename... Types> using CallbackManager = CallbackManager_t<std::mutex, Types...>;

}

#endif

