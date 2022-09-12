/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_SINGLETON_H
#define ERL_SINGLETON_H

namespace Erl
{

template <class T> class Singleton
{

public:

    inline static T& getInstance()
    {
        static T instance_;
        return instance_;
    }

private:

    inline Singleton(const Singleton &sing) = delete;
    inline Singleton & operator = (const Singleton &sing) = delete;

};


}

#endif

