/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_MOVINGFILTER_H
#define ERL_MOVINGFILTER_H

#include <vector>

namespace Erl
{

template<class T> class MovingFilter
{

public:

    inline MovingFilter(unsigned s, T initvalue = T())
        : size(s),
          counter(0),
          full(false)
    {
        Data.resize(size, initvalue);
        dOut = s*Data[0];
    }

    inline ~MovingFilter()
    {
    }

    inline void add (T Din)
    {
        dOut = dOut + (Din - Data[counter]);
        Data[counter] = Din;
        counter++;
        if (counter == size)
        {
            counter = 0;
            full = true;
        }
    }

    inline T get () const
    {
        if (full)
        {
            return dOut/size;
        }
        else
        {
            if (counter >= 1)
                return Data[counter - 1];
            else
                return Data[0];
        }
    }

    inline bool isfull () const
    {
        return full;
    }

    inline unsigned getCounter () const
    {
        return counter;
    }

    inline std::vector<T> getData () const
    {
        return Data;
    }

protected:

    int size, counter;
    bool full;
    std::vector<T> Data;
    T dOut;
};

}

#endif
