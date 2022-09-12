/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_PQ_DISCRETIZE_H
#define ERL_PQ_DISCRETIZE_H

#include "pq_math.h"

namespace PQ{
template<class T>
size_t discHalfUVSphereSize(const T& _length, const Erl::Vector3<T>& _p,const Erl::Vector3<T>& _n,const T& _r)
{
    size_t c_Points = 1;
    size_t c_Ncirc = std::ceil( (ERL_PI/2. * _r) / _length);
    T c_r(_r); c_Points += std::ceil( (2.*ERL_PI* c_r) / _length);
    for(size_t iC(1);iC<c_Ncirc;iC++)
    {
        c_r = _r*std::cos( (ERL_PI/2.)* T(iC)/T(c_Ncirc) );
        c_Points += std::ceil( (2.*ERL_PI* c_r) / _length);
    }
    return c_Points;
}
template <class T , class matrix>
inline size_t discHalfUVSphere(const T& _length, const Erl::Vector3<T>& _p,const Erl::Vector3<T>& _n,const T& _r, matrix&& _discrPoints)
{
    size_t c_Points = 0;
    size_t c_CircPoints = 0;
    size_t c_Ncirc = std::ceil( (ERL_PI/2. * _r) / _length);
    T c_r,c_su,c_sv;

    Erl::Vector3<T> c_u,c_v,c_p;
    null_n(_n,c_u,c_v);

    for(size_t iC(0);iC<c_Ncirc;iC++)
    {
        c_r          =         _r*std::cos(T(ERL_PI/2.) * T(iC)/T(c_Ncirc));
        c_p          = _p + _n*_r*std::sin(T(ERL_PI/2.) * T(iC)/T(c_Ncirc));
        c_CircPoints =           std::ceil(T(2.*ERL_PI) *  c_r / _length);

        for(size_t iP(0);iP<c_CircPoints;iP++)
        {
            c_su = std::sin(T(2.*ERL_PI*iP)/c_CircPoints);
            c_sv = std::cos(T(2.*ERL_PI*iP)/c_CircPoints);
            _discrPoints.col(c_Points++) = c_p + c_r*(c_su*c_u + c_sv*c_v);
        }
    }
    _discrPoints.col(c_Points++) = _p + _n*_r;
    return c_Points;
}

}




#endif // ERL_PQ_DISCRETIZE_H
