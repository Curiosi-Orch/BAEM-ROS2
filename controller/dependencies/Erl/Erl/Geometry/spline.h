/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_SPLINE_H
#define ERL_SPLINE_H

#if ERL_CONSTEXPR_ACTIVE==ERL_TRUE
#include <Eigen/Dense>
#include "Erl/Utility/util.h"
namespace Erl
{
template<class T> class
Spline
{
typedef typename std::decay<typename Erl::extract_value_type<T>::value_type>::type  Real;
static ERL_USE_CONSTEXPR int Dof     = Erl::extract_value_type<T>::Rows   ;
static ERL_USE_CONSTEXPR int Rows    = Erl::extract_value_type<T>::Rows   ;
static ERL_USE_CONSTEXPR int Cols    = Erl::extract_value_type<T>::Cols   ;
static ERL_USE_CONSTEXPR int Options = Erl::extract_value_type<T>::Options;
static ERL_USE_CONSTEXPR int MaxRows = Erl::extract_value_type<T>::MaxRows;
static ERL_USE_CONSTEXPR int MaxCols = Erl::extract_value_type<T>::MaxCols;

typedef Eigen::Matrix<Real,Rows,4,Options> MatD4Internal;
typedef Eigen::Matrix<Real,Rows,2,Options> MatD2Internal;
typedef Eigen::Matrix<Real,4   ,1,Options> Vec4;
typedef Eigen::Matrix<Real,Dof ,1,Options> VecDof;

public :
    inline static size_t centripetal(const T& _inpoints, const Real& _dist, T& _outinterp, bool _autoalloc=false)
    {
        static_assert(Options == Eigen::ColMajor, "Input Matrix has to be ColMajor.");
        if(_autoalloc)
        {size_t iReqSize(centripetalsize(_inpoints,_dist));
         if(iReqSize<_outinterp.cols()){ _outinterp.resize(Eigen::NoChange,iReqSize);}}
        return interpolate(.5,_inpoints,_dist,_outinterp);
    }
    template<class C>
    inline static size_t centripetal(const C& _inpoints, const Real& _dist, C& _outinterp)
    {
        static_assert(Options == Eigen::ColMajor, "Input Matrix has to be ColMajor.");
        return interpolate(.5,_inpoints,_dist,_outinterp);
    }
    inline static size_t centripetalsize(const T& _inpoints, const Real& _dist)
    {
        static_assert(Options == Eigen::ColMajor, "Input Matrix has to be ColMajor.");
        return interpolatesize(.5,_inpoints,_dist);
    }
    inline static size_t chordal(const T& _inpoints, const Real& _dist, T& _outinterp, bool _autoalloc=false)
    {
        static_assert(Options == Eigen::ColMajor, "Input Matrix has to be ColMajor.");
        if(_autoalloc)
        {size_t iReqSize(chordalsize(_inpoints,_dist));
         if(iReqSize<_outinterp.cols()){ _outinterp.resize(Eigen::NoChange,iReqSize);}}
        return interpolate(1.,_inpoints,_dist,_outinterp);
    }
    template<class C>
    inline static size_t chordal(const C& _inpoints, const Real& _dist, C& _outinterp)
    {
        static_assert(Options == Eigen::ColMajor, "Input Matrix has to be ColMajor.");
        return interpolate(1.,_inpoints,_dist,_outinterp);
    }
    inline static size_t chordalsize(const T& _inpoints, const Real& _dist)
    {
        static_assert(Options == Eigen::ColMajor, "Input Matrix has to be ColMajor.");
        return interpolatesize(1.,_inpoints,_dist);
    }
    inline static size_t interpolate(Real _alpha, const T& _inpoints, const Real& _dist, T& _outinterp, bool _autoalloc=false)
    {
        if(_autoalloc)
        {size_t iReqSize(interpolatesize(_alpha,_inpoints,_dist));
         if(iReqSize<_outinterp.cols()){ _outinterp.resize(Eigen::NoChange,iReqSize);}}
        Vec4 c_t;  long c_NPoints = _inpoints.cols();
        ERL_ASSERT(c_NPoints>=4 && "Spline interpolation requires at least 4 points.");
        Real c_dist = _dist * std::pow(Dof,.5*(_alpha-1.));
        c_t[0] = Real(0.);
        c_t[1] =         calcDist<MatD2Internal,1>(_alpha,_inpoints.template topLeftCorner<Dof,2>());
        c_t[2] = c_t[1]+ calcDist<MatD2Internal,1>(_alpha,_inpoints.template block        <Dof,2>(0,1));
        c_t[3] = c_t[2]+ calcDist<MatD2Internal,1>(_alpha,_inpoints.template block        <Dof,2>(0,2));
        size_t c_AddedPoints(0);
        c_AddedPoints = interpBegin(c_t,_inpoints.template topLeftCorner<Dof,4>(),c_dist,c_AddedPoints,_outinterp);

        for(long iP(1);iP<(c_NPoints-3);iP++)
        {
            c_AddedPoints += interpCentre(c_t,_inpoints.template block<Dof,4>(0,iP-1),c_dist,c_AddedPoints,_outinterp);
            c_t[0] = c_t[1]; c_t[1] = c_t[2]-c_t[0]; c_t[2] = c_t[3]-c_t[0]; c_t[0] = 0;
            c_t[3] = c_t[2]+calcDist<MatD2Internal,1>(_alpha,_inpoints.template block<Dof,2>(0,iP+2));
        }
        c_AddedPoints += interpCentre(c_t,_inpoints.template block<Dof,4>(0,c_NPoints-4),c_dist,c_AddedPoints,_outinterp);
        c_AddedPoints += interpEnd   (c_t,_inpoints.template topRightCorner<Dof,4>(),c_dist,c_AddedPoints,_outinterp);
        return c_AddedPoints;
    }
    inline static size_t interpolatesize(Real _alpha, const T& _inpoints, const Real& _dist)
    {
        Vec4 c_t;  long c_NPoints = _inpoints.cols();
        ERL_ASSERT(c_NPoints>=4 && "Spline interpolation requires at least 4 points.");
        Real c_dist = _dist * std::pow(Dof,.5*(_alpha-1.));
        c_t[0] = Real(0.);
        c_t[1] =         calcDist<MatD2Internal,1>(_alpha,_inpoints.template topLeftCorner<Dof,2>());
        c_t[2] = c_t[1]+ calcDist<MatD2Internal,1>(_alpha,_inpoints.template block        <Dof,2>(0,1));
        c_t[3] = c_t[2]+ calcDist<MatD2Internal,1>(_alpha,_inpoints.template block        <Dof,2>(0,2));
        size_t c_AddedPoints(0);
        c_AddedPoints = outputsizeBegin(c_t,c_dist);

        for(long iP(1);iP<(c_NPoints-3);iP++)
        {
            c_AddedPoints += outputsizeCentre(c_t,c_dist);
            c_t[0] = c_t[1]; c_t[1] = c_t[2]-c_t[0]; c_t[2] = c_t[3]-c_t[0]; c_t[0] = 0;
            c_t[3] = c_t[2]+calcDist<MatD2Internal,1>(_alpha,_inpoints.template block<Dof,2>(0,iP+2));
        }
        c_AddedPoints += outputsizeCentre(c_t,c_dist);
        c_AddedPoints += outputsizeEnd   (c_t,c_dist);
        return c_AddedPoints;
    }
    template<class C>
    inline static size_t interpolate(Real _alpha, const C& _inpoints, const Real& _dist, C& _outinterp)
    {
        _outinterp.clear();
        Vec4 c_t;  long c_NPoints = _inpoints.size();
        auto c_inpointsIter_t  = _inpoints.begin();
        auto c_inpointsIter_i  = _inpoints.begin();
        Real c_dist = _dist * std::pow(Dof,.5*(_alpha-1.));
        c_t[0] = Real(0.);
        c_t[1] =         calcDist(_alpha,*(++c_inpointsIter_t),*c_inpointsIter_t);
        c_t[2] = c_t[1]+ calcDist(_alpha,*(++c_inpointsIter_t),*c_inpointsIter_t);
        c_t[3] = c_t[2]+ calcDist(_alpha,*(++c_inpointsIter_t),*c_inpointsIter_t);

        size_t c_AddedPoints = interpContainerBegin(c_t,c_inpointsIter_i,c_dist,_outinterp);

        for(long iP(1);iP<(c_NPoints-3);iP++)
        {
            c_AddedPoints += interpContainerCentre(c_t,c_inpointsIter_i,c_dist,_outinterp);
            c_t[0] = c_t[1]; c_t[1] = c_t[2]-c_t[0]; c_t[2] = c_t[3]-c_t[0]; c_t[0] = 0;
            c_t[3] = c_t[2]+calcDist(_alpha,*(++c_inpointsIter_t),*c_inpointsIter_t);
            c_inpointsIter_i++;
        }
        c_AddedPoints += interpContainerCentre(c_t,c_inpointsIter_i,c_dist,_outinterp);
        c_AddedPoints += interpContainerEnd   (c_t,c_inpointsIter_i,c_dist,_outinterp);
        return c_AddedPoints;
    }
protected:
    template<class Tin>
    inline static size_t interpCentre(const Vec4& _t, const Tin& _inpoints, const Real& _dist, const long& _coladd, T& _outinterp)
    {
        size_t c_NPoints = std::max<size_t>(2ULL,std::round((_t[2]-_t[1])/_dist));
        Real c_dist = (_t[2]-_t[1])/(c_NPoints-1);
        Real c_t; VecDof A1,A2,A3,B1,B2;
        for(size_t iP(0);iP<c_NPoints-1;iP++)
        {
            c_t = _t[1]+(iP+1)*c_dist;

            A1 = (_t[1]-c_t)/(_t[1]-_t[0])*_inpoints.col(0) + (c_t-_t[0])/(_t[1]-_t[0])*_inpoints.col(1);
            A2 = (_t[2]-c_t)/(_t[2]-_t[1])*_inpoints.col(1) + (c_t-_t[1])/(_t[2]-_t[1])*_inpoints.col(2);
            A3 = (_t[3]-c_t)/(_t[3]-_t[2])*_inpoints.col(2) + (c_t-_t[2])/(_t[3]-_t[2])*_inpoints.col(3);

            B1 = (_t[2]-c_t)/(_t[2]-_t[0])*A1 + (c_t-_t[0])/(_t[2]-_t[0])*A2;
            B2 = (_t[3]-c_t)/(_t[3]-_t[1])*A2 + (c_t-_t[1])/(_t[3]-_t[1])*A3;

            _outinterp.col(iP+_coladd)  = (_t[2]-c_t)/(_t[2]-_t[1])*B1 + (c_t-_t[1])/(_t[2]-_t[1])*B2;
        }
        return c_NPoints-1;
    }
    template<class Iter, class C>
    inline static size_t interpContainerCentre(const Vec4& _t, const Iter& _inpoints, const Real& _dist, C& _outinterp)
    {
        size_t c_NPoints = std::max<size_t>(2ULL,std::round((_t[2]-_t[1])/_dist));
        Real c_dist = (_t[2]-_t[1])/(c_NPoints-1);
        Real c_t; VecDof A1,A2,A3,B1,B2;
        for(size_t iP(0);iP<c_NPoints-1;iP++)
        {
            c_t = _t[1]+(iP+1)*c_dist;

            A1 = (_t[1]-c_t)/(_t[1]-_t[0])* *(_inpoints)              + (c_t-_t[0])/(_t[1]-_t[0])* *(std::next(_inpoints,1));
            A2 = (_t[2]-c_t)/(_t[2]-_t[1])* *(std::next(_inpoints,1)) + (c_t-_t[1])/(_t[2]-_t[1])* *(std::next(_inpoints,2));
            A3 = (_t[3]-c_t)/(_t[3]-_t[2])* *(std::next(_inpoints,2)) + (c_t-_t[2])/(_t[3]-_t[2])* *(std::next(_inpoints,3));

            B1 = (_t[2]-c_t)/(_t[2]-_t[0])*A1 + (c_t-_t[0])/(_t[2]-_t[0])*A2;
            B2 = (_t[3]-c_t)/(_t[3]-_t[1])*A2 + (c_t-_t[1])/(_t[3]-_t[1])*A3;

            _outinterp.push_back( (_t[2]-c_t)/(_t[2]-_t[1])*B1 + (c_t-_t[1])/(_t[2]-_t[1])*B2 );
        }
        return c_NPoints-1;
    }
    template<class Tin>
    inline static size_t interpBegin(const Vec4& _t, const Tin& _inpoints, const Real& _dist, const long& _coladd, T& _outinterp)
    {
        VecDof c_First = Real(2.)*_inpoints.col(0)-_inpoints.col(1);
        Real   c_t0 = -_t[1];
        size_t c_NPoints = std::max<size_t>(1ULL,std::round((_t[1]-_t[0])/_dist))+1;
        Real c_dist = (_t[1]-_t[0])/(c_NPoints-1);
        Real c_t; VecDof A1,A2,A3,B1,B2;
        for(size_t iP(0);iP<c_NPoints;iP++)
        {
            c_t = iP*c_dist;

            A1 = (_t[0]-c_t)/(_t[0]-c_t0 )*         c_First + (c_t-c_t0 )/(_t[0]-c_t0 )*_inpoints.col(0);
            A2 = (_t[1]-c_t)/(_t[1]-_t[0])*_inpoints.col(0) + (c_t-_t[0])/(_t[1]-_t[0])*_inpoints.col(1);
            A3 = (_t[2]-c_t)/(_t[2]-_t[1])*_inpoints.col(1) + (c_t-_t[1])/(_t[2]-_t[1])*_inpoints.col(2);

            B1 = (_t[1]-c_t)/(_t[1]-c_t0 )*A1 + (c_t-c_t0 )/(_t[1]-c_t0 )*A2;
            B2 = (_t[2]-c_t)/(_t[2]-_t[0])*A2 + (c_t-_t[0])/(_t[2]-_t[0])*A3;

            _outinterp.col(iP+_coladd)  = (_t[1]-c_t)/(_t[1]-_t[0])*B1 + (c_t-_t[0])/(_t[1]-_t[0])*B2;
        }
        return c_NPoints;
    }
    template<class Iter, class C>
    inline static size_t interpContainerBegin(const Vec4& _t, const Iter& _inpoints, const Real& _dist, C& _outinterp)
    {
        VecDof c_First = Real(2.)*(*(_inpoints))-*(std::next(_inpoints,1));
        Real   c_t0 = -_t[1];
        size_t c_NPoints = std::max<size_t>(1ULL,std::round((_t[1]-_t[0])/_dist))+1;
        Real c_dist = (_t[1]-_t[0])/(c_NPoints-1);
        Real c_t; VecDof A1,A2,A3,B1,B2;
        for(size_t iP(0);iP<c_NPoints;iP++)
        {
            c_t = iP*c_dist;

            A1 = (_t[0]-c_t)/(_t[0]-c_t0 )*                   c_First + (c_t-c_t0 )/(_t[0]-c_t0 )* *(             _inpoints);
            A2 = (_t[1]-c_t)/(_t[1]-_t[0])*            *(_inpoints  ) + (c_t-_t[0])/(_t[1]-_t[0])* *(std::next(_inpoints,1));
            A3 = (_t[2]-c_t)/(_t[2]-_t[1])* *(std::next(_inpoints,1)) + (c_t-_t[1])/(_t[2]-_t[1])* *(std::next(_inpoints,2));

            B1 = (_t[1]-c_t)/(_t[1]-c_t0 )*A1 + (c_t-c_t0 )/(_t[1]-c_t0 )*A2;
            B2 = (_t[2]-c_t)/(_t[2]-_t[0])*A2 + (c_t-_t[0])/(_t[2]-_t[0])*A3;

            _outinterp.push_back( (_t[1]-c_t)/(_t[1]-_t[0])*B1 + (c_t-_t[0])/(_t[1]-_t[0])*B2 );
        }
        return c_NPoints;
    }
    template<class Tin>
    inline static size_t interpEnd(const Vec4& _t, const Tin& _inpoints, const Real& _dist, const long& _coladd, T& _outinterp)
    {
        VecDof c_End = Real(2.)*_inpoints.col(3)-_inpoints.col(2);
        Real   c_t3  = Real(2.)*_t[3]-_t[2];
        size_t c_NPoints = std::max<size_t>(2ULL,std::round((_t[3]-_t[2])/_dist));
        Real c_dist = (_t[3]-_t[2])/(c_NPoints-1);
        Real c_t; VecDof A1,A2,A3,B1,B2;
        for(size_t iP(0);iP<c_NPoints-1;iP++)
        {
            c_t = _t[2]+(iP+1)*c_dist;

            A1 = (_t[2]-c_t)/(_t[2]-_t[1])*_inpoints.col(1) + (c_t-_t[1])/(_t[2]-_t[1])*_inpoints.col(2);
            A2 = (_t[3]-c_t)/(_t[3]-_t[2])*_inpoints.col(2) + (c_t-_t[2])/(_t[3]-_t[2])*_inpoints.col(3);
            A3 = (c_t3 -c_t)/(c_t3 -_t[3])*_inpoints.col(3) + (c_t-_t[3])/(c_t3 -_t[3])*c_End;

            B1 = (_t[3]-c_t)/(_t[3]-_t[1])*A1 + (c_t-_t[1])/(_t[3]-_t[1])*A2;
            B2 = (c_t3 -c_t)/(c_t3 -_t[2])*A2 + (c_t-_t[2])/(c_t3 -_t[2])*A3;

            _outinterp.col(iP+_coladd)  = (_t[3]-c_t)/(_t[3]-_t[2])*B1 + (c_t-_t[2])/(_t[3]-_t[2])*B2;
        }
        return c_NPoints-1;
    }
    template<class Iter, class C>
    inline static size_t interpContainerEnd(const Vec4& _t, const Iter& _inpoints, const Real& _dist, C& _outinterp)
    {
        VecDof c_End = Real(2.)*(*(std::next(_inpoints,3)))-*(std::next(_inpoints,2));
        Real   c_t3  = Real(2.)*_t[3]-_t[2];
        size_t c_NPoints = std::max<size_t>(2ULL,std::round((_t[3]-_t[2])/_dist));
        Real c_dist = (_t[3]-_t[2])/(c_NPoints-1);
        Real c_t; VecDof A1,A2,A3,B1,B2;
        for(size_t iP(0);iP<c_NPoints-1;iP++)
        {
            c_t = _t[2]+(iP+1)*c_dist;

            A1 = (_t[2]-c_t)/(_t[2]-_t[1])* *(std::next(_inpoints,1)) + (c_t-_t[1])/(_t[2]-_t[1])* *(std::next(_inpoints,2));
            A2 = (_t[3]-c_t)/(_t[3]-_t[2])* *(std::next(_inpoints,2)) + (c_t-_t[2])/(_t[3]-_t[2])* *(std::next(_inpoints,3));
            A3 = (c_t3 -c_t)/(c_t3 -_t[3])* *(std::next(_inpoints,3)) + (c_t-_t[3])/(c_t3 -_t[3])*  (                 c_End);

            B1 = (_t[3]-c_t)/(_t[3]-_t[1])*A1 + (c_t-_t[1])/(_t[3]-_t[1])*A2;
            B2 = (c_t3 -c_t)/(c_t3 -_t[2])*A2 + (c_t-_t[2])/(c_t3 -_t[2])*A3;

            _outinterp.push_back( (_t[3]-c_t)/(_t[3]-_t[2])*B1 + (c_t-_t[2])/(_t[3]-_t[2])*B2 );
        }
        return c_NPoints-1;
    }
    inline static size_t outputsizeCentre(const Vec4& _t, const Real& _dist)
    {
        return std::max<size_t>(2ULL,std::round((_t[2]-_t[1])/_dist))-1;
    }
    inline static size_t outputsizeBegin(const Vec4& _t, const Real& _dist)
    {
        return std::max<size_t>(1ULL,std::round((_t[1]-_t[0])/_dist))+1;
    }
    inline static size_t outputsizeEnd(const Vec4& _t, const Real& _dist)
    {
        return std::max<size_t>(2ULL,std::round((_t[3]-_t[2])/_dist))-1;
    }
    template<class Mat, int _iCol>
    inline static Real calcDist(Real _alpha, const Mat& _Input)
    {
        return (_Input.template rightCols<_iCol>()-_Input.template leftCols<_iCol>()).colwise()
                .squaredNorm().unaryExpr([_alpha](Real e){return std::pow(e,Real(.5)*_alpha);})
                .sum();
    }
    template<class Vec>
    inline static Real calcDist(Real _alpha, const Vec& _In0, const Vec& _In1)
    {
        return (_In1-_In0).colwise().squaredNorm()
               .unaryExpr([_alpha](Real e){return std::pow(e,Real(.5)*_alpha);})
               .sum();
    }
};
}
#endif

#endif // ERL_SPLINE_H
