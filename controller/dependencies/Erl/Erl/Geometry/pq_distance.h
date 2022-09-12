/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_PQ_DISTANCE_H
#define ERL_PQ_DISTANCE_H

#include <limits>
#include <Erl/_forwards.h>
#include <Erl/Core.h>
#include <Erl/Utility/util.h>


#include "pq_primitives.h"
#include "pq_math.h"

#ifdef GC_PQPROJECT
#include "branchcounter.h"
#endif

namespace PQ{
using namespace Erl;

#if defined(__GNUC__) || (_MSC_VER >= 1900)
template<class T>
struct Constants
{
    static ERL_USE_CONSTEXPR T Zero_Tolerance        = 0;
    static ERL_USE_CONSTEXPR long double Max_Real    = std::numeric_limits<T>::max();
};
template<>
struct Constants<long double>
{
    static ERL_USE_CONSTEXPR long double Zero_Tolerance   = 1.e-10;
    static ERL_USE_CONSTEXPR long double Max_Real    = std::numeric_limits<long double>::max();
};
template<>
struct Constants<double>
{
    static ERL_USE_CONSTEXPR double Zero_Tolerance        = 1.e-8;
    static ERL_USE_CONSTEXPR long double Max_Real    = std::numeric_limits<double>::max();
};
template<>
struct Constants<float>
{
    static ERL_USE_CONSTEXPR float Zero_Tolerance         = 1.e-6;
    static ERL_USE_CONSTEXPR long double Max_Real    = std::numeric_limits<float>::max();
};
template <class T>
ERL_USE_CONSTEXPR T Constants<T>::Zero_Tolerance        ;
#endif

//***********************************************************
template<typename T, typename PT, typename OT>
struct PinO
{
    inline bool operator()(const PT&, const OT& ) const noexcept
    {
        return true;
    }
};

template<typename T>
struct PinO< T, Point<T> ,Triangle<T> >
{
    inline bool operator()(const Point<T>& _p, const Triangle<T>& _tri) const noexcept
    {
        // Compute vectors
        Erl::Vector3<T> v0 = _tri.p2() - _tri.p0();
        Erl::Vector3<T> v1 = _tri.p1() - _tri.p0();
        Erl::Vector3<T> v2 = _p - _tri.p0();

        // Compute dot products
        T c_d00 = v0.dot(v0);
        T c_d01 = v0.dot(v1);
        T c_d02 = v0.dot(v2);
        T c_d11 = v1.dot(v1);
        T c_d12 = v1.dot(v2);

        // Compute barycentric coordinates
        T i_den = T(1.) / (c_d00 * c_d11 - c_d01 * c_d01);
        T u = (c_d11 * c_d02 - c_d01 * c_d12) * i_den;
        T v = (c_d00 * c_d12 - c_d01 * c_d02) * i_den;

        T c_dplane = calcP2O(_p,Plane<T>(_tri.p0(),v0.cross(v1))).norm();

        // Check if point is in triangle
        return (c_dplane <Erl::Constants<T>::Zero_Tolerance
                && (u >= -Erl::Constants<T>::Zero_Tolerance)
                && (v >= -Erl::Constants<T>::Zero_Tolerance)
                && (u + v < (T(1)+Erl::Constants<T>::Zero_Tolerance)) );
    }
};
template<typename T>
struct PinO< T, Point<T> ,Contour<T> >
{
    inline bool operator()(const Point<T>& _p, const Contour<T>& _con) const noexcept
    {
        //T c_dplane  = std::abs((_con.p()-_p).normalized().dot(_con.n().normalized()));
        T c_dplane  = calcP2O(_p,Plane<T>(_con.p(),_con.n())).norm();
        T c_dcentre = (_p-_con.p()).norm();

        //return (   !( 1e-2 < c_dplane/*Erl::Constants<T>::Zero_Tolerance*/)
        //         && (c_dcentre<(_con.r()+Erl::Constants<T>::Zero_Tolerance)) );

        return (    (c_dplane < Erl::Constants<T>::Zero_Tolerance)
                 && (c_dcentre<(_con.r()+Erl::Constants<T>::Zero_Tolerance)) );

    }
};


//******************************************************************************************************************************************************
// calcP2O: calc PROXY to OBJECT calculate points on object and proxy

//*** POINT TO OBJECT **************************************************
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p1, const Vector3<T>& _p2){
    return ProxyRes<T>(_p1,_p2);
}
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p,const Sphere<T>& _s){
    return ProxyRes<T>(_p,_s.p()+(_p-_s.p()).unitvec()*_s.r());
}
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p, const Line<T>& _linf){
    return ProxyRes<T>(_p,
                       _linf.p()+
                       _linf.v()*(_linf.v().dot
                       (_p-_linf.p()))/_linf.v().squaredNorm()
                       );
}
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p, const Ray<T>& _r){
    return ProxyRes<T>(_p,_r.p()+_r.v()*std::max(T(0.),_r.v().dot(_p-_r.p())/_r.v().squaredNorm()));
}
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p, const Segment<T>& _l){
    Vector3<T> _v(_l[1]-_l[0]);
    return ProxyRes<T>(_p,_l[0]+(_v*Erl::clamp<T>((_v.dot(_p-_l[0]))/_v.squaredNorm(),T(0),T(1))));
}
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p, const Plane<T>& _plane){
    return ProxyRes<T>(_p,_p-_plane.n()*_plane.n().dot(_p-_plane.p()));
}
#define VERTEX_TRIANGLE_ORIGINAL
#ifdef VERTEX_TRIANGLE_ORIGINAL
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p, const Triangle<T>& _tri){
#else
template<typename T> inline ProxyRes<T> calcP2Oo(const Vector3<T>& _p, const Triangle<T>& _tri){
#endif
    Vector3<T> diff  = _tri.p0() - _p;
    Vector3<T> edge0 = _tri.p1() - _tri.p0();
    Vector3<T> edge1 = _tri.p2() - _tri.p0();



    T a00 = edge0.squaredNorm();
    T a01 = edge0.dot(edge1);
    T a11 = edge1.squaredNorm();
    T b0  = diff.dot(edge0);
    T b1  = diff.dot(edge1);
    T det = std::fabs(a00*a11 - a01*a01);
    T s   = a01*b1 - a11*b0;
    T t   = a01*b0 - a00*b1;

    if (s + t <= det)
    {
        if (s < (T)0)
        {
            if (t < (T)0)  // region 4
            {
                if (b0 < (T)0)
                {
                    t = (T)0;
                    if (-b0 >= a00)
                    {
                        s = (T)1;

                    }
                    else
                    {
                        s = -b0/a00;

                    }
                }
                else
                {
                    s = (T)0;
                    if (b1 >= (T)0)
                    {
                        t = (T)0;

                    }
                    else if (-b1 >= a11)
                    {
                        t = (T)1;

                    }
                    else
                    {
                        t = -b1/a11;

                    }
                }
            }
            else  // region 3
            {
                s = (T)0;
                if (b1 >= (T)0)
                {
                    t = (T)0;

                }
                else if (-b1 >= a11)
                {
                    t = (T)1;

                }
                else
                {
                    t = -b1/a11;

                }
            }
        }
        else if (t < (T)0)  // region 5
        {
            t = (T)0;
            if (b0 >= (T)0)
            {
                s = (T)0;

            }
            else if (-b0 >= a00)
            {
                s = (T)1;

            }
            else
            {
                s = -b0/a00;

            }
        }
        else  // region 0
        {
            // minimum at interior point
            T invDet = ((T)1)/det;
            s *= invDet;
            t *= invDet;

        }
    }
    else
    {
        T tmp0, tmp1, numer, denom;

        if (s < (T)0)  // region 2
        {
            tmp0 = a01 + b0;
            tmp1 = a11 + b1;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - ((T)2)*a01 + a11;
                if (numer >= denom)
                {
                    s = (T)1;
                    t = (T)0;

                }
                else
                {
                    s = numer/denom;
                    t = (T)1 - s;

                }
            }
            else
            {
                s = (T)0;
                if (tmp1 <= (T)0)
                {
                    t = (T)1;

                }
                else if (b1 >= (T)0)
                {
                    t = (T)0;

                }
                else
                {
                    t = -b1/a11;

                }
            }
        }
        else if (t < (T)0)  // region 6
        {
            tmp0 = a01 + b1;
            tmp1 = a00 + b0;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - ((T)2)*a01 + a11;
                if (numer >= denom)
                {
                    t = (T)1;
                    s = (T)0;

                }
                else
                {
                    t = numer/denom;
                    s = (T)1 - t;

                }
            }
            else
            {
                t = (T)0;
                if (tmp1 <= (T)0)
                {
                    s = (T)1;

                }
                else if (b0 >= (T)0)
                {
                    s = (T)0;

                }
                else
                {
                    s = -b0/a00;
                }
            }
        }
        else  // region 1
        {
            numer = a11 + b1 - a01 - b0;
            if (numer <= (T)0)
            {
                s = (T)0;
                t = (T)1;

            }
            else
            {
                denom = a00 - ((T)2)*a01 + a11;
                if (numer >= denom)
                {
                    s = (T)1;
                    t = (T)0;

                }
                else
                {
                    s = numer/denom;
                    t = (T)1 - s;

                }
            }
        }
    }
    return ProxyRes<T>(_p,_tri.p0()+ s*edge0 + t*edge1);
}
//#define VAR2
#ifndef VERTEX_TRIANGLE_ORIGINAL_
//#ifdef  VAR2
template<typename T> inline ProxyRes<T> calcP2On2(const Vector3<T>& _p, const Triangle<T>& _tri)
{
    Vector3<T> diff  = _tri.p0() - _p;
    Vector3<T> edge0 = _tri.p1() - _tri.p0();
    Vector3<T> edge1 = _tri.p2() - _tri.p0();

    T a00 = edge0.squaredNorm();
    T a01 = edge0.dot(edge1);
    T a11 = edge1.squaredNorm();
    T b0  = diff.dot(edge0);
    T b1  = diff.dot(edge1);
    T det = a00*a11 - a01*a01;
    T s   = a01*b1 - a11*b0;
    T t   = a01*b0 - a00*b1;

    if (s + t <= det)
    {
//        T _sn = (s<0); T _tn = (t<0);
//        if(_sn||_tn)
//        {
//            s =     Erl::clamp<T>(-b0/a00,0,  _tn );
//            t = _sn*Erl::clamp<T>(-b1/a11,0, 1 - s);
//        }
        if (s < T(0))
        {
            s = Erl::clamp<T>(-b0/a00,0,T(t<0) );
            t = Erl::clamp<T>(-b1/a11,0,1-s);
        }
        else if (t < T(0))  // region 5
        {
            t = (T)0;
            s = Erl::clamp<T>( -b0/a00,0,1);
        }
        else  // region 0
        {
            // minimum at interior point
            T invDet = T(1)/det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        T numer;
        T denom = a00 + a11 - T(2)*a01 ; // >=0
        if (s < T(0))  // region 2
        {
            numer = (a11 + b1) - (a01 + b0);
            s = Erl::clamp<T>(numer/denom,0,1  );
            t = Erl::clamp<T>(-b1/a11    ,0,1-s);
        }
        else if (t < T(0))  // region 6
        {
            numer = (a00 + b0) - (a01 + b1);
            t = Erl::clamp<T>(numer/denom,0,1  );
            s = Erl::clamp<T>(-b0/a00    ,0,1-t);
        }
        else  // region 1
        {
            numer = a11 + b1 - a01 - b0;
            s = Erl::clamp<T>(numer/denom,0,1);
            t = T(1)-s;
        }
    }
    return ProxyRes<T>(_p,_tri.p0()+ s*edge0 + t*edge1);
}
//#else
template<typename T> inline ProxyRes<T> calcP2On1(const Vector3<T>& _p, const Triangle<T>& _tri){
    Vector3<T> diff  = _tri.p0() - _p;
    Vector3<T> edge0 = _tri.p1() - _tri.p0();
    Vector3<T> edge1 = _tri.p2() - _tri.p0();

    // #define VERTEX_TRIANGLE_ORIGINAL

    T a00 = edge0.squaredNorm();
    T a01 = edge0.dot(edge1);
    T a11 = edge1.squaredNorm();
    T b0  = diff.dot(edge0);
    T b1  = diff.dot(edge1);
    T det = std::fabs(a00*a11 - a01*a01);
    T s   = a01*b1 - a11*b0;
    T t   = a01*b0 - a00*b1;

    if (s + t <= det)
    {
        if (s < (T)0)
        {
            if (t < (T)0)  // region 4
            {
                if (b0 < (T)0)
                {
                    t = (T)0;
                    s = std::min<T>(-b0/a00,1);
                }
                else
                {
                    s = (T)0;
                    t = Erl::clamp<T>( -b1/a11,0,1);
                }
            }
            else  // region 3
            {
                s = (T)0;
                t = Erl::clamp<T>( -b1/a11,0,1);
            }
        }
        else if (t < (T)0)  // region 5
        {
            t = (T)0;
            s = Erl::clamp<T>( -b0/a00,0,1);
        }
        else  // region 0
        {
            // minimum at interior point
            T invDet = ((T)1)/det;
            s *= invDet;
            t *= invDet;

        }
    }
    else
    {
        T tmp0, tmp1, numer, denom;

        if (s < (T)0)  // region 2
        {
            tmp0 = a01 + b0;
            tmp1 = a11 + b1;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - ((T)2)*a01 + a11;
                s = std::min<T>(numer/denom,1);
                t = T(1)-s;
            }
            else
            {
                s = T(0);
                t = Erl::clamp<T>(-b1/a11,0,1);
            }
        }
        else if (t < (T)0)  // region 6
        {
            tmp0 = a01 + b1;
            tmp1 = a00 + b0;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - ((T)2)*a01 + a11;
                t = std::min<T>(numer/denom,1);
                s = T(1) - t;
            }
            else
            {
                t = (T)0;
                s = Erl::clamp<T>( -b0/a00,0,1);
            }
        }
        else  // region 1
        {
            numer = a11 + b1 - a01 - b0;
            denom = a00 - ((T)2)*a01 + a11;
            s = Erl::clamp<T>(numer/denom,0,1);
            t = T(1)-s;
        }
    }
    return ProxyRes<T>(_p,_tri.p0()+ s*edge0 + t*edge1);
}
//#endif
#endif
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p,const Circle<T>& _circ){
       // Absolute Distance from point to plane of circle.
       Vector3<T> diff0 = _p - _circ.p();
       T dist = diff0.dot(_circ.n());

       // Projection of P-C onto plane is Q-C = P-C - (fDist)*N.
       Vector3<T> diff1 = diff0 - dist*_circ.n();
       T dist1 = diff1.norm();

       return ProxyRes<T>(_p, _circ.p() + diff1*(_circ.r()/dist1));
}
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p, const Contour<T>& _con){
       // Absolute Distance from point to plane of circle.
       Vector3<T> diff0 = _p - _con.p();
       T dist = diff0.dot(_con.n());

       // Projection of P-C onto plane is Q-C = P-C - (fDist)*N.
       Vector3<T> diff1 = diff0 - dist*_con.n();
       T dist1 = diff1.norm();

       if(dist1>_con.r())
       {
           return ProxyRes<T>(_p, _con.p() + diff1*(_con.r()/dist1));

       }
       else{
            return ProxyRes<T>(_p, _con.p() + diff1);
       }


}
template<typename T> inline ProxyRes<T> calcP2O(const Vector3<T>& _p,const GCylinders<T>& _cyl){
        unsigned seg_idx[2];
		seg_idx[0]=(_cyl.getSegIdx(0));
		seg_idx[1]=(_cyl.getSegIdx(1)-1);
        Vector3<T> d1,d2,n,rhol1,rhol2,v2,v3,vd,vb1,vb2,vb3,vb4,_tmp_closest,_tmp_overall_closest;
        T _tmp_min_squardist = Constants<T>::Max_Real;
        T w_squardist,lamda1,lamda2,lamda3,lamda4;

    for (unsigned int j=seg_idx[0]; j<=seg_idx[1]; j++){

        // Step 1: n = (C[j]-p[i]) * (c[j+1]-p[i])
        if(j==seg_idx[0])
        {
            d1=_cyl[j].p()-_p;
        }
        else
        {
            d1=d2;

        }
        d2=_cyl[j+1].p()-_p;

        n=d1.xprod(d2);

        // Step 2: rhol = n[j] * M[j]
        rhol1=n.xprod(_cyl[j].n());
        rhol2=n.xprod(_cyl[j+1].n());


        // Step 3: v2 = C[j]+t[j].normal_roh1; v3 = C[j+1]+t[j+1].normal_roh2

        // Calculate v2
        v2 = _cyl[j].p() + rhol1*(_cyl[j].r()  /rhol1.norm());

        // Calculate v3
        v3 = _cyl[j+1].p()+rhol2*(_cyl[j+1].r()/rhol2.norm());


        // Step 4: find lamda (old version: less adders and multipliers)
        // vd = v3 -v1

        vd= v3-_cyl[j].p();

        // vb1=v1-p[i]; vb2=v2-p[i]; vb3=v3-p[i]; vb4=v4-p[i]
        vb1 =   _cyl[j].p()-_p;
        vb2 =           v2 -_p;
        vb3 =           v3 -_p;
        vb4 = _cyl[j+1].p()-_p;

        // lamda
        if ((vd[2] < vd[0]) && (vd[2] < vd[1])) {
            lamda1 = vb2[0]*vb1[1] - vb1[0]*vb2[1];
            lamda2 = vb3[0]*vb2[1] - vb2[0]*vb3[1];
            lamda3 = vb4[0]*vb3[1] - vb3[0]*vb4[1];
            lamda4 = vb1[0]*vb4[1] - vb4[0]*vb1[1];
        } else if ((vd[1] < vd[0]) && (vd[1] < vd[2])) {
            lamda1 = vb2[2]*vb1[0] - vb1[2]*vb2[0];
            lamda2 = vb3[2]*vb2[0] - vb2[2]*vb3[0];
            lamda3 = vb4[2]*vb3[0] - vb3[2]*vb4[0];
            lamda4 = vb1[2]*vb4[0] - vb4[2]*vb1[0];
        } else {
            lamda1 = vb2[1]*vb1[2] - vb1[1]*vb2[2];
            lamda2 = vb3[1]*vb2[2] - vb2[1]*vb3[2];
            lamda3 = vb4[1]*vb3[2] - vb3[1]*vb4[2];
            lamda4 = vb1[1]*vb4[2] - vb4[1]*vb1[2];
        }

        // Step 5: Calculate delta
        if ( ((lamda1 > 0) && (lamda2 > 0) && (lamda3 > 0) && (lamda4 > 0)) || ((lamda1 < 0) && (lamda2 < 0) && (lamda3 < 0) && (lamda4 < 0)) ){
            // if the point is inside the contour
            // return zero no further calculation required
            return ProxyRes<T>(_p,_p);

        }

        // If the point is outside the contour
        // u = -(v2-p).(v3-v2)/(||v3-v2||^2)
        Vector3<T> v32(v3 - v2);

        double u = -(vb2*v32) / v32.squaredNorm();
            // delta
            if (u>1){

                ////////////////////////////////////////////////////////
                //// Add for consideration of pathway ending
                if (j == seg_idx[1])
                {
                    Vector3<T> v43(_cyl[j+1].p()-v3);

                    double u_2 = - (vb3*v43)/v43.squaredNorm();
                    if (u_2 > 0)
                    {
                        _tmp_closest=v3*(T(1)-u_2)+_cyl[j+1].p()*u_2;
                        w_squardist=(_p-_tmp_closest).squaredNorm();


                    }
                    else
                    {
                        _tmp_closest=v3;
                        w_squardist=vb3.squaredNorm();

                    }
                }
                else
                {
                    _tmp_closest=v3;
                    w_squardist=vb3.squaredNorm();

                }
                ////====================== End =============================////

            }else if (u<0){

                ////////////////////////////////////////////////////////
                //// Add for consideration of pathway ending

                if (j == seg_idx[0])
                {
                    // double3 v21;
                    Vector3<T> v21(v2-_cyl[j].p());
                    double u_2 =  - (vb1*v21);
                    if (u_2 < 0)
                    {
                        u_2/=v21.squaredNorm();
                        _tmp_closest = _cyl[j].p()*(T(1)-u_2)+v2*u_2;
                        w_squardist=(_p-_tmp_closest).squaredNorm();

                    }
                    else
                    {
                        _tmp_closest=v2;
                        w_squardist = vb2.squaredNorm();
                    }
                } else
                {
                    _tmp_closest=v2;
                    w_squardist = vb2.squaredNorm();
                }

                ////====================== End =============================////

            }else {
                _tmp_closest=v2*(T(1)-u)+v3*u;
                w_squardist= (_p-_tmp_closest).squaredNorm();
            }

            if(w_squardist<_tmp_min_squardist)
            {
             _tmp_min_squardist=w_squardist;
             _tmp_overall_closest=_tmp_closest;
            }



    }
    return ProxyRes<T>(_tmp_overall_closest,_p);

}

//*** LINE_inf TO OBJECT **************************************************

//*** LINE TO OBJECT **************************************************
template<typename T> inline ProxyRes<T> calcP2O(const Segment<T>& _l,const Vector3<T>& _p)
{
    return calcP2O(_p,_l).swap();
}
template<typename T> inline ProxyRes<T> calcP2O(const Segment<T>& _l,const Sphere<T>& _s){
    Vector3<T> _v(_l[1]-_l[0]);
    T t0= Erl::clamp(_v*(_s.p()-_l[0]),T(0),T(1));
    Vector3<T> _pl= _l[0]+(_v*t0);
    return ProxyRes<T>(_pl,_s.p()+((_pl-_s.p()).unitvec()*_s.r()));
}
template<typename T> inline ProxyRes<T> calcP2Oo(const Segment<T>& _lProx,const Segment<T>& _lObj)
{
    Vector3<T> c_Prox[2]={_lProx.p0(),_lProx.p1()};
    T c_lProxRadius((c_Prox[0]-c_Prox[1]).norm());
    Vector3<T> c_lProxCentre((c_Prox[0]+c_Prox[1])*T(0.5));
    Vector3<T> c_lProxDirection((c_Prox[1]-c_Prox[0])/c_lProxRadius);
    c_lProxRadius/=T(2);

    Vector3<T> c_Obj[2]={_lObj.p0(),_lObj.p1()};
    T c_lObjRadius((c_Obj[0]-c_Obj[1]).norm());
    Vector3<T> c_lObjCentre((c_Obj[0]+c_Obj[1])*T(0.5));
    Vector3<T> c_lObjDirection((c_Obj[1]-c_Obj[0])/c_lObjRadius);
    c_lObjRadius/=T(2);


    Vector3<T> diff = c_lProxCentre - c_lObjCentre;
    T a01 = -c_lProxDirection.dot(c_lObjDirection);
    T b0 = diff.dot(c_lProxDirection);
    T b1 = -diff.dot(c_lObjDirection);
    T det = fabs(T(1) - a01*a01);
    T s0, s1, extDet0, extDet1, tmpS0, tmpS1;

    if (det >= Constants<T>::Zero_Tolerance)
    {
        // Segments are not parallel.
        s0 = a01*b1 - b0;
        s1 = a01*b0 - b1;
        extDet0 = c_lProxRadius*det;
        extDet1 = c_lObjRadius*det;

        if (s0 >= -extDet0)
        {
            if (s0 <= extDet0)
            {
                if (s1 >= -extDet1)
                {
                    if (s1 <= extDet1)  // region 0 (interior)
                    {
                        // Minimum at interior points of segments.
                        T invDet = T(1)/det;
                        s0 *= invDet;
                        s1 *= invDet;
                    }
                    else  // region 3 (side)
                    {
                        s1 = c_lObjRadius;
                        tmpS0 = -(a01*s1 + b0);
                        if (tmpS0 < -c_lProxRadius)
                        {
                            s0 = -c_lProxRadius;
                        }
                        else if (tmpS0 <= c_lProxRadius)
                        {
                            s0 = tmpS0;
                        }
                        else
                        {
                            s0 = c_lProxRadius;
                        }
                    }
                }
                else  // region 7 (side)
                {
                    s1 = -c_lObjRadius;
                    tmpS0 = -(a01*s1 + b0);
                    if (tmpS0 < -c_lProxRadius)
                    {
                        s0 = -c_lProxRadius;
                    }
                    else if (tmpS0 <= c_lProxRadius)
                    {
                        s0 = tmpS0;
                    }
                    else
                    {
                        s0 = c_lProxRadius;
                    }
                }
            }
            else
            {
                if (s1 >= -extDet1)
                {
                    if (s1 <= extDet1)  // region 1 (side)
                    {
                        s0 = c_lProxRadius;
                        tmpS1 = -(a01*s0 + b1);
                        if (tmpS1 < -c_lObjRadius)
                        {
                            s1 = -c_lObjRadius;
                        }
                        else if (tmpS1 <= c_lObjRadius)
                        {
                            s1 = tmpS1;
                        }
                        else
                        {
                            s1 = c_lObjRadius;
                        }
                    }
                    else  // region 2 (corner)
                    {
                        s1 = c_lObjRadius;
                        tmpS0 = -(a01*s1 + b0);
                        if (tmpS0 < -c_lProxRadius)
                        {
                            s0 = -c_lProxRadius;
                        }
                        else if (tmpS0 <= c_lProxRadius)
                        {
                            s0 = tmpS0;
                        }
                        else
                        {
                            s0 = c_lProxRadius;
                            tmpS1 = -(a01*s0 + b1);
                            if (tmpS1 < -c_lObjRadius)
                            {
                                s1 = -c_lObjRadius;
                            }
                            else if (tmpS1 <= c_lObjRadius)
                            {
                                s1 = tmpS1;
                            }
                            else
                            {
                                s1 = c_lObjRadius;
                            }
                        }
                    }
                }
                else  // region 8 (corner)
                {
                    s1 = -c_lObjRadius;
                    tmpS0 = -(a01*s1 + b0);
                    if (tmpS0 < -c_lProxRadius)
                    {
                        s0 = -c_lProxRadius;

                    }
                    else if (tmpS0 <= c_lProxRadius)
                    {
                        s0 = tmpS0;

                    }
                    else
                    {
                        s0 = c_lProxRadius;
                        tmpS1 = -(a01*s0 + b1);
                        if (tmpS1 > c_lObjRadius)
                        {
                            s1 = c_lObjRadius;

                        }
                        else if (tmpS1 >= -c_lObjRadius)
                        {
                            s1 = tmpS1;

                        }
                        else
                        {
                            s1 = -c_lObjRadius;

                        }
                    }
                }
            }
        }
        else
        {
            if (s1 >= -extDet1)
            {
                if (s1 <= extDet1)  // region 5 (side)
                {
                    s0 = -c_lProxRadius;
                    tmpS1 = -(a01*s0 + b1);
                    if (tmpS1 < -c_lObjRadius)
                    {
                        s1 = -c_lObjRadius;

                    }
                    else if (tmpS1 <= c_lObjRadius)
                    {
                        s1 = tmpS1;

                    }
                    else
                    {
                        s1 = c_lObjRadius;

                    }
                }
                else  // region 4 (corner)
                {
                    s1 = c_lObjRadius;
                    tmpS0 = -(a01*s1 + b0);
                    if (tmpS0 > c_lProxRadius)
                    {
                        s0 = c_lProxRadius;
                    }
                    else if (tmpS0 >= -c_lProxRadius)
                    {
                        s0 = tmpS0;
                    }
                    else
                    {
                        s0 = -c_lProxRadius;
                        tmpS1 = -(a01*s0 + b1);
                        if (tmpS1 < -c_lObjRadius)
                        {
                            s1 = -c_lObjRadius;

                        }
                        else if (tmpS1 <= c_lObjRadius)
                        {
                            s1 = tmpS1;

                        }
                        else
                        {
                            s1 = c_lObjRadius;

                        }
                    }
                }
            }
            else   // region 6 (corner)
            {
                s1 = -c_lObjRadius;
                tmpS0 = -(a01*s1 + b0);
                if (tmpS0 > c_lProxRadius)
                {
                    s0 = c_lProxRadius;

                }
                else if (tmpS0 >= -c_lProxRadius)
                {
                    s0 = tmpS0;

                }
                else
                {
                    s0 = -c_lProxRadius;
                    tmpS1 = -(a01*s0 + b1);
                    if (tmpS1 < -c_lObjRadius)
                    {
                        s1 = -c_lObjRadius;

                    }
                    else if (tmpS1 <= c_lObjRadius)
                    {
                        s1 = tmpS1;

                    }
                    else
                    {
                        s1 = c_lObjRadius;

                    }
                }
            }
        }
    }
    else
    {
        // The segments are parallel.  The average b0 term is designed to
        // ensure symmetry of the function.  That is, dist(seg0,seg1) and
        // dist(seg1,seg0) should produce the same number.
        T e0pe1 = c_lProxRadius + c_lObjRadius;
        T sign = (a01 > T(0) ? T(-1) : T(1));
        T b0Avr = T(0.5)*(b0 - sign*b1);
        T lambda = -b0Avr;
        if (lambda < -e0pe1)
        {
            lambda = -e0pe1;
        }
        else if (lambda > e0pe1)
        {
            lambda = e0pe1;
        }
        s1 = -sign*lambda*c_lObjRadius/e0pe1;
        s0 = lambda + sign*s1;
    }
    return ProxyRes<T> (c_lProxCentre + s0*c_lProxDirection,
        c_lObjCentre + s1*c_lObjDirection);
}

template<typename T> inline ProxyRes<T> calcP2O(const Segment<T>& _lProx,const Segment<T>& _lObj)
{
        //#define SEGMENT_SEGMENT_ORIGINAL
        #ifdef SEGMENT_SEGMENT_ORIGINAL
        Vector3<T> c_Prox[2]={_lProx.p0(),_lProx.p1()};
        T c_lProxRadius((c_Prox[0]-c_Prox[1]).norm());
        Vector3<T> c_lProxCentre((c_Prox[0]+c_Prox[1])*T(0.5));
        Vector3<T> c_lProxDirection((c_Prox[1]-c_Prox[0])/c_lProxRadius);
        c_lProxRadius/=T(2);

        Vector3<T> c_Obj[2]={_lObj.p0(),_lObj.p1()};
        T c_lObjRadius((c_Obj[0]-c_Obj[1]).norm());
        Vector3<T> c_lObjCentre((c_Obj[0]+c_Obj[1])*T(0.5));
        Vector3<T> c_lObjDirection((c_Obj[1]-c_Obj[0])/c_lObjRadius);
        c_lObjRadius/=T(2);


        Vector3<T> diff = c_lProxCentre - c_lObjCentre;
        T a01 = -c_lProxDirection.dot(c_lObjDirection);
        T b0 = diff.dot(c_lProxDirection);
        T b1 = -diff.dot(c_lObjDirection);
        T det = fabs(T(1) - a01*a01);
        T s0, s1, extDet0, extDet1, tmpS0, tmpS1;

        if (det >= Constants<T>::Zero_Tolerance)
        {
            // Segments are not parallel.
            s0 = a01*b1 - b0;
            s1 = a01*b0 - b1;
            extDet0 = c_lProxRadius*det;
            extDet1 = c_lObjRadius*det;

            if (s0 >= -extDet0)
            {
                if (s0 <= extDet0)
                {
                    if (s1 >= -extDet1)
                    {
                        if (s1 <= extDet1)  // region 0 (interior)
                        {
                            // Minimum at interior points of segments.
                            T invDet = T(1)/det;
                            s0 *= invDet;
                            s1 *= invDet;
                        }
                        else  // region 3 (side)
                        {
                            s1 = c_lObjRadius;
                            tmpS0 = -(a01*s1 + b0);
                            if (tmpS0 < -c_lProxRadius)
                            {
                                s0 = -c_lProxRadius;
                            }
                            else if (tmpS0 <= c_lProxRadius)
                            {
                                s0 = tmpS0;
                            }
                            else
                            {
                                s0 = c_lProxRadius;
                            }
                        }
                    }
                    else  // region 7 (side)
                    {
                        s1 = -c_lObjRadius;
                        tmpS0 = -(a01*s1 + b0);
                        if (tmpS0 < -c_lProxRadius)
                        {
                            s0 = -c_lProxRadius;
                        }
                        else if (tmpS0 <= c_lProxRadius)
                        {
                            s0 = tmpS0;
                        }
                        else
                        {
                            s0 = c_lProxRadius;
                        }
                    }
                }
                else
                {
                    if (s1 >= -extDet1)
                    {
                        if (s1 <= extDet1)  // region 1 (side)
                        {
                            s0 = c_lProxRadius;
                            tmpS1 = -(a01*s0 + b1);
                            if (tmpS1 < -c_lObjRadius)
                            {
                                s1 = -c_lObjRadius;
                            }
                            else if (tmpS1 <= c_lObjRadius)
                            {
                                s1 = tmpS1;
                            }
                            else
                            {
                                s1 = c_lObjRadius;
                            }
                        }
                        else  // region 2 (corner)
                        {
                            s1 = c_lObjRadius;
                            tmpS0 = -(a01*s1 + b0);
                            if (tmpS0 < -c_lProxRadius)
                            {
                                s0 = -c_lProxRadius;
                            }
                            else if (tmpS0 <= c_lProxRadius)
                            {
                                s0 = tmpS0;
                            }
                            else
                            {
                                s0 = c_lProxRadius;
                                tmpS1 = -(a01*s0 + b1);
                                if (tmpS1 < -c_lObjRadius)
                                {
                                    s1 = -c_lObjRadius;
                                }
                                else if (tmpS1 <= c_lObjRadius)
                                {
                                    s1 = tmpS1;
                                }
                                else
                                {
                                    s1 = c_lObjRadius;
                                }
                            }
                        }
                    }
                    else  // region 8 (corner)
                    {
                        s1 = -c_lObjRadius;
                        tmpS0 = -(a01*s1 + b0);
                        if (tmpS0 < -c_lProxRadius)
                        {
                            s0 = -c_lProxRadius;

                        }
                        else if (tmpS0 <= c_lProxRadius)
                        {
                            s0 = tmpS0;

                        }
                        else
                        {
                            s0 = c_lProxRadius;
                            tmpS1 = -(a01*s0 + b1);
                            if (tmpS1 > c_lObjRadius)
                            {
                                s1 = c_lObjRadius;

                            }
                            else if (tmpS1 >= -c_lObjRadius)
                            {
                                s1 = tmpS1;

                            }
                            else
                            {
                                s1 = -c_lObjRadius;

                            }
                        }
                    }
                }
            }
            else
            {
                if (s1 >= -extDet1)
                {
                    if (s1 <= extDet1)  // region 5 (side)
                    {
                        s0 = -c_lProxRadius;
                        tmpS1 = -(a01*s0 + b1);
                        if (tmpS1 < -c_lObjRadius)
                        {
                            s1 = -c_lObjRadius;

                        }
                        else if (tmpS1 <= c_lObjRadius)
                        {
                            s1 = tmpS1;

                        }
                        else
                        {
                            s1 = c_lObjRadius;

                        }
                    }
                    else  // region 4 (corner)
                    {
                        s1 = c_lObjRadius;
                        tmpS0 = -(a01*s1 + b0);
                        if (tmpS0 > c_lProxRadius)
                        {
                            s0 = c_lProxRadius;
                        }
                        else if (tmpS0 >= -c_lProxRadius)
                        {
                            s0 = tmpS0;
                        }
                        else
                        {
                            s0 = -c_lProxRadius;
                            tmpS1 = -(a01*s0 + b1);
                            if (tmpS1 < -c_lObjRadius)
                            {
                                s1 = -c_lObjRadius;

                            }
                            else if (tmpS1 <= c_lObjRadius)
                            {
                                s1 = tmpS1;

                            }
                            else
                            {
                                s1 = c_lObjRadius;

                            }
                        }
                    }
                }
                else   // region 6 (corner)
                {
                    s1 = -c_lObjRadius;
                    tmpS0 = -(a01*s1 + b0);
                    if (tmpS0 > c_lProxRadius)
                    {
                        s0 = c_lProxRadius;

                    }
                    else if (tmpS0 >= -c_lProxRadius)
                    {
                        s0 = tmpS0;

                    }
                    else
                    {
                        s0 = -c_lProxRadius;
                        tmpS1 = -(a01*s0 + b1);
                        if (tmpS1 < -c_lObjRadius)
                        {
                            s1 = -c_lObjRadius;

                        }
                        else if (tmpS1 <= c_lObjRadius)
                        {
                            s1 = tmpS1;

                        }
                        else
                        {
                            s1 = c_lObjRadius;

                        }
                    }
                }
            }
        }
        else
        {
            // The segments are parallel.  The average b0 term is designed to
            // ensure symmetry of the function.  That is, dist(seg0,seg1) and
            // dist(seg1,seg0) should produce the same number.
            T e0pe1 = c_lProxRadius + c_lObjRadius;
            T sign = (a01 > T(0) ? T(-1) : T(1));
            T b0Avr = T(0.5)*(b0 - sign*b1);
            T lambda = -b0Avr;
            if (lambda < -e0pe1)
            {
                lambda = -e0pe1;
            }
            else if (lambda > e0pe1)
            {
                lambda = e0pe1;
            }
            s1 = -sign*lambda*c_lObjRadius/e0pe1;
            s0 = lambda + sign*s1;
        }
        return ProxyRes<T> (c_lProxCentre + s0*c_lProxDirection,
            c_lObjCentre + s1*c_lObjDirection);
        #else


        Erl::Vector3<T> l0 = _lProx.p0();
        Erl::Vector3<T> v0 = _lProx.p1()-l0;
        Erl::Vector3<T> l1 = _lObj.p0();
        Erl::Vector3<T> v1 = _lObj.p1()-l1;

        if(v0.dot(v1)<T(0.))
        { l0 = l0+v0; v0 = -v0;}

        Erl::Vector3<T> u = l0 - l1;
        T  a = v0.dot(v0);
        T  b = v0.dot(v1);
        T  c = v1.dot(v1);
        T  d = v0.dot(u);
        T  e = v1.dot(u);
        T  det = a * c - b * b;

        T s0 = (b * e - c * d)/det;
        T s1 = (a * e - b * d)/det;

        s0 = Erl::clamp(Erl::clamp(s0,-d/a,(-d+b)/a),T(0.),T(1.));
        s1 = Erl::clamp(Erl::clamp(s1,e/c,(e + b)/c),T(0.),T(1.));

        return ProxyRes<T> (l1 + s1*v1,
                            l0 + s0*v0);

        #endif


}

template<typename T> inline ProxyRes<T> calcP2O(const Segment<T>& _l, const Circle<T>& _con)
{
    // line: Object
    // disc: proxy

    T c_lProxRadius = (_l.p0()-_l.p1()).norm();
    Erl::Vector3<T> c_lProxCentre= (_l.p0()+_l.p1())*T(0.5); // l0+v0*0.5f;
    Erl::Vector3<T> c_lProxDirection = (_l.p1()-_l.p0())/c_lProxRadius;
    c_lProxRadius*=T(.5);

    Erl::Vector3<T> diff = c_lProxCentre - _con.p(); //
    T diffSqrLen = diff.dot(diff);
    T a1  = T(1.); //MdM
    T a0  = diff.dot(c_lProxDirection); //DdM
    T NdM = _con.n().dot(c_lProxDirection);
    T DdN = diff.dot(_con.n());

    //T a0 = DdM;
    //T a1 = MdM;
    T b0 = a0 - NdM*DdN;
    T b1 = a1 - NdM*NdM;
    T c0 = diffSqrLen - DdN*DdN;
    T c1 = b0;
    T c2 = b1;
    T rsqr = _con.r()*_con.r();

    T a0sqr = a0*a0;
    T a1sqr = a1*a1;
    T twoA0A1 = 2.0f*a0*a1;
    T b0sqr = b0*b0;
    T b1Sqr = b1*b1;
    T twoB0B1 = 2.0f*b0*b1;
    T twoC1 = 2.0f*c1;

    // The minimum point B+t*M occurs when t is a root of the quartic
    // equation whose coefficients are defined below.
    //Polynomial1<T> poly(4);
    //T poly[4]; T poly0 = (1.0f/(a0sqr*c0 - b0sqr*rsqr));
    //poly[0] = (twoA0A1*c0 + a0sqr*twoC1 - twoB0B1*rsqr)*poly0;
    //poly[1] = (a1sqr*c0 + twoA0A1*twoC1 + a0sqr*c2 - b1Sqr*rsqr)*poly0;
    //poly[2] = (a1sqr*twoC1 + twoA0A1*c2)*poly0;
    //poly[3] = (a1sqr*c2)*poly0;

    T poly[5];
    T cRoots[4];
    unsigned root_count;
    poly[4] = a1sqr*c2;                                             // x^4
    poly[3] = (a0sqr*c0 - b0sqr*rsqr);                              // x^3
    poly[2] = (twoA0A1*c0 + a0sqr*twoC1 - twoB0B1*rsqr);            // x^2
    poly[1] = (a1sqr*c0 + twoA0A1*twoC1 + a0sqr*c2 - b1Sqr*rsqr);   // x^1
    poly[0] = (a1sqr*twoC1 + twoA0A1*c2);                           // x^0

    if(poly[4]!=0.0)
    {

        poly[4] = 1. / poly[4];
        poly[3] *= poly[4];
        poly[2] *= poly[4];
        poly[1] *= poly[4];
        poly[0] *= poly[4];

        root_count = root_poly4(poly,cRoots);
//        root_count = root_poly(4,poly,cRoots);

//        poly[4] = 1.;// 1.0 / poly[4];
//        T cRoots2[4];
//        unsigned root_count2 = root_poly(4,poly,cRoots2);
//        std::cout<<"# # # # # # # # # # #"<<std::endl;
//        for(unsigned iR(0);iR<root_count;iR++)
//        {
//            std::cout<<"iR: "<<iR<<" | Root: "<<cRoots[iR]<<std::endl;
//        }
//        std::cout<<"# # # # # # # # # # #"<<std::endl;
//        for(unsigned iR(0);iR<root_count2;iR++)
//        {
//            std::cout<<"iR: "<<iR<<" | Root: "<<cRoots2[iR]<<std::endl;
//        }
//        std::cout<<"# # # # # # # # # # #"<<std::endl;

//        T cRoots3[4];
//        unsigned root_count3 = gsl_poly_solve_quartic(poly[0],poly[1],poly[2],poly[3],
//                                                      &cRoots3[0],&cRoots3[1],&cRoots3[2],&cRoots3[3]);

////        T cRoots3[4];
////        unsigned root_count3 = gsl_poly_solve_quartic(poly[0],poly[1],poly[2],poly[3],
////                                                      &cRoots3[3],&cRoots3[2],&cRoots3[1],&cRoots3[0]);
//        for(unsigned iR(0);iR<root_count3;iR++)
//        {
//            std::cout<<"iR: "<<iR<<" | Root: "<<cRoots3[iR]<<std::endl;
//        }
//        std::cout<<"# # # # # # # # # # #"<<std::endl;

    }
    else if(poly[3]!=0.0)
    {
        poly[3] = 1.0 / poly[3];
        poly[2] *= poly[3];
        poly[1] *= poly[3];
        poly[0] *= poly[3];
        root_count = root_poly3(poly,cRoots);

        poly[3] = 1.;
        T cRoots2[4];
        unsigned root_count2 = root_poly(3,poly,cRoots2);

        for(unsigned iR(0);iR<root_count;iR++)
        {
            std::cout<<"iR: "<<iR<<" | Root: "<<cRoots[iR]<<std::endl;
        }
        std::cout<<"# # # # # # # # # # #"<<std::endl;
        for(unsigned iR(0);iR<root_count2;iR++)
        {
            std::cout<<"iR: "<<iR<<" | Root: "<<cRoots2[iR]<<std::endl;
        }
        std::cout<<"# # # # # # # # # # #"<<std::endl;
    }
    else if(poly[2]!=0.0)
    {
        poly[2] = 1.0 / poly[2];
        poly[1] /= poly[3];
        poly[0] /= poly[3];
        root_count = root_poly2(poly,cRoots);

    }
    else if(poly[1]!=0.0)
    {
        root_count= 1;
        cRoots[0] = (-1.0)*poly[0]/poly[1];
    }
    else
    {
        root_count = 0;
    }


    //if(root_count<1)
    //{
    //    printf("ERROR: count roots: %d\n",root_count);
    //}

    T  c_dist_best, c_dist_tmp;
    Erl::Vector3<T> c_pObj_tmp;

    // FOR t=0;
    T currPara = clamp(cRoots[0],-c_lProxRadius,c_lProxRadius);
    c_pObj_tmp = c_lProxCentre + currPara*c_lProxDirection; //P is point on line
    //calc_disc_point(c_pObj_tmp, cD, nD, rD, r_sqDist,  r_pProxy,r_pObj);

    ProxyRes<T> c_ProxyResBest = calcP2O(c_pObj_tmp,_con);
    c_dist_best = c_ProxyResBest.squaredNorm();
    ProxyRes<T> c_ProxyResTmp;


    for (unsigned i = 1; i < root_count; ++i)
    {
        // Compute distance from P(t) to circle.
        currPara = clamp(cRoots[i],-c_lProxRadius,c_lProxRadius);
        c_pObj_tmp = c_lProxCentre + currPara*c_lProxDirection; //P is point on line
        //calc_disc_point(c_pObj_tmp, cD, nD, rD, &c_dist_tmp,&c_pProxy_tmp,&c_pObj_tmp);
        c_ProxyResTmp = calcP2O(c_pObj_tmp,_con);
        c_dist_tmp = c_ProxyResTmp.squaredNorm();

        if (c_dist_tmp< c_dist_best)
        {
            c_dist_best = c_dist_tmp;
            c_ProxyResBest = c_ProxyResTmp;
        }
    }
    return c_ProxyResBest;
}
template<typename T> inline ProxyRes<T> calcP2O(const Contour<T>& _con,const Segment<T>& _l)
{
    return calcP2O(_l,_con).swap();
}
template<typename T> inline ProxyRes<T> calcP2O(const Segment<T>& _l, const Contour<T>& _con)
{
    // line: Object
    // disc: proxy

    T c_lProxRadius = (_l.p0()-_l.p1()).norm();
    Erl::Vector3<T> c_lProxCentre= (_l.p0()+_l.p1())*T(.5); // l0+v0*0.5f;
    Erl::Vector3<T> c_lProxDirection = (_l.p1()-_l.p0())/c_lProxRadius;
    c_lProxRadius*=T(.5);

    Erl::Vector3<T> diff = c_lProxCentre - _con.p(); //
    T diffSqrLen = diff.dot(diff);
    T a1  = T(1.); //MdM
    T a0  = diff.dot(c_lProxDirection); //DdM
    T NdM = _con.n().dot(c_lProxDirection);
    T DdN = diff.dot(_con.n());

    //T a0 = DdM;
    //T a1 = MdM;
    T b0 = a0 - NdM*DdN;
    T b1 = a1 - NdM*NdM;
    T c0 = diffSqrLen - DdN*DdN;
    T c1 = b0;
    T c2 = b1;
    T rsqr = _con.r()*_con.r();

    T a0sqr = a0*a0;
    T a1sqr = a1*a1;
    T twoA0A1 = 2.*a0*a1;
    T b0sqr = b0*b0;
    T b1Sqr = b1*b1;
    T twoB0B1 = 2.*b0*b1;
    T twoC1 = 2.*c1;

    // The minimum point B+t*M occurs when t is a root of the quartic
    // equation whose coefficients are defined below.
    //Polynomial1<T> poly(4);
    //T poly[4]; T poly0 = (1.0f/(a0sqr*c0 - b0sqr*rsqr));
    //poly[0] = (twoA0A1*c0 + a0sqr*twoC1 - twoB0B1*rsqr)*poly0;
    //poly[1] = (a1sqr*c0 + twoA0A1*twoC1 + a0sqr*c2 - b1Sqr*rsqr)*poly0;
    //poly[2] = (a1sqr*twoC1 + twoA0A1*c2)*poly0;
    //poly[3] = (a1sqr*c2)*poly0;

    T poly[5];
    T cRoots[4];
    unsigned root_count;
    poly[4] = a1sqr*c2;                                             // x^4
    poly[3] = (a0sqr*c0 - b0sqr*rsqr);                              // x^3
    poly[2] = (twoA0A1*c0 + a0sqr*twoC1 - twoB0B1*rsqr);            // x^2
    poly[1] = (a1sqr*c0 + twoA0A1*twoC1 + a0sqr*c2 - b1Sqr*rsqr);   // x^1
    poly[0] = (a1sqr*twoC1 + twoA0A1*c2);                           // x^0

    if(poly[4]!=0.0)
    {
        poly[4] = 1. / poly[4];
        poly[3] *= poly[4];
        poly[2] *= poly[4];
        poly[1] *= poly[4];
        poly[0] *= poly[4];
        root_count = root_poly4(poly,cRoots);

    }
    else if(poly[3]!=0.0)
    {
        poly[3] = 1. / poly[3];
        poly[2] *= poly[3];
        poly[1] *= poly[3];
        poly[0] *= poly[3];
        root_count = root_poly3(poly,cRoots);
    }
    else if(poly[2]!=0.0)
    {
        poly[2] = 1. / poly[2];
        poly[1] /= poly[3];
        poly[0] /= poly[3];
        root_count = root_poly2(poly,cRoots);
    }
    else if(poly[1]!=0.0)
    {
        root_count= 1;
        cRoots[0] = (-1.)*poly[0]/poly[1];
    }
    else
    {
        root_count = 0;
    }


    //if(root_count<1)
    //{
    //    printf("ERROR: count roots: %d\n",root_count);
    //}

    T  c_dist_best, c_dist_tmp;
    Erl::Vector3<T> c_pObj_tmp;

    // FOR t=0;
    T currPara = clamp(cRoots[0],-c_lProxRadius,c_lProxRadius);
    c_pObj_tmp = c_lProxCentre + currPara*c_lProxDirection; //P is point on line
    //calc_disc_point(c_pObj_tmp, cD, nD, rD, r_sqDist,  r_pProxy,r_pObj);

    ProxyRes<T> c_ProxyResBest = calcP2O(c_pObj_tmp,_con);
    c_dist_best = c_ProxyResBest.squaredNorm();
    ProxyRes<T> c_ProxyResTmp;


    for (unsigned i = 1; i < root_count; ++i)
    {
        // Compute distance from P(t) to circle.
        currPara = clamp(cRoots[i],-c_lProxRadius,c_lProxRadius);
        c_pObj_tmp = c_lProxCentre + currPara*c_lProxDirection; //P is point on line
        //calc_disc_point(c_pObj_tmp, cD, nD, rD, &c_dist_tmp,&c_pProxy_tmp,&c_pObj_tmp);
        c_ProxyResTmp = calcP2O(c_pObj_tmp,_con);
        c_dist_tmp = c_ProxyResTmp.squaredNorm();

        if (c_dist_tmp< c_dist_best)
        {
            c_dist_best = c_dist_tmp;
            c_ProxyResBest = c_ProxyResTmp;
        }
    }
    return c_ProxyResBest;


}
template<typename T> inline ProxyRes<T> calcP2O(const Line<T>& _l, const Triangle<T>& _tri)
{
    T c_linePara;
    return calcP2O(_l,_tri,c_linePara);
}
template<typename T> inline ProxyRes<T> calcP2O(const Line<T>& _l, const Triangle<T>& _tri, T& _linePara)
{
        // Test if line intersects triangle.  If so, the squared distance is zero.
        Vector3<T> edge0(_tri.p1() - _tri.p0());
        Vector3<T> edge1(_tri.p2() - _tri.p0());
        Vector3<T> normal((edge0.cross(edge1)).normalized());


        T NdD = normal.dot(_l.v()); // v is unit-norm

        if (fabs(NdD) > Constants<T>::Zero_Tolerance)
        {
            // The line and triangle are not parallel, so the line intersects
            // the plane of the triangle.
            Vector3<T> diff = _l.p() - _tri.p0();
            Vector3<T> U, V;
            ComplementOrthoNormalBasis<T>(_l.v(),U,V);
            T UdE0 = U.dot(edge0);
            T UdE1 = U.dot(edge1);
            T UdDiff = U.dot(diff);
            T VdE0 = V.dot(edge0);
            T VdE1 = V.dot(edge1);
            T VdDiff = V.dot(diff);
            T invDet = (T(1)/(UdE0*VdE1 - UdE1*VdE0));

            // Barycentric coordinates for the point of intersection.
            T b1 = (VdE1*UdDiff - UdE1*VdDiff)*invDet;
            T b2 = (UdE0*VdDiff - VdE0*UdDiff)*invDet;
            T b0 = T(1) - b1 - b2;

            if (b0 >= T(0) && b1 >= T(0) && b2 >= T(0))
            {
                // Line parameter for the point of intersection.
                T DdE0 = _l.v().dot(edge0);
                T DdE1 = _l.v().dot(edge1);
                T DdDiff = _l.v().dot(diff);
                _linePara = b1*DdE0 + b2*DdE1 - DdDiff;

                return ProxyRes<T>(_l.p()+_linePara*_l.v(),_tri.p0() + b1*edge0 + b2*edge1);



            }
        }

        // Either (1) the line is not parallel to the triangle and the point of
        // intersection of the line and the plane of the triangle is outside the
        // triangle or (2) the line and triangle are parallel.  Regardless, the
        // closest point on the triangle is on an edge of the triangle.  Compare
        // the line to all three edges of the triangle.

        T c_lineParaTmp, c_SegParaTmp;
        ProxyRes<T> c_ProxyRes    = calcP2O(_l,Segment<T>(_tri[2],_tri[0]),    _linePara,c_SegParaTmp);
        T sqrDist    = c_ProxyRes.squaredNorm();

        ProxyRes<T> c_ProxyResTmp = calcP2O(_l,Segment<T>(_tri[0],_tri[1]),c_lineParaTmp,c_SegParaTmp);
        T sqrDistTmp = c_ProxyResTmp.squaredNorm();
        if (sqrDistTmp < sqrDist)
        {
            c_ProxyRes = c_ProxyResTmp;
            _linePara  = c_lineParaTmp;
            sqrDist = sqrDistTmp;
        }
        c_ProxyResTmp             = calcP2O(_l,Segment<T>(_tri[1],_tri[2]),c_lineParaTmp,c_SegParaTmp);
        sqrDistTmp    = c_ProxyResTmp.squaredNorm();
        if (sqrDistTmp < sqrDist)
        {
            c_ProxyRes = c_ProxyResTmp;
            _linePara  = c_lineParaTmp;
            sqrDist = sqrDistTmp;
        }


//        for (int i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
//        {

//            Segment<T> c_Segment(_tri[i0],_tri[i1]);

//            ProxyRes<T> c_ProxyResTmp = calcP2O(_l,c_Segment,c_lineParaTmp,c_SegParaTmp);
//            T sqrDistTmp = c_ProxyResTmp.squaredNorm();
//            if (sqrDistTmp < sqrDist)
//            {
//                _ProxyRes = c_ProxyResTmp;
//                _linePara = c_lineParaTmp;
//                sqrDist = sqrDistTmp;
//            }
//        }
        return c_ProxyRes;

}
template<typename T> inline ProxyRes<T> calcP2O(const Line<T>& _l, const Segment<T>& _s)
{
    T c_lPara,c_sPara;
    return calcP2O(_l,_s,c_lPara,c_sPara);
}
template<typename T> inline ProxyRes<T> calcP2Oo(const Line<T>& _l, const Segment<T>& _s)
{
    T c_lPara,c_sPara;
    return calcP2Oo(_l,_s,c_lPara,c_sPara);
}
template<typename T> inline ProxyRes<T> calcP2O(const Line<T>& _l, const Segment<T>& _s, T& _lPara, T& _sPara)
{
    Vector3<T> diff = _l.p() - (_s.p0()+_s.p1())*T(.5) ;
    Vector3<T> c_SegDirection = (_s.p1()-_s.p0());
    T c_SegExtent = c_SegDirection.norm();
    c_SegDirection/=c_SegExtent;
    c_SegExtent*=T(.5);

    T a01 = -_l.v().dot(c_SegDirection);
    T b0 = diff.dot(_l.v());

     _sPara = Erl::clamp<T>((a01*b0 + diff.dot(c_SegDirection) )/ std::max<T>(T(1) - a01*a01,Erl::Constants<T>::Zero_Tolerance_Strict),
                            -c_SegExtent,c_SegExtent);
     _lPara = -(a01*_sPara + b0);

    return ProxyRes<T>(_l.p() + _lPara*_l.v(),
                      (_s.p0()+_s.p1())*T(.5) + _sPara*c_SegDirection);


}
template<typename T> inline ProxyRes<T> calcP2Oo(const Line<T>& _l, const Segment<T>& _s, T& _lPara, T& _sPara)
{
    Vector3<T> diff = _l.p() - (_s.p0()+_s.p1())/T(2.) ;

    Vector3<T> c_SegDirection = (_s.p1()-_s.p0());
    T c_SegExtent = c_SegDirection.norm();
    c_SegDirection/=c_SegExtent;
    c_SegExtent*=T(.5);

    T a01 = -_l.v().dot(c_SegDirection);
    T b0 = diff.dot(_l.v());
    //T c = diff.squaredNorm();
    T det = std::abs<T>((T)1 - a01*a01);
    T b1, extDet;

    if (det >= Constants<T>::Zero_Tolerance)
    {
        // The line and segment are not parallel.
        b1 = -diff.dot(c_SegDirection);
        _sPara = a01*b0 - b1;
        extDet = c_SegExtent*det;

        if (_sPara >= -extDet)
        {
            if (_sPara <= extDet)
            {
                // Two interior points are closest, one on the line and one
                // on the segment.
                T invDet = ((T)1)/det;
                _lPara = (a01*b1 - b0)*invDet;
                _sPara *= invDet;

                //T _lPara2 = -(a01*_sPara + b0);
                //invDet =((T)1)/det;;
            }
            else
            {
                // The endpoint e1 of the segment and an interior point of
                // the line are closest.
                _sPara = c_SegExtent;
                _lPara = -(a01*_sPara + b0);
            }
        }
        else
        {
            // The end point e0 of the segment and an interior point of the
            // line are closest.
            _sPara = -c_SegExtent;
            _lPara = -(a01*_sPara + b0);
        }
    }
    else
    {
        // The line and segment are parallel.  Choose the closest pair so that
        // one point is at segment center.
        _sPara = (T)0;
        _lPara = -b0;
    }

    return ProxyRes<T>(_l.p() + _lPara*_l.v(),
                      (_s.p0()+_s.p1())/T(2.) + _sPara*c_SegDirection);


}
template<typename T> inline ProxyRes<T> calcP2O(const Triangle<T>& _tri,const Segment<T>& _s)
{
    return calcP2O(_s,_tri).swap();
}
template<typename T> inline ProxyRes<T> calcP2O(const Segment<T>& _s, const Triangle<T>& _tri)
{
    Erl::Vector3<T> c_Centre =  T(.5)*(_s.p0()+_s.p1());
    Vector3<T> c_SegDirection = (_s.p1()-_s.p0());
    T c_SegExtent = c_SegDirection.norm();
    c_SegDirection/=c_SegExtent;
    c_SegExtent*=T(.5);
    Line<T> c_Line(c_Centre,c_SegDirection);
    T c_SegmentPara(0);


    ProxyRes<T> c_ProxyRes=calcP2O(c_Line,_tri,c_SegmentPara);
    if(std::abs<T>(c_SegmentPara) >c_SegExtent)
    {
        return calcP2O(c_Centre+c_SegDirection*c_SegExtent*Erl::signum(c_SegmentPara),_tri);
    }
    return  c_ProxyRes;
}
template<typename T> inline ProxyRes<T> calcP2Oo(const Segment<T>& _s, const Triangle<T>& _tri)
{
    Line<T> c_Line(T(.5)*(_s.p0()+_s.p1()),(_s.p1()-_s.p0()).normalized());
    T c_SegmentPara(0);

    Vector3<T> c_SegDirection = (_s.p1()-_s.p0());
    T c_SegExtent = c_SegDirection.norm();
    c_SegDirection/=c_SegExtent;
    c_SegExtent*=T(.5);

    ProxyRes<T> c_ProxyRes=calcP2O(c_Line,_tri,c_SegmentPara);

    if(c_SegmentPara >= -c_SegExtent)
    {
        if(c_SegmentPara <= c_SegExtent)
        {
            return c_ProxyRes;
        }
        else
        {
            return calcP2O(_s.p1(),_tri);
        }
    }
    return  calcP2O(_s.p0(),_tri);
}

// *** TRIANGLE TO OBJECT
template<typename T> inline ProxyRes<T> calcP2O(const Contour<T>& _con,const Triangle<T>& _tri){
    return calcP2O(_tri,_con).swap();
}
#define ERL_V0 0
#define ERL_V1 1
#define ERL_V ERL_V1

#if ERL_V==ERL_V1
template<typename T> inline ProxyRes<T> calcP2O(const Triangle<T>& _tri, const Contour<T>& _con)
{
    Vector3<T> e0     = _tri.p1()-_tri.p0();
    Vector3<T> e1     = _tri.p2()-_tri.p0();
    Vector3<T> n_tri  = e0.cross(e1).normalized();

    Vector3<T> n_par = n_tri.dot(_con.n())*_con.n();
    Vector3<T> n_per = n_par-n_tri;

    T c_nper_norm = n_per.norm();

    Vector3<T> pCloseDisc = _con.p()-Erl::signum(n_tri.dot(_tri.p0()-_con.p()))*n_per*(_con.r()/c_nper_norm);

    Vector3<T> diff  = _tri.p0() - pCloseDisc;
    T a00 = e0.dot(e0);
    T a01 = e0.dot(e1);
    T a11 = e1.dot(e1);
    T b0  = diff.dot(e0);
    T b1  = diff.dot(e1);
    T det = std::fabs(a00*a11 - a01*a01);
    T s = (a01*b1 - a11*b0);
    T t = (a01*b0 - a00*b1);

    if( (s<0) || (t<0) || ((s+t)>det) || (c_nper_norm<PQ::Constants<T>::Zero_Tolerance) )
    {
//        ProxyRes<T> c_PlaneRes = calcP2O(pCloseDisc,Plane<T>(_tri.p0(), n_tri));
//        ProxyRes<T> c_SegBest = calcP2O(Segment<T>(_tri.p0(),_tri.p1()),c_PlaneRes.pObj());
//        ProxyRes<T> c_Seg2    = calcP2O(Segment<T>(_tri.p0(),_tri.p2()),c_PlaneRes.pObj());
//        ProxyRes<T> c_Seg3    = calcP2O(Segment<T>(_tri.p1(),_tri.p2()),c_PlaneRes.pObj());
//        T c_sqNormBest =
//         Erl::min_assign(c_SegBest.squaredNorm(),c_Seg2.squaredNorm(),c_SegBest,c_Seg2,c_SegBest);
//         Erl::min_assign(c_sqNormBest           ,c_Seg3.squaredNorm(),c_SegBest,c_Seg3,c_SegBest);
//         c_SegBest.swap();
//         c_SegBest.setProxy(pCloseDisc);
//         return c_SegBest;

                    ProxyRes<T> c_SegBest = calcP2O(Segment<T>(_tri.p0(),_tri.p1()),_con);
                    ProxyRes<T> c_Seg2    = calcP2O(Segment<T>(_tri.p0(),_tri.p2()),_con);
                    ProxyRes<T> c_Seg3    = calcP2O(Segment<T>(_tri.p1(),_tri.p2()),_con);

         //   //ProxyRes<T> c_Seg4 = calcP2O(_con.p(),Plane<T>(_tri.p0(),n_tri));

            T c_sqNormBest =
             Erl::min_assign(c_SegBest.squaredNorm(),c_Seg2.squaredNorm(),c_SegBest,c_Seg2,c_SegBest);
             Erl::min_assign(c_sqNormBest           ,c_Seg3.squaredNorm(),c_SegBest,c_Seg3,c_SegBest);

        //    // Erl::min_assign(c_sqNormBest           ,c_Seg4.squaredNorm(),c_SegBest,c_Seg4,c_SegBest);

         //c_SegBest.swap();
         //c_SegBest.setProxy(pCloseDisc);
         //return c_SegBest;

          return c_SegBest.swap();
    }
    return  calcP2O(pCloseDisc,Plane<T>(_tri.p0(), n_tri));//calcP2O(pCloseDisc,Plane<T>(_tri.p0(), n_tri)).swap();
}
#elif ERL_V==ERL_V0
template<typename T> inline ProxyRes<T> calcP2O(const Triangle<T>& _tri, const Contour<T>& _con)
{


    Vector3<T> e0 = _tri.p1()-_tri.p0();
    Vector3<T> e1 = _tri.p2()-_tri.p0();
    Vector3<T> n_tri  = e0.cross(e1).normalized();



    Vector3<T> n_par = n_tri.dot(_con.n())*_con.n();
    Vector3<T> n_per = n_par-n_tri;

    Vector3<T> pCloseDisc;
    bool c_calcSeg =false;

    T c_nper_norm = n_per.norm();
    if(c_nper_norm<PQ::Constants<T>::Zero_Tolerance)
    {
        c_calcSeg = true;
    }

    if(n_tri.dot(_tri.p0()-_con.p())<T(0.))
    {
        pCloseDisc = _con.p()+n_per*(_con.r()/c_nper_norm);
    }
    else
    {
        pCloseDisc = _con.p()-n_per*(_con.r()/c_nper_norm);
    }


    Vector3<T> diff  = _tri.p0() - pCloseDisc;
    T a00 = e0.dot(e0);
    T a01 = e0.dot(e1);
    T a11 = e1.dot(e1);
    T b0  = diff.dot(e0);
    T b1  = diff.dot(e1);
    T det = std::abs<T>(a00*a11 - a01*a01);
    T s = (a01*b1 - a11*b0);
    T t = (a01*b0 - a00*b1);


    if(!(s<=det && s>=.0 && t<=det && t>=.0 && (s+t)<=det) || c_calcSeg)
    {
            T  c_sqNormBest, c_sqNormTmp;


            ProxyRes<T> c_SegBest = calcP2O(Segment<T>(_tri.p0(),_tri.p1()),_con);
            ProxyRes<T> c_Seg2    = calcP2O(Segment<T>(_tri.p0(),_tri.p2()),_con);
            ProxyRes<T> c_Seg3    = calcP2O(Segment<T>(_tri.p1(),_tri.p2()),_con);

            c_sqNormBest = c_SegBest.squaredNorm();
            c_sqNormTmp  = c_Seg2.squaredNorm();
            if(c_sqNormTmp<c_sqNormBest)
            {
                c_sqNormBest= c_sqNormTmp;
                c_SegBest   = c_Seg2;
            }
            c_sqNormTmp  = c_Seg3.squaredNorm();
            if(c_sqNormTmp<c_sqNormBest)
            {
                c_sqNormBest= c_sqNormTmp;
                c_SegBest   = c_Seg3;
            }
            return c_SegBest;
    }
    return  calcP2O(pCloseDisc,Plane<T>(_tri.p0(), n_tri));


    /*
//    % 1.) Closest point circle to plane
//    % the colsest point on the disc has to be on the circle unless
//    % (a): circle and triangle are parallel
//    % (b): the edges of the triangle are closer to the circle
//    % calculate n_tri, n_disc

    Vector3<T> e0(_tri.p2()-_tri.p1());
    Vector3<T> e1(_tri.p3()-_tri.p1());
    Vector3<T> n_tri(e0.cross(e1).normalized());


    Vector3<T> n_par((n_tri.dot(_con.n()))*(_con.n()));
    Vector3<T> n_per(n_par-n_tri);
    Point<T> pCloseDisc(_con.p()+n_per*(_con.r()/n_per.norm()));
    // Check if the correspoinding closest point on triangle_plane is in
    // the triangle or outside

    //(I.) calc s,t parameter of tri(s,t) = p0+s*edge0+t*edge1
    Vector3<T> diff  = _tri.p1() - pCloseDisc;
    //
        //T c_diffnorm = diff.norm();
    //
    T a00 = e0.dot(e0);
    T a01 = e0.dot(e1);
    T a11 = e1.dot(e1);
    T b0 = diff.dot(e0);
    T b1 = diff.dot(e1);
    T det = abs(a00*a11 - a01*a01);
    T s = (a01*b1 - a11*b0);
    T t = (a01*b0 - a00*b1);

    //(II.) check if 0<=s<=1 and 0<=t<=1 and s+t <=1

    // if it fails -> calculate edges to circle;
    //T min_sqr_dist = CONST<T>::MAX_T;

//    if(s<=det && s>=0 && t<=det && t>=0 && (s+t)<=det)
//    {
//    ProxyRes<T> proxy_res(Point<T>(0,0,0),Point<T>(CONST<T>::MAX_T,CONST<T>::MAX_T,CONST<T>::MAX_T));
//    T tmp_sqr_dist;
//    for (unsigned i0(2), i1(0); i1 < 3; i0 = i1++)
//    {

//            ProxyRes<T> tmp_proxy_res(calcP2O(Line<T> (_tri[i0],_tri[i1]),_con));
//            tmp_sqr_dist = tmp_proxy_res.squaredNorm();
//            if(tmp_sqr_dist<min_sqr_dist)
//            {
//                min_sqr_dist=tmp_sqr_dist;
//                proxy_res = tmp_proxy_res;
//            }
//    }
//    return proxy_res;
//    }
//    else
//    {
//        return calcP2O(pCloseDisc,Plane<T> (_tri.p1(),n_tri));

//    }

    if (s + t <= det)
    {
        if (s < (T)0)
        {
            if (t < (T)0)  // region 4
            {
                if (b0 < (T)0)
                {
                    t = (T)0;
                    if (-b0 >= a00)
                    {
                        s = (T)1;

                    }
                    else
                    {
                        s = -b0/a00;

                    }
                }
                else
                {
                    s = (T)0;
                    if (b1 >= (T)0)
                    {
                        t = (T)0;

                    }
                    else if (-b1 >= a11)
                    {
                        t = (T)1;

                    }
                    else
                    {
                        t = -b1/a11;
                    }
                }
            }
            else  // region 3
            {
                s = (T)0;
                if (b1 >= (T)0)
                {
                    t = (T)0;
                }
                else if (-b1 >= a11)
                {
                    t = (T)1;

                }
                else
                {
                    t = -b1/a11;

                }
            }
        }
        else if (t < (T)0)  // region 5
        {
            t = (T)0;
            if (b0 >= (T)0)
            {
                s = (T)0;

            }
            else if (-b0 >= a00)
            {
                s = (T)1;

            }
            else
            {
                s = -b0/a00;

            }
        }
        else  // region 0
        {
            // minimum at interior point
            T invDet = ((T)1)/det;
            s *= invDet;
            t *= invDet;

        }
    }
    else
    {
        T tmp0, tmp1, numer, denom;

        if (s < (T)0)  // region 2
        {
            tmp0 = a01 + b0;
            tmp1 = a11 + b1;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - ((T)2)*a01 + a11;
                if (numer >= denom)
                {
                    s = (T)1;
                    t = (T)0;

                }
                else
                {
                    s = numer/denom;
                    t = (T)1 - s;

                }
            }
            else
            {
                s = (T)0;
                if (tmp1 <= (T)0)
                {
                    t = (T)1;

                }
                else if (b1 >= (T)0)
                {
                    t = (T)0;

                }
                else
                {
                    t = -b1/a11;

                }
            }
        }
        else if (t < (T)0)  // region 6
        {
            tmp0 = a01 + b1;
            tmp1 = a00 + b0;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - ((T)2)*a01 + a11;
                if (numer >= denom)
                {
                    t = (T)1;
                    s = (T)0;

                }
                else
                {
                    t = numer/denom;
                    s = (T)1 - t;

                }
            }
            else
            {
                t = (T)0;
                if (tmp1 <= (T)0)
                {
                    s = (T)1;

                }
                else if (b0 >= (T)0)
                {
                    s = (T)0;

                }
                else
                {
                    s = -b0/a00;

                }
            }
        }
        else  // region 1
        {
            numer = a11 + b1 - a01 - b0;
            if (numer <= (T)0)
            {
                s = (T)0;
                t = (T)1;

            }
            else
            {
                denom = a00 - ((T)2)*a01 + a11;
                if (numer >= denom)
                {
                    s = (T)1;
                    t = (T)0;

                }
                else
                {
                    s = numer/denom;
                    t = (T)1 - s;

                }
            }
        }
    }

    if(t==T(0))
    {
        if(s==T(1))
        {
            return calcP2O(Segment<T> (_tri.p2(),_tri.p3()),_con);
        }
        return calcP2O(Segment<T> (_tri.p1(),_tri.p2()),_con);

    }

    if(s==T(0))
    {
        if(t==T(1))
        {
           return calcP2O(Segment<T> (_tri.p2(),_tri.p3()),_con);
        }
        return calcP2O(Segment<T> (_tri.p1(),_tri.p3()),_con);
    }

    if( std::abs<T>((s+t)-T(1))<PQ::Constants<T>::Zero_Tolerance)
    {
        return calcP2O(Segment<T> (_tri.p2(),_tri.p3()),_con);
    }

    Point<T> c_PTri = Point<T>(_tri.p1()+ s*e0 + t*e1);
    return calcP2O(c_PTri,_con);
    */


}
#endif
template<typename T> inline ProxyRes<T> calcP2O(const Cylinder<T>& _cyl,const Triangle<T>& _tri)
{ return calcP2O(_tri,_cyl).swap();}
template<typename T> inline ProxyRes<T> calcP2O(const Triangle<T>& _tri, const Cylinder<T>& _cyl)
{
    ProxyRes<T> c_Res = calcP2O(Segment<T>(_cyl.p1(),_cyl.p2()),_tri);
    if      (c_Res.pProxy() == _cyl.p1())
    { return calcP2O(Contour<T>(_cyl.p1(),_cyl.n(),_cyl.r()),_tri);}
    else if (c_Res.pProxy() == _cyl.p2())
    { return calcP2O(Contour<T>(_cyl.p2(),_cyl.n(),_cyl.r()),_tri); }
    c_Res.setProxy(c_Res.pProxy()+c_Res.getVecProxy2Obj().normalized()*_cyl.r());
    return c_Res;
}
template<typename T> inline ProxyRes<T> calcP2O(const CylinderE<T>& _cyl,const Triangle<T>& _tri)
{ return calcP2O(_tri,_cyl).swap();}
template<typename T> inline ProxyRes<T> calcP2O(const Triangle<T>& _tri, const CylinderE<T>& _cyl)
{
    ProxyRes<T> c_Res = calcP2O(Segment<T>(_cyl.p1(),_cyl.p2()),_tri);
    if      (c_Res.pProxy() == _cyl.p1() && (Erl::as_int(_cyl.getCylinderType()) & Erl::as_int(CylinderType::BACK)) )
    { return calcP2O(Contour<T>(_cyl.p1(),_cyl.n(),_cyl.r()),_tri);}
    else if (c_Res.pProxy() == _cyl.p2() && (Erl::as_int(_cyl.getCylinderType()) & Erl::as_int(CylinderType::FRONT)))
    { return calcP2O(Contour<T>(_cyl.p2(),_cyl.n(),_cyl.r()),_tri); }

    c_Res.setProxy(c_Res.pProxy()+c_Res.getVecProxy2Obj().normalized()*_cyl.r());
    return c_Res;
}
template<typename T>
inline ProxyRes<T> calcP2O(const Triangle<T>& _tri, const Pill<T>& _pill)
{
    return calcP2O(_pill,_tri).swap();
}
template<typename T>
inline ProxyRes<T> calcP2O(const Pill<T>& _pill, const Triangle<T>& _tri)
{
    ProxyRes<T> c_Res  =  calcP2O(Segment<T>(_pill.template p<0>(),_pill.template p<1>()),_tri);
    c_Res.setProxy(c_Res.pProxy()+c_Res.getVecProxy2Obj().normalized()*_pill.pill_radius());
   return c_Res;
}
namespace details
{
    template<typename T, typename Seg >
    struct helper_calcP2O
    {
        inline ProxyRes<T> operator()(const Triangle<T>& _tri, const Cylinders<T,Seg>& _cyl) const noexcept
        {
            #if __cplusplus>=201402L
            typedef typename Seg::value_type S_t;
            static constexpr S_t  S = Seg::value;

            T c_SquaredNorm;
            ProxyRes<T> c_Res  = calcP2O(Segment<T>(_cyl.template p<0>(),_cyl.template p<1>()),_tri);
            if(c_Res.pProxy() == _cyl.template p<0>())
            { c_Res = calcP2O(Contour<T>(_cyl.template p<0>(),_cyl.template n<0>(),_cyl.template r<0>()),_tri);}
            else if (c_Res.pProxy() != _cyl.template p<1>())
            { c_Res.setProxy(c_Res.pProxy()+c_Res.getVecProxy2Obj().normalized()*_cyl.template r<0>()); }
            c_SquaredNorm = c_Res.squaredNorm();

            ProxyRes<T> c_Res_tmp; T c_SquaredNorm_tmp;
            Erl::static_for<1,S-1>()([&](auto && i_ic){
                constexpr size_t i = decltype(i_ic)::value;
                c_Res_tmp = calcP2O(Pill<T>(_cyl.template p<i>,_cyl. template p<i+1>,_cyl.template r<i>),_tri);
                c_SquaredNorm_tmp = c_Res_tmp.squaredNorm();
                if(c_SquaredNorm_tmp<c_SquaredNorm)
                { c_SquaredNorm = c_SquaredNorm_tmp;
                  c_Res         = c_Res_tmp;}

            });

            c_Res_tmp  = calcP2O(Segment<T>(_cyl.template p<S-1>(),_cyl.template p<S>()),_tri);

            if(c_Res_tmp.pProxy() == _cyl.template p<S>())
            { c_Res_tmp = calcP2O(Contour<T>(_cyl.template p<S>(),_cyl.template n<S-1>(),_cyl.template r<S-1>()),_tri);}
            else if (c_Res_tmp.pProxy() != _cyl.template p<S-1>())
            { c_Res_tmp.setProxy(c_Res_tmp.pProxy()+c_Res_tmp.getVecProxy2Obj().normalized()*_cyl.template r<S-1>()); }
            c_SquaredNorm_tmp = c_Res_tmp.squaredNorm();

            if(c_SquaredNorm_tmp<c_SquaredNorm)
            { c_SquaredNorm = c_SquaredNorm_tmp;
              c_Res         = c_Res_tmp;}
            return c_Res;
            #else
            return ProxyRes<T>();
            #endif
        }
    };
    template<typename T>
    struct helper_calcP2O<T,Dynamic>
    {
        inline ProxyRes<T> operator()(const Triangle<T>& _tri, const Cylinders<T,Dynamic>& _cyl) const noexcept
        {
            typedef typename Dynamic::value_type S_t;
            S_t  S = _cyl.size();

            T c_SquaredNorm;
            ProxyRes<T> c_Res  = calcP2O(Segment<T>(_cyl.p(0),_cyl.p(1)),_tri);
            if(c_Res.pProxy() == _cyl.p(0))
            { c_Res = calcP2O(Contour<T>(_cyl.p(0),_cyl.n(0),_cyl.r(0)),_tri);}
            else if (c_Res.pProxy() != _cyl.p(1))
            { c_Res.setProxy(c_Res.pProxy()+c_Res.getVecProxy2Obj().normalized()*_cyl.r(0)); }
            c_SquaredNorm = c_Res.squaredNorm();

            ProxyRes<T> c_Res_tmp; T c_SquaredNorm_tmp;
            for(long i(1); i<(S-1);i++)
            {
                c_Res_tmp = calcP2O(Pill<T>(_cyl.p(i),_cyl.p(i+1),_cyl.r(i)),_tri);
                c_SquaredNorm_tmp = c_Res_tmp.squaredNorm();
                if(c_SquaredNorm_tmp<c_SquaredNorm)
                { c_SquaredNorm = c_SquaredNorm_tmp;
                  c_Res         = c_Res_tmp;}

            };

            c_Res_tmp  = calcP2O(Segment<T>(_cyl.p(S-1),_cyl.p(S)),_tri);
            if(c_Res_tmp.pProxy() == _cyl.p(S))
            { c_Res_tmp = calcP2O(Contour<T>(_cyl.p(S),_cyl.n(S-1),_cyl.r(S-1)),_tri);}
            else if (c_Res_tmp.pProxy() != _cyl.p(S-1))
            { c_Res_tmp.setProxy(c_Res_tmp.pProxy()+c_Res_tmp.getVecProxy2Obj().normalized()*_cyl.r(S-1)); }
            c_SquaredNorm_tmp = c_Res_tmp.squaredNorm();

            if(c_SquaredNorm_tmp<c_SquaredNorm)
            { c_SquaredNorm = c_SquaredNorm_tmp;
              c_Res         = c_Res_tmp;}
            return c_Res;
        }
    };
}
template<typename T, typename Seg >
inline ProxyRes<T> calcP2O(const Triangle<T>& _tri, const Cylinders<T,Seg>& _cyl)
{ return details::helper_calcP2O<T,Seg>()(_tri,_cyl); }
template<typename T> inline ProxyRes<T> calcP2O(const Triangle<T>& _tri, const GCylinder<T>& _cyl)
{

//    ATTEMPT 1
    Vector3<T> short_dir(calcP2O(Segment<T>(_cyl.con1().p(),_cyl.con2().p()),_tri).getVecProxy2Obj());
    Vector3<T> short_n0_para(_cyl.con1().n()*(_cyl.con1().n().dot(short_dir)));
    Vector3<T> short_n1_para(_cyl.con2().n()*(_cyl.con2().n().dot(short_dir)));
    Vector3<T> short_n0_perp((short_dir-short_n0_para).normalized());
    Vector3<T> short_n1_perp((short_dir-short_n1_para).normalized());
    Vector3<T> cyl_out_p0(_cyl.con1().p()+short_n0_perp*_cyl.con1().r());
    Vector3<T> cyl_out_p1(_cyl.con2().p()+short_n1_perp*_cyl.con2().r());
    Segment<T> outer_line(cyl_out_p0,cyl_out_p1);

////    ATTEMPT 2
//    Vector3<T> short_dir0(calcP2O(_cyl.con1().p(),_tri).getVecProxy2Obj());
//    Vector3<T> short_dir1(calcP2O(_cyl.con2().p(),_tri).getVecProxy2Obj());
//    Vector3<T> short_n0_para(_cyl.con1().n()*(_cyl.con1().n().dot(short_dir0)));
//    Vector3<T> short_n1_para(_cyl.con2().n()*(_cyl.con2().n().dot(short_dir1)));
//    Vector3<T> short_n0_perp((short_dir0-short_n0_para).normalized());
//    Vector3<T> short_n1_perp((short_dir1-short_n1_para).normalized());
//    Vector3<T> cyl_out_p0(_cyl.con1().p()+short_n0_perp*_cyl.con1().r());
//    Vector3<T> cyl_out_p1(_cyl.con2().p()+short_n1_perp*_cyl.con2().r());
//    Segment<T> outer_line(cyl_out_p0,cyl_out_p1);

//    // ATTEMPT 3
//    //Vector3<T> short_dir0(calcP2O(_cyl.con1().p(),_tri).getVecProxy2Obj());
//    //Vector3<T> short_dir1(calcP2O(_cyl.con2().p(),_tri).getVecProxy2Obj());

//    Vector3<T> short_dir0 = _cyl.con1().p()-_tri.p1();
//    Vector3<T> short_dir1 = _cyl.con2().p()-_tri.p1();

//    Vector3<T> nplane = short_dir0.cross(short_dir1);
//    Vector3<T> gamma0 = nplane.cross(_cyl.con1().n());
//    Vector3<T> gamma1 = nplane.cross(_cyl.con2().n());

//    Vector3<T> p2 = _cyl.con1().p() + _cyl.con1().r()* gamma0.normalized();
//    Vector3<T> p3 = _cyl.con2().p() + _cyl.con2().r()* gamma1.normalized();
//    Segment<T> outer_line(p2,p3);


    ProxyRes<T> proxy_result         = calcP2O(outer_line ,_tri);
    ProxyRes<T> _tmp_proxy_res_con1  = calcP2O(_cyl.con1(),_tri);
    ProxyRes<T> _tmp_proxy_res_con2  = calcP2O(_cyl.con2(),_tri);

    T min_squaredNorm = proxy_result.squaredNorm();
    T tmp_squaredNorm = _tmp_proxy_res_con1.squaredNorm();

    if(tmp_squaredNorm<min_squaredNorm)
    {
        min_squaredNorm = tmp_squaredNorm;
        proxy_result    = _tmp_proxy_res_con1;
    }
    tmp_squaredNorm = _tmp_proxy_res_con2.squaredNorm();
    if(tmp_squaredNorm<min_squaredNorm)
    {
        min_squaredNorm = tmp_squaredNorm;
        proxy_result    = _tmp_proxy_res_con2;
    }

    return proxy_result;

}
template<typename T> inline ProxyRes<T> calcP2O(const Triangle<T>& _tri, const GCylinders<T>& _cyl)
{
    T min_squaredNorm = Constants<T>::Max_Real;
    ProxyRes<T> proxy_result(Point<T>(0,0,0),Point<T>(Constants<T>::Max_Real,Constants<T>::Max_Real,Constants<T>::Max_Real));

    for (unsigned j=_cyl.getSegIdx(0); j<_cyl.getSegIdx(1); j++){


        Vector3<T> short_dir(calcP2O(Segment<T>(_cyl[j].p(),_cyl[j+1].p()),_tri).getVecProxy2Obj());

        Vector3<T> short_n0_para(_cyl[j].n()*(_cyl[j].n()*short_dir));
        Vector3<T> short_n1_para(_cyl[j+1].n()*(_cyl[j+1].n()*short_dir));

        Vector3<T> short_n0_perp((short_dir-short_n0_para).unitvec());
        Vector3<T> short_n1_perp((short_dir-short_n1_para).unitvec());


        Vector3<T> cyl_out_p0(  _cyl[j].p()+short_n0_perp*_cyl[j].r());
        Vector3<T> cyl_out_p1(_cyl[j+1].p()+short_n1_perp*_cyl[j+1].r());

        Segment<T> outer_line(cyl_out_p0,cyl_out_p1);

        ProxyRes<T> _tmp_proxy_res(calcP2O(outer_line,_tri));

        if(j==_cyl.getSegIdx(0) && (_tmp_proxy_res.pObj()-cyl_out_p0).squaredNorm() < Constants<T>::Zero_Tolerance)
        {
            _tmp_proxy_res=calcP2O(_tri,_cyl[j]);
        }
        else if(j==_cyl.getSegIdx(1) && (_tmp_proxy_res.pObj()-cyl_out_p1).squaredNorm() < Constants<T>::Zero_Tolerance){
            _tmp_proxy_res=calcP2O(_tri,_cyl[j+1]);
        }
        T _tmp_shortest_squaredNorm(_tmp_proxy_res.squaredNorm());

        if(_tmp_shortest_squaredNorm<min_squaredNorm)
        {
            min_squaredNorm=_tmp_shortest_squaredNorm;
            proxy_result=_tmp_proxy_res;

        }

    }
    return proxy_result;

}

//******************************************************************************************************************************************************
//squaredNormP2O: calculate the squared distance of PROXY to OBJECT

//*** POINT TO OBJECT **************************************************
template<typename T> inline T squaredNormP2O(const Point<T>& _p1,const  Point<T>& _p2){
    return (_p1-_p2).squaredNorm();
}
template<typename T> inline T squaredNormP2O(const Point<T>& _p,const Sphere<T>& _s){
    return (_p-(_s.p()+(_p-_s.p()).unitvec()*_s.r())).squaredNorm();
}
template<typename T> inline T squaredNormP2O(const Point<T>& _p, const Line<T>& _linf){
    return (_p-(_linf.p()+_linf.v()*(_linf.v().dot(_p-_linf.p()))/_linf.v().squaredNorm())).squaredNorm();
}
template<typename T> inline T squaredNormP2O(const Point<T>& _p,const  Ray<T>& _r){
    return (_p-_r.p()+_r.v()*std::max(T(0.),_r.v().dot(_p-_r.p())/_r.v().squaredNorm())).squaredNorm();
}
template<typename T> inline T squaredNormP2O(const Point<T>& _p,const  Segment<T>& _l){
    Vector3<T> _v(_l[1]-_l[0]);
    return (_p-_l[0]+(_v*Erl::clamp<T>((_v.dot(_p-_l[0]))/_v.squaredNorm(),T(0),T(1)))).squaredNorm();
}
template<typename T> inline T squaredNormP2O(const Point<T>& _p,const  Plane<T>& _plane){
    T dist(_plane.n()*(_p-_plane.p()));
    return dist*dist;
}
template<typename T> inline T squaredNormP2O(const Point<T>& _p, const Triangle<T>& _tri){
    return calcP2O(_p,_tri).dist_squar();
}
template<typename T> inline T squaredNormP2O(const Point<T>& _p, const Contour<T>& _con){
    return calcP2O(_p,_con).dist_squar();
}

//*** LINE TO OBJECT **************************************************

//******************************************************************************************************************************************************
//normP2O: calculate distance of PROXY to OBJECT

//*** POINT TO OBJECT **************************************************
template<typename T> inline T normP2O(const Point<T>& _p1, const Point<T>& _p2){
    return sqrt(squaredNormP2O(_p1,_p2));
}
template<typename T> inline T normP2O(const Point<T>& _p, const Sphere<T>& _s){
    return sqrt(squaredNormP2O(_p,_s));
}
template<typename T> inline T normP2O(const Point<T>& _p, const Line<T>& _linf){
    return sqrt(squaredNormP2O(_p,_linf));
}
template<typename T> inline T normP2O(const Point<T>& _p, const Ray<T>& _r){
    return sqrt(squaredNormP2O(_p,_r));
}
template<typename T> inline T normP2O(const Point<T>& _p, const Segment<T>& _l){
    return sqrt(squaredNormP2O(_p,_l));
}
template<typename T> inline T normP2O(const Point<T>& _p, const Plane<T>& _plane){
    return (_plane.n()*(_p-_plane.p()));
}
template<typename T> inline T normP2O(const Point<T>& _p, const Triangle<T>& _tri){
    return calcP2O(_p,_tri).norm();
}
template<typename T> inline T normP2O(const Point<T>& _p, const Contour<T>& _con){
    return calcP2O(_p,_con).norm();
}

//*** LINE TO OBJECT **************************************************






}




#endif // ERL_PQ_DISTANCE_H
