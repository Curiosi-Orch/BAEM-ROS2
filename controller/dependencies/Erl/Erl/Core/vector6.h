/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_VECTOR6_H
#define ERL_VECTOR6_H

#include <Eigen/Core>
#include "Erl/_forwards.h"

namespace Erl
{

typedef Vector6<double>    Vector6d;
typedef Vector6<float>     Vector6f;
typedef Vector6<int>       Vector6i;
typedef Vector6<unsigned>  Vector6u;

template<class T> class
ERL_ALIGNAS_VECTOR6
Vector6 : public Eigen::Matrix< T, 6, 1 >
{

public:

// Essential ======================================================================================================

    typedef Eigen::Matrix< T, 6, 1 > BaseClass;

    inline Vector6(): BaseClass() {}

    template<typename OtherDerived> inline Vector6 (const Eigen::MatrixBase<OtherDerived>& other)
        : BaseClass(other) {}

    template<typename OtherDerived> inline Vector6 & operator= (const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->BaseClass::operator=(other);
        return *this;
    }

// Constructors ====================================================================================================

    inline Vector6 (T x, T y, T z, T a, T b, T c) : BaseClass()
    {
        this->BaseClass::data()[0] = x;
        this->BaseClass::data()[1] = y;
        this->BaseClass::data()[2] = z;
        this->BaseClass::data()[3] = a;
        this->BaseClass::data()[4] = b;
        this->BaseClass::data()[5] = c;
    }

    template <typename OtherType> inline Vector6 (const OtherType *dataIn) : BaseClass()
    {
        this->BaseClass::data()[0] = (T)dataIn[0];
        this->BaseClass::data()[1] = (T)dataIn[1];
        this->BaseClass::data()[2] = (T)dataIn[2];
        this->BaseClass::data()[3] = (T)dataIn[3];
        this->BaseClass::data()[4] = (T)dataIn[4];
        this->BaseClass::data()[5] = (T)dataIn[5];
    }

    template<typename OtherType1, typename OtherType2> inline Vector6 (const Eigen::Matrix<OtherType1,3,1>& V1, const Eigen::Matrix<OtherType2,3,1>& V2)
    {
        this->BaseClass::data()[0] = V1(0);
        this->BaseClass::data()[1] = V1(1);
        this->BaseClass::data()[2] = V1(2);
        this->BaseClass::data()[3] = V2(0);
        this->BaseClass::data()[4] = V2(1);
        this->BaseClass::data()[5] = V2(2);
    }

// From pointer ======================================================================================================

    template <typename OtherType> inline static Vector6 fromPointer(const OtherType *dataIn)
    {
        Vector6 returnVector6;
        returnVector6.BaseClass::data()[0] = T(dataIn[0]);
        returnVector6.BaseClass::data()[1] = T(dataIn[1]);
        returnVector6.BaseClass::data()[2] = T(dataIn[2]);
        returnVector6.BaseClass::data()[3] = T(dataIn[3]);
        returnVector6.BaseClass::data()[4] = T(dataIn[4]);
        returnVector6.BaseClass::data()[5] = T(dataIn[5]);
        return returnVector6;
    }

    template <typename OtherType> inline static Vector6 fromPointer(const OtherType *dataInV1, const OtherType *dataInV2)
    {
        Vector6 returnVector6;
        returnVector6.BaseClass::data()[0] = T(dataInV1[0]);
        returnVector6.BaseClass::data()[1] = T(dataInV1[1]);
        returnVector6.BaseClass::data()[2] = T(dataInV1[2]);
        returnVector6.BaseClass::data()[3] = T(dataInV2[0]);
        returnVector6.BaseClass::data()[4] = T(dataInV2[1]);
        returnVector6.BaseClass::data()[5] = T(dataInV2[2]);
        return returnVector6;
    }

// Accessors ======================================================================================================

    inline T x () const
    {
        return this->BaseClass::data()[0];
    }

    inline T y () const
    {
        return this->BaseClass::data()[1];
    }

    inline T z () const
    {
        return this->BaseClass::data()[2];
    }

    inline T a () const
    {
        return this->BaseClass::data()[3];
    }

    inline T b () const
    {
        return this->BaseClass::data()[4];
    }

    inline T c () const
    {
        return this->BaseClass::data()[5];
    }

    inline T& x ()
    {
        return this->BaseClass::data()[0];
    }

    inline T& y ()
    {
        return this->BaseClass::data()[1];
    }

    inline T& z ()
    {
        return this->BaseClass::data()[2];
    }

    inline T& a ()
    {
        return this->BaseClass::data()[3];
    }

    inline T& b ()
    {
        return this->BaseClass::data()[4];
    }

    inline T& c ()
    {
        return this->BaseClass::data()[5];
    }

// Operators ========================================================================================================

    friend std::ostream & operator<< (std::ostream& os, const Vector6 &obj)
    {
        os << obj.coeff(0) << " " << obj.coeff(1) << " " << obj.coeff(2) << " " << obj.coeff(3) << " " << obj.coeff(4) << " " << obj.coeff(5);
        return os;
    }

// Getters ========================================================================================================

    inline Vector3<T> getVector1 () const
    {
        return Vector3<T>(this->BaseClass::data()[0], this->BaseClass::data()[1], this->BaseClass::data()[2]);
    }

    inline Vector3<T> getVector2 () const
    {
        return Vector3<T>(this->BaseClass::data()[3], this->BaseClass::data()[4], this->BaseClass::data()[5]);
    }

// Setters ========================================================================================================

    template<typename OtherType1> inline void setVector1 (const Eigen::Matrix<OtherType1,3,1>& V1)
    {
        this->BaseClass::data()[0] = V1(0);
        this->BaseClass::data()[1] = V1(1);
        this->BaseClass::data()[2] = V1(2);
    }

    template<typename OtherType1> inline void setVector2 (const Eigen::Matrix<OtherType1,3,1>& V2)
    {
        this->BaseClass::data()[3] = V2(0);
        this->BaseClass::data()[4] = V2(1);
        this->BaseClass::data()[5] = V2(2);
    }

// Functions ========================================================================================================

    inline static Vector6 Zero ()
    {
        return BaseClass::Zero();
    }

    inline Vector6 cwiseProduct (const BaseClass &V1) const
    {
        return this->BaseClass::cwiseProduct(V1);
    }

};

}


#endif
