/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_VECTOR3_H
#define ERL_VECTOR3_H

#include <Eigen/Geometry>
#include "Erl/_forwards.h"

namespace Erl
{

typedef Vector3<double>    Vector3d;
typedef Vector3<float>     Vector3f;
typedef Vector3<int>       Vector3i;
typedef Vector3<unsigned>  Vector3u;

template<class T>  class
ERL_ALIGNAS_VECTOR3
Vector3 : public Eigen::Matrix< T, 3, 1 >
{

public:

// Essential ======================================================================================================

    typedef Eigen::Matrix< T, 3, 1 > BaseClass;

    inline Vector3(): BaseClass() {}

    template<typename OtherDerived> inline Vector3 (const Eigen::MatrixBase<OtherDerived>& other)
        : BaseClass(other) {}

    template<typename OtherDerived> inline Vector3 & operator= (const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->BaseClass::operator=(other);
        return *this;
    }

// Constructors ====================================================================================================

    inline Vector3(T x, T y, T z) : BaseClass(x, y, z)    {}

    template <typename OtherType> inline Vector3 (const OtherType *dataIn)
        : BaseClass((T)dataIn[0], (T)dataIn[1], (T)dataIn[2])
    {}

// From pointer ======================================================================================================

    template<typename OtherType> inline static Vector3 fromPointer (const OtherType *data)
    {
        Vector3 returnVector3;
        returnVector3.BaseClass::data()[0] = data[0];
        returnVector3.BaseClass::data()[1] = data[1];
        returnVector3.BaseClass::data()[2] = data[2];
        return returnVector3;
    }

// Operators ========================================================================================================

    inline Vector3 operator + ( const BaseClass& vector ) const
    {
        return Vector3(this->BaseClass::operator + (vector));
    }

    inline Vector3 operator - ( const BaseClass& vector ) const
    {
        return Vector3(this->BaseClass::operator - (vector));
    }

    inline Vector3 operator * ( const T &fScalar ) const
    {
        return Vector3(this->BaseClass::operator * (fScalar));
    }

    inline Vector3 operator / ( const T &fScalar ) const
    {
        ERL_ASSERT( fScalar != 0.0 && "Vector3 divided by zero");
        return Vector3(this->BaseClass::operator / (fScalar));
    }

    inline const Vector3& operator + () const
    {
        return *this;
    }

    inline Vector3 operator - () const
    {
        return Vector3(this->BaseClass::operator - ());
    }

    inline friend Vector3 operator * (const T &fScalar, const BaseClass& vector )
    {
        return Vector3(vector*fScalar);
    }

    inline Vector3& operator += ( const BaseClass& vector )
    {
        this->BaseClass::operator += (vector);
        return *this;
    }

    inline Vector3& operator -= ( const BaseClass& vector )
    {
        this->BaseClass::operator -= (vector);
        return *this;
    }

    inline Vector3& operator *= ( const T &fScalar )
    {
        this->BaseClass::operator *= (fScalar);
        return *this;
    }

    inline Vector3& operator /= ( const T &fScalar )
    {
        ERL_ASSERT( fScalar != 0.0 && "Vector3 divided by zero");
        this->BaseClass::operator /= (fScalar);
        return *this;
    }

    inline bool operator == ( const BaseClass& vector ) const
    {
        return this->BaseClass::operator == (vector);
    }

    inline bool operator != ( const BaseClass& vector ) const
    {
        return this->BaseClass::operator != (vector);
    }

    friend std::ostream & operator<< (std::ostream& os, const Vector3 &obj)
    {
        os << obj.coeff(0) << " " << obj.coeff(1) << " " << obj.coeff(2);
        return os;
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

    inline T* getData (T* data) const
    {
        memcpy(data,this->BaseClass::data(),3*sizeof(T));
        return data;
    }

// Functions ========================================================================================================

    inline T squaredNorm () const
    {
        return this->BaseClass::squaredNorm();
    }

    inline T norm () const
    {
        return this->BaseClass::norm();
    }

    inline void normalize ()
    {
        this->BaseClass::normalize();
    }

    inline Vector3 normalized () const
    {
        return this->BaseClass::normalized();
    }

    inline T distanceTo (const BaseClass &V1) const
    {
        return (V1 - *this).norm();
    }

    inline T squaredDistanceTo (const BaseClass &V1) const
    {
        return (V1 - *this).squaredNorm();
    }

    inline Vector3 cross (const BaseClass &V1) const
    {
        return this->BaseClass::cross(V1);
    }

    inline T dot (const BaseClass &V1) const
    {
        return this->BaseClass::dot(V1);
    }

    inline Vector3 cwiseProduct (const BaseClass &V1) const
    {
        return this->BaseClass::cwiseProduct(V1);
    }

    inline bool hasNaN() const
    {
        return (this->unaryExpr([](T e){return std::isnan(e);}).any());
    }

    inline bool isnormal() const
    {
        return (this->unaryExpr([](T e){return std::isnormal(e);}).all());
    }

    inline bool isfinite() const
    {
        return (this->unaryExpr([](T e){return std::isfinite(e);}).all());
    }

    inline static Vector3 minJerkInterpolation (const BaseClass &Vi, const BaseClass &Vf, const double &t)
    {
        ERL_ASSERT (t >= 0 && t <= 1 && "t outside MinimumJerk interpolation boundaries");
        return Vi + (Vf-Vi)*(10*std::pow(t, 3) - 15*std::pow(t, 4) + 6*std::pow(t, 5));
    }

    inline static Vector3 minVelocityInterpolation (const BaseClass &Vi, const BaseClass &Vf, const double &t)
    {
        ERL_ASSERT (t >= 0 && t <= 1 && "t outside MinimumVelocity interpolation boundaries");
        return Vi + (Vf - Vi)*t;
    }

    inline static Vector3 minAccelerationInterpolation (const BaseClass &Vi, const BaseClass &Vf, const double &t)
    {
        ERL_ASSERT (t >= 0 && t <= 1 && "t outside MinimumAcceleration interpolation boundaries");
        return Vi + (Vf - Vi)*(3*std::pow(t, 2) - 2*std::pow(t, 3));
    }

    enum
    {
        MinimumJerk,
        MinimumVelocity,
        MinimumAcceleration
    };

    inline static Vector3 interpolation (const BaseClass &Vi, const BaseClass &Vf, const double &t, const int interpolationType)
    {
        if (interpolationType == MinimumJerk)
        {
            return minJerkInterpolation(Vi, Vf, t);
        }
        else if (interpolationType == MinimumVelocity)
        {
            return minVelocityInterpolation(Vi, Vf, t);
        }
        else if (interpolationType == MinimumAcceleration)
        {
            return minAccelerationInterpolation(Vi, Vf, t);
        }
    }

    inline static Vector3 Zero ()
    {
        return BaseClass::Zero();
    }

    inline static Vector3 NaN ()
    {
        return BaseClass::Ones()*std::numeric_limits<T>::quiet_NaN();
    }

};

}

#endif
