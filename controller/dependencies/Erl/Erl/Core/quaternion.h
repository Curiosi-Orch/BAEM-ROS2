/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_QUATERNION_H
#define ERL_QUATERNION_H

#include <Eigen/Core>
#include "Erl/_forwards.h"
#include <limits>

namespace Erl
{

typedef Quaternion<double> Quaterniond;
typedef Quaternion<float>  Quaternionf;

template<class T> class
ERL_ALIGNAS_QUATERNION
Quaternion : public Eigen::Matrix< T, 4, 1 >
{

public:

    typedef std::integral_constant<int, 1> FlipFirstQuaternion;
    typedef std::integral_constant<int, 2> FlipSecondQuaternion;

// Essential ======================================================================================================

    typedef Eigen::Matrix< T, 4, 1 > BaseClass;

    inline Quaternion(): BaseClass() {}

    template<typename OtherDerived> inline Quaternion (const Eigen::MatrixBase<OtherDerived>& other)
        : BaseClass(other) {}

    template<typename OtherDerived> inline Quaternion & operator= (const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->BaseClass::operator=(other);
        return *this;
    }

// Constructors ====================================================================================================

    inline Quaternion(T w, T x, T y, T z) : BaseClass(w, x, y, z)    {}

    template <typename OtherType> inline Quaternion (const OtherType *dataIn)
        : BaseClass((T)dataIn[0], (T)dataIn[1], (T)dataIn[2], (T)dataIn[3])
    {}

// From matrix =========================================================================================

#ifdef __GNUC__
    template<typename OtherType, int Rdim, int Cdim> inline Quaternion(const Eigen::Matrix<OtherType, Rdim, Cdim> &M)
        : Quaternion(fromMatrix(M))   {}
#else
    template<typename OtherType, int Rdim, int Cdim> inline Quaternion (const Eigen::Matrix<OtherType, Rdim, Cdim> &M)
    {
        *this = fromMatrix(M);
    }
#endif

    template<typename OtherType, int Rdim, int Cdim> inline Quaternion& operator = (const Eigen::Matrix<OtherType, Rdim, Cdim> &M)
    {
        *this = fromMatrix(M);
        return *this;
    }

// From translation ======================================================================================

    template <typename OtherType> inline Quaternion (const Eigen::Matrix<OtherType, 3, 1> &vector)
        : BaseClass(0, vector.x(), vector.y(), vector.z()) {}

    template <typename OtherType> inline Quaternion& operator = (const Eigen::Matrix<OtherType, 3, 1> &vector)
    {
        this->BaseClass::data()[0] = 0;
        this->BaseClass::data()[1] = vector(0);
        this->BaseClass::data()[2] = vector(1);
        this->BaseClass::data()[3] = vector(2);
        return *this;
    }

// From pointer ======================================================================================================

    template<typename OtherType> inline static Quaternion fromPointer (const OtherType *data)
    {
        Quaternion returnQuaternion;
        returnQuaternion.BaseClass::data()[0] = data[0];
        returnQuaternion.BaseClass::data()[1] = data[1];
        returnQuaternion.BaseClass::data()[2] = data[2];
        returnQuaternion.BaseClass::data()[3] = data[3];
        return returnQuaternion;
    }

// Operators ===========================================================================================

    inline Quaternion operator * (const Quaternion &Q) const
    {
        return Quaternion
        (
            this->BaseClass::data()[0] * Q.data()[0] - this->BaseClass::data()[1] * Q.data()[1] - this->BaseClass::data()[2] * Q.data()[2] - this->BaseClass::data()[3] * Q.data()[3],
            this->BaseClass::data()[0] * Q.data()[1] + this->BaseClass::data()[1] * Q.data()[0] + this->BaseClass::data()[2] * Q.data()[3] - this->BaseClass::data()[3] * Q.data()[2],
            this->BaseClass::data()[0] * Q.data()[2] + this->BaseClass::data()[2] * Q.data()[0] + this->BaseClass::data()[3] * Q.data()[1] - this->BaseClass::data()[1] * Q.data()[3],
            this->BaseClass::data()[0] * Q.data()[3] + this->BaseClass::data()[3] * Q.data()[0] + this->BaseClass::data()[1] * Q.data()[2] - this->BaseClass::data()[2] * Q.data()[1]
        );
    }

    inline Quaternion& operator *= (const Quaternion &Q)
    {
        *this  = (*this)*Q;
        return *this;
    }

    inline Vector3<T> operator * (const Eigen::Matrix<T, 3, 1> &V) const
    {
        return rotateVector(V);
    }

    inline Quaternion operator * (const T &in) const
    {
        return Quaternion(this->BaseClass::data()[0]*in, this->BaseClass::data()[1]*in, this->BaseClass::data()[2]*in, this->BaseClass::data()[3]*in);
    }

    inline Quaternion& operator *= (const T &in)
    {
        this->BaseClass::data()[0] *= in;
        this->BaseClass::data()[1] *= in;
        this->BaseClass::data()[2] *= in;
        this->BaseClass::data()[3] *= in;
        return *this;
    }

    inline friend Quaternion operator * (const T &in, const Quaternion &Q)
    {
        return Quaternion(Q.data()[0]*in, Q.data()[1]*in, Q.data()[2]*in, Q.data()[3]*in);
    }

    inline Quaternion operator / (const T &in) const
    {
        ERL_ASSERT(in != 0 && "Dividing quaternion by zero");
        return Quaternion(this->BaseClass::data()[0]/in, this->BaseClass::data()[1]/in, this->BaseClass::data()[2]/in, this->BaseClass::data()[3]/in);
    }

    inline Quaternion operator /= (const T &in)
    {
        ERL_ASSERT(in != 0 && "Dividing quaternion by zero");
        this->BaseClass::data()[0] /= in;
        this->BaseClass::data()[1] /= in;
        this->BaseClass::data()[2] /= in;
        this->BaseClass::data()[3] /= in;
        return *this;
    }

    inline bool operator == (const Quaternion &Q) const
    {
        return (this->BaseClass::data()[0] == Q.data()[0] && this->BaseClass::data()[1] == Q.data()[1] && this->BaseClass::data()[2] == Q.data()[2] && this->BaseClass::data()[3] == Q.data()[3]);
    }

    inline bool operator != (const Quaternion &Q) const
    {
        return (this->BaseClass::data()[0] != Q.data()[0] || this->BaseClass::data()[1] != Q.data()[1] || this->BaseClass::data()[2] != Q.data()[2] || this->BaseClass::data()[3] != Q.data()[3]);
    }

    inline friend std::ostream& operator << (std::ostream &out, const Quaternion &Q)
    {
        out << Q.data()[0] << " " << Q.data()[1] << " " << Q.data()[2] << " " << Q.data()[3];
        return out;
    }

    inline Quaternion operator + (const Quaternion &Q) const
    {
        return Quaternion(this->BaseClass::data()[0] + Q.data()[0], this->BaseClass::data()[1] + Q.data()[1], this->BaseClass::data()[2] + Q.data()[2], this->BaseClass::data()[3] + Q.data()[3]);
    }

    inline Quaternion operator - (const Quaternion &Q) const
    {
        return Quaternion(this->BaseClass::data()[0] - Q.data()[0], this->BaseClass::data()[1] - Q.data()[1], this->BaseClass::data()[2] - Q.data()[2], this->BaseClass::data()[3] - Q.data()[3]);
    }

    inline Quaternion& operator += (const Quaternion &Q)
    {
        this->BaseClass::data()[0] += Q.data()[0];
        this->BaseClass::data()[1] += Q.data()[1];
        this->BaseClass::data()[2] += Q.data()[2];
        this->BaseClass::data()[3] += Q.data()[3];
        return *this;
    }

    inline Quaternion& operator -= (const Quaternion &Q)
    {
        this->BaseClass::data()[0] -= Q.data()[0];
        this->BaseClass::data()[1] -= Q.data()[1];
        this->BaseClass::data()[2] -= Q.data()[2];
        this->BaseClass::data()[3] -= Q.data()[3];
        return *this;
    }

    inline const Quaternion& operator + () const
    {
        return *this;
    }

    inline Quaternion operator - () const
    {
        return Quaternion(-this->BaseClass::data()[0], -this->BaseClass::data()[1], -this->BaseClass::data()[2], -this->BaseClass::data()[3]);
    }

// Accessors ===========================================================================================
    inline T* data()
    {
        return &this->BaseClass::data()[0];
    }
    inline const T* data() const
    {
        return &this->BaseClass::data()[0];
    }

    inline T& operator () (const int &i)
    {
        ERL_ASSERT( i < 4 && "Quaternion lookup index > 3");
        return this->BaseClass::data()[i];
    }

    inline T operator () (const int &i) const
    {
        ERL_ASSERT( i < 4 && "Quaternion lookup index > 3");
        return this->BaseClass::data()[i];
    }

    inline T w () const
    {
        return this->BaseClass::data()[0];
    }

    inline T x () const
    {
        return this->BaseClass::data()[1];
    }

    inline T y () const
    {
        return this->BaseClass::data()[2];
    }

    inline T z () const
    {
        return this->BaseClass::data()[3];
    }

    inline T& w ()
    {
        return this->BaseClass::data()[0];
    }

    inline T& x ()
    {
        return this->BaseClass::data()[1];
    }

    inline T& y ()
    {
        return this->BaseClass::data()[2];
    }

    inline T& z ()
    {
        return this->BaseClass::data()[3];
    }

// Getters ===========================================================================================

    template<int MajorType = Erl::ColMajor> inline Rotmat<T, MajorType> getMatrix () const
    {
        return Rotmat<T, MajorType>(*this);
    }

    inline Vector3<T> getVector () const
    {
        return Vector3<T>(this->BaseClass::data()[1], this->BaseClass::data()[2], this->BaseClass::data()[3]);
    }
    inline T getAngleOfAngleAxis() const
    {
        #if (ERL_ASSERT_ACTIVE == ERL_TRUE)
        ERL_RMV_UNVAR_SINGLE
        T _zero(std::abs<T>(T(1.)-norm())); ERL_ASSERT ( (_zero <= Erl::Constants<T>::Zero_Tolerance) && ("Quaternion not normalized, cannot transform to angle-axis"));
        #endif
        if (std::abs(this->BaseClass::data()[0]) >= 1)
        {
            return T(0);
        }
        return T(2)*std::acos(this->BaseClass::data()[0]);

    }
    inline void getAngleAxis(T &angle, Eigen::Matrix<T, 3, 1> &axis) const
    {
        #if (ERL_ASSERT_ACTIVE == ERL_TRUE)
            T _zero(std::abs<T>(T(1.)-norm())); ERL_ASSERT ( (_zero <= Erl::Constants<T>::Zero_Tolerance) && ("Quaternion not normalized, cannot transform to angle-axis"));
        #endif
        getAngleAxis_unsafe(angle,axis);
    }

    inline void getAngleAxis_unsafe(T &angle, Eigen::Matrix<T, 3, 1> &axis) const
    {
        if (std::abs(this->BaseClass::data()[0]) >= (T(1.)-Erl::Constants<T>::Zero_Tolerance))
        {
            angle = 0;
            axis.data()[0] = 1;
            axis.data()[1] = 0;
            axis.data()[2] = 0;
        }
        else
        {
            angle = 2.*std::acos(this->BaseClass::data()[0]);
            T temp = 1./std::sqrt(1 - this->BaseClass::data()[0]*this->BaseClass::data()[0]);
            axis.data()[0] = this->BaseClass::data()[1]*temp;
            axis.data()[1] = this->BaseClass::data()[2]*temp;
            axis.data()[2] = this->BaseClass::data()[3]*temp;
        }
    }

    inline Vector3<T> getRotationVector () const
    {
        T c_angle;
        Eigen::Matrix<T, 3, 1> c_axis;
        getAngleAxis(c_angle,c_axis);
        return c_angle*c_axis;
    }
    inline Vector3<T> getRotationVector_unsafe () const
    {
        T c_angle;
        Eigen::Matrix<T, 3, 1> c_axis;
        getAngleAxis_unsafe(c_angle,c_axis);
        return c_angle*c_axis;
    }

// Setters ===========================================================================================

    inline static Quaternion fromAngleAxis (const T &angle, const Eigen::Matrix<T, 3, 1> &axis)
    {
        Vector3<T> temp(axis);
        temp.normalize();
        return Quaternion(cos(angle/2), sin(angle/2)*temp.coeff(0), sin(angle/2)*temp.coeff(1), sin(angle/2)*temp.coeff(2));
    }

    inline static Quaternion fromAngleAxis (const T &angle, const T &x, const T &y, const T &z)
    {
        T vNorm = std::sqrt(x*x + y*y + z*z);
        ERL_ASSERT(vNorm >= T(0) && "Norm of axis vector is zero for angleAxis creation");
        return Quaternion(cos(angle/2), sin(angle/2)*x/vNorm, sin(angle/2)*y/vNorm, sin(angle/2)*z/vNorm);
    }

    inline static Quaternion fromRotationVector(const Eigen::Matrix<T, 3, 1> &axis)
    {
        Vector3<T> temp(axis);
        T angle = temp.norm();
        if(angle<=Erl::Constants<T>::Zero_Tolerance)
            return Quaternion(1,0,0,0);

        temp/=angle;
        return Quaternion(cos(angle/2), sin(angle/2)*temp.coeff(0), sin(angle/2)*temp.coeff(1), sin(angle/2)*temp.coeff(2));

    }

    inline static Quaternion fromRotationVector(const T &x, const T &y, const T &z)
    {
        Vector3<T> temp(x,y,z);
        T angle = temp.norm();
        if(angle<=Erl::Constants<T>::Zero_Tolerance)
            return Quaternion(1,0,0,0);

        temp/=angle;
        return Quaternion(cos(angle/2), sin(angle/2)*temp.coeff(0), sin(angle/2)*temp.coeff(1), sin(angle/2)*temp.coeff(2));

    }

// Functions ===========================================================================================

    template <typename OtherType> inline Quaternion<OtherType> cast () const
    {
        return Quaternion<OtherType>((OtherType)this->BaseClass::data()[0], (OtherType)this->BaseClass::data()[1], (OtherType)this->BaseClass::data()[2], (OtherType)this->BaseClass::data()[3]);
    }

protected:

    template<typename OtherType, int Rdim, int Cdim> inline static Quaternion fromMatrix (const Eigen::Matrix<OtherType, Rdim, Cdim> &mat)
    {
        ERL_ASSERT ((((Rdim == 3) && (Cdim == 3)) || ((Rdim == 3) && (Cdim == 4)) || ((Rdim == 4) && (Cdim == 4))) && "Matrix of incorrect dimensions for conversion to quaternion");

        using std::sqrt;
        Quaternion Q;

        T t = mat.trace();
        if (t > T(0))
        {
            t = sqrt(t + T(1.0));
            Q.data()[0] = T(0.5)*t;
            t = T(.5)/t;
            Q.data()[1] = (mat.coeff(2,1) - mat.coeff(1,2)) * t;
            Q.data()[2] = (mat.coeff(0,2) - mat.coeff(2,0)) * t;
            Q.data()[3] = (mat.coeff(1,0) - mat.coeff(0,1)) * t;
        }
        else
        {
            int i = 0;
            if (mat.coeff(1,1) > mat.coeff(0,0))
                i = 1;
            if (mat.coeff(2,2) > mat.coeff(i,i))
                i = 2;
            int j = (i+1)%3;
            int k = (j+1)%3;

            t = sqrt(mat.coeff(i,i)-mat.coeff(j,j)-mat.coeff(k,k) + T(1.0));
            Q(i+1) = T(.5) * t;
            t = T(.5)/t;
            Q.data()[0] = (mat.coeff(k,j)-mat.coeff(j,k))*t;
            Q(j+1) = (mat.coeff(j,i)+mat.coeff(i,j))*t;
            Q(k+1) = (mat.coeff(k,i)+mat.coeff(i,k))*t;
        }
        return Q;
    }

public:

    inline T squaredNorm () const
    {
        return (this->BaseClass::data()[0]*this->BaseClass::data()[0] + this->BaseClass::data()[1]*this->BaseClass::data()[1] + this->BaseClass::data()[2]*this->BaseClass::data()[2] + this->BaseClass::data()[3]*this->BaseClass::data()[3]);
    }

    inline T norm () const
    {
        return std::sqrt(this->BaseClass::data()[0]*this->BaseClass::data()[0] + this->BaseClass::data()[1]*this->BaseClass::data()[1] + this->BaseClass::data()[2]*this->BaseClass::data()[2] + this->BaseClass::data()[3]*this->BaseClass::data()[3]);
    }

    inline Quaternion normalized () const
    {
        T normValue = norm();
        ERL_ASSERT(normValue != 0 && "Quaternion norm = 0");
        return Quaternion(this->BaseClass::data()[0]/normValue, this->BaseClass::data()[1]/normValue, this->BaseClass::data()[2]/normValue, this->BaseClass::data()[3]/normValue);
    }

    inline void normalize ()
    {
        T normValue = norm();
        ERL_ASSERT(normValue != 0 && "Quaternion norm = 0");
        this->BaseClass::data()[0] /= normValue;
        this->BaseClass::data()[1] /= normValue;
        this->BaseClass::data()[2] /= normValue;
        this->BaseClass::data()[3] /= normValue;
    }

    inline Quaternion conj () const
    {
        return Quaternion(this->BaseClass::data()[0], -this->BaseClass::data()[1], -this->BaseClass::data()[2], -this->BaseClass::data()[3]);
    }

    inline Quaternion inv () const
    {
        T normValue = norm();
        ERL_ASSERT(normValue != 0 && "Quaternion norm = 0");
        return Quaternion(this->BaseClass::data()[0]/normValue, -this->BaseClass::data()[1]/normValue, -this->BaseClass::data()[2]/normValue, -this->BaseClass::data()[3]/normValue);
    }

    inline Vector3<T> rotateVector (const Eigen::Matrix<T, 3, 1> &Va) const
    {
        return (*this*Quaternion(Va)*((*this).conj())).getVector();
    }

    inline T dot (const Quaternion &Q2) const
    {
        return this->BaseClass::data()[0] * Q2.data()[0] + this->BaseClass::data()[1] * Q2.data()[1] + this->BaseClass::data()[2] * Q2.data()[2] + this->BaseClass::data()[3] * Q2.data()[3];
    }

    inline bool hasNaN() const
    {
        return (std::isnan(this->BaseClass::data()[0]) || std::isnan(this->BaseClass::data()[1]) || std::isnan(this->BaseClass::data()[2]) || std::isnan(this->BaseClass::data()[3]));
    }

    inline bool isnormal() const
    {
        return (std::isnormal(this->BaseClass::data()[0]) && std::isnormal(this->BaseClass::data()[1]) && std::isnormal(this->BaseClass::data()[2]) && std::isnormal(this->BaseClass::data()[3]));
    }

    inline bool isfinite() const
    {
        return (std::isfinite(this->BaseClass::data()[0]) && std::isfinite(this->BaseClass::data()[1]) && std::isfinite(this->BaseClass::data()[2]) && std::isfinite(this->BaseClass::data()[3]));
    }

    template <typename FlipIndex = FlipFirstQuaternion>
    inline static Quaternion slerp (const Quaternion &Q1, const Quaternion &Q2, const double t)
    {
        #if (ERL_ASSERT_ACTIVE == ERL_TRUE)
        T na = Q1.squaredNorm();
        T nb = Q2.squaredNorm();
        ERL_RMV_UNVAR_SINGLE
        bool nOK = (std::abs(na-1) < Erl::Constants<T>::Zero_Tolerance) && (std::abs(nb-1) < Erl::Constants<T>::Zero_Tolerance);
        ERL_ASSERT (nOK && "Quaternion norm is too different from 1 for slerp");
        ERL_ASSERT (t >= 0 && t <= 1 && "t outside slerp interpolation boundaries");
        #endif

        Quaternion Q1c = Q1;
        Quaternion Q2c = Q2;

        T cosTheta = Q1c.dot(Q2c);
        if (cosTheta < 0)
            FlipQuaternion_impl<FlipIndex>::run(Q1c, Q2c);

        if (t <= 0)
            return Q1c;
        else if (t >= 1)
            return Q2c;

        if (cosTheta > (T(1.)-Erl::Constants<T>::Zero_Tolerance_Soft))
            return Quaternion((1.0 - t) * Q1c + t * Q2c).normalized();
        else
        {
            T theta = acos(cosTheta);
            return Quaternion((std::sin((1-t)*theta)*Q1c + std::sin(t*theta)*Q2c)/std::sin(theta)).normalized();
        }
    }

    template <typename FlipIndex = FlipFirstQuaternion>
    inline static Quaternion nlerp (const Quaternion &Q1, const Quaternion &Q2, const double t)
    {
        #if (ERL_ASSERT_ACTIVE == ERL_TRUE)
        T na = Q1.squaredNorm();
        T nb = Q2.squaredNorm();
        ERL_RMV_UNVAR_SINGLE
        bool nOK = (std::abs(na-1) < Erl::Constants<T>::Zero_Tolerance) && (std::abs(nb-1) < Erl::Constants<T>::Zero_Tolerance);
        ERL_ASSERT (nOK && "Quaternion norm is too different from 1 for slerp");
        ERL_ASSERT (t >= 0 && t <= 1 && "t outside slerp interpolation boundaries");
        #endif

        Quaternion Q1c = Q1;
        Quaternion Q2c = Q2;

        T cosTheta = Q2c.dot(Q1c);
        if (cosTheta < 0)
            FlipQuaternion_impl<FlipIndex>::run(Q1c, Q2c);

        if (t <= 0)
            return Q1c;
        else if (t >= 1)
            return Q2c;

        return (Q1c * (1.0 - t) + Q2c * t).normalized();
    }

    inline static Quaternion Identity ()
    {
        return Quaternion(1,0,0,0);
    }

    inline static Quaternion Zero ()
    {
        return Quaternion(0,0,0,0);
    }

    inline static Quaternion NaN()
    {
        return Quaternion::Identity()*std::numeric_limits<T>::quiet_NaN();
    }

    inline static Quaternion getRotationfromVec3toVec3(const Vector3<T>& _fromVec3,
                                                       const Vector3<T>& _toVec3)
    {
        Vector3<T> c_TmpFromVec3 = _fromVec3.normalized();
        Vector3<T> c_TmpToVec3   = _toVec3.normalized();
        Vector3<T> c_CrossVec  = c_TmpFromVec3.cross(c_TmpToVec3).normalized();
        T          c_Angle        = std::acos(c_TmpFromVec3.dot(c_TmpToVec3));
        T          c_HalfAngleSin = std::sin (c_Angle/T(2));
        return Quaternion(std::cos(c_Angle/T(2)), c_HalfAngleSin*c_CrossVec.coeff(0), c_HalfAngleSin*c_CrossVec.coeff(1), c_HalfAngleSin*c_CrossVec.coeff(2));
    }

    private:

    template <typename FlipIndex = FlipFirstQuaternion, class DUMMY = void> struct FlipQuaternion_impl
    { inline static void run (Quaternion &Q1, Quaternion &Q2); };

ERL_RMV_UNPARA_BEGIN

    template <class DUMMY> struct FlipQuaternion_impl<FlipFirstQuaternion, DUMMY>
    { inline static void run (Quaternion &Q1, Quaternion &Q2) { Q1 = -Q1; } };

    template <class DUMMY> struct FlipQuaternion_impl<FlipSecondQuaternion, DUMMY>
    { inline static void run (Quaternion &Q1, Quaternion &Q2) { Q2 = -Q2; } };

ERL_RMV_UNPARA_END

};

}

#endif
