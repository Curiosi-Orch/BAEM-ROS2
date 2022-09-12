/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

/// Core class for rotation matrices. Do not include manually.
#ifndef ERL_ROTMAT_H
    #error "Do not include _rotmatCore.h -> include rotmat.h instead"
#endif

#if (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_GENERAL)

#define ERL_TEMPLATE_ARGS <class T, int MajorType>
#define ERL_MAJOR_TYPE_VALUE MajorType
#define ERL_CLASS_ARGS

#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_COL)

#define ERL_TEMPLATE_ARGS <class T>
#define ERL_MAJOR_TYPE_VALUE ColMajor
#define ERL_CLASS_ARGS <T, ERL_MAJOR_TYPE_VALUE>

#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_ROW)

#define ERL_TEMPLATE_ARGS <class T>
#define ERL_MAJOR_TYPE_VALUE RowMajor
#define ERL_CLASS_ARGS <T, ERL_MAJOR_TYPE_VALUE>

#endif

#include <iostream>
#include <Erl/Utility/util.h>

namespace Erl
{

template ERL_TEMPLATE_ARGS class
ERL_ALIGNAS_ROTMAT
Rotmat ERL_CLASS_ARGS : public Eigen::Matrix< T, 3, 3, ERL_MAJOR_TYPE_VALUE >
{

public:

// Essential =================================================================================================

    typedef Eigen::Matrix< T, 3, 3, ERL_MAJOR_TYPE_VALUE > BaseClass;

    inline Rotmat (): BaseClass() {}

    template<typename OtherType> inline Rotmat (const Eigen::MatrixBase<OtherType>& other)
        : BaseClass(other) {}

    template<typename OtherType>
    inline Rotmat & operator = (const Eigen::MatrixBase <OtherType>& other)
    {
        this->BaseClass::operator=(other);
        return *this;
    }

// Constructors ===============================================================================================

#if (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_GENERAL)
    inline Rotmat (const T& _00, const T& _01, const T& _02, const T& _10, const T& _11, const T& _12, const T& _20, const T& _21, const T& _22);
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_COL)
    inline Rotmat (const T& _00, const T& _01, const T& _02, const T& _10, const T& _11, const T& _12, const T& _20, const T& _21, const T& _22)
    {
        this->BaseClass::data()[0] = _00;
        this->BaseClass::data()[3] = _01;
        this->BaseClass::data()[6] = _02;

        this->BaseClass::data()[1] = _10;
        this->BaseClass::data()[4] = _11;
        this->BaseClass::data()[7] = _12;

        this->BaseClass::data()[2] = _20;
        this->BaseClass::data()[5] = _21;
        this->BaseClass::data()[8] = _22;
    }
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_ROW)
    inline Rotmat (const T& _00, const T& _01, const T& _02, const T& _10, const T& _11, const T& _12, const T& _20, const T& _21, const T& _22)
    {
        this->BaseClass::data()[0] = _00;
        this->BaseClass::data()[1] = _01;
        this->BaseClass::data()[2] = _02;

        this->BaseClass::data()[3] = _10;
        this->BaseClass::data()[4] = _11;
        this->BaseClass::data()[5] = _12;

        this->BaseClass::data()[6] = _20;
        this->BaseClass::data()[7] = _21;
        this->BaseClass::data()[8] = _22;
    }
#endif

    template<typename OtherType> inline Rotmat (const Quaternion<OtherType> &quaternion)
        : BaseClass(fromQuaternion(quaternion)) {}

    template<typename OtherType> inline Rotmat & operator = (const Quaternion<OtherType> &quaternion)
    {
        this->operator=(fromQuaternion(quaternion));
        return *this;
    }

//  Static constructors from pointer ========================================================================

#if (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_GENERAL)
    template<typename OtherType> inline static Rotmat fromPointer (OtherType *data, int dataMajorType);
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_COL)
    template<typename OtherType> inline static Rotmat fromPointer (OtherType *data, int dataMajorType)
    {
        Rotmat returnRot;
        if (dataMajorType == Erl::ColMajor)
        {
            for (int i = 0; i < 9; i++)
                returnRot.data()[i] = data[i];
        }
        else
        {
            returnRot.data()[0] = data[0];
            returnRot.data()[1] = data[3];
            returnRot.data()[2] = data[6];

            returnRot.data()[3] = data[1];
            returnRot.data()[4] = data[4];
            returnRot.data()[5] = data[7];

            returnRot.data()[6] = data[2];
            returnRot.data()[7] = data[5];
            returnRot.data()[8] = data[8];
        }
        return returnRot;
    }
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_ROW)
    template<typename OtherType> inline static Rotmat fromPointer (OtherType *data, int dataMajorType)
    {
        Rotmat returnRot;
        if (dataMajorType == Erl::ColMajor)
        {
            returnRot.data()[0] = data[0];
            returnRot.data()[1] = data[3];
            returnRot.data()[2] = data[6];

            returnRot.data()[3] = data[1];
            returnRot.data()[4] = data[4];
            returnRot.data()[5] = data[7];

            returnRot.data()[6] = data[2];
            returnRot.data()[7] = data[5];
            returnRot.data()[8] = data[8];
        }
        else
        {
            for (int i = 0; i < 9; i++)
                returnRot.data()[i] = data[i];
        }
        return returnRot;
    }
#endif

// Operators =======================================================================================================

    inline Rotmat operator * (const T &scalar) const
    {
        return Rotmat(this->BaseClass::operator*(scalar));
    }

    inline Rotmat operator / (const T &scalar) const
    {
        return Rotmat(this->BaseClass::operator/(scalar));
    }

    inline Rotmat & operator *= (const T &scalar)
    {
        *this = Rotmat(this->BaseClass::operator*(scalar));
        return *this;
    }

    inline Rotmat & operator /= (const T &scalar)
    {
        *this = Rotmat(this->BaseClass::operator/(scalar));
        return *this;
    }

    inline friend Rotmat operator * (const T &scalar, const Rotmat Rot)
    {
        return Rotmat(Rot*scalar);
    }

    template<typename OtherType> inline Vector3<T> operator * (const Eigen::Matrix< OtherType, 3, 1 > vector) const
    {
        return Vector3<T> (this->BaseClass::operator*(vector));
    }

    template<typename OtherType> inline Rotmat operator * (const Eigen::Matrix< OtherType, 3, 3 > rotation) const
    {
        return Rotmat (this->BaseClass::operator*(rotation));
    }

    template<typename OtherType> inline Rotmat operator * (const Eigen::DiagonalMatrix< OtherType, 3, 3 > rotation) const
    {
        return Rotmat (this->BaseClass::operator*(rotation));
    }

    inline T operator () (const unsigned &r, const unsigned &c) const
    {
        return this->BaseClass::operator()(r, c);
    }

    inline T& operator () (const unsigned &r, const unsigned &c)
    {
        return this->BaseClass::operator()(r, c);
    }


// Getters =======================================================================================================

    inline Quaternion<T> getQuaternion () const
    {
        return Quaternion<T>(*this);
    }

    inline Vector3<T> getColumn0 () const
    {
        return Vector3<T>(this->BaseClass::coeff(0,0), this->BaseClass::coeff(1,0), this->BaseClass::coeff(2,0));
    }

    inline Vector3<T> getColumn1 () const
    {
        return Vector3<T>(this->BaseClass::coeff(0,1), this->BaseClass::coeff(1,1), this->BaseClass::coeff(2,1));
    }

    inline Vector3<T> getColumn2 () const
    {
        return Vector3<T>(this->BaseClass::coeff(0,2), this->BaseClass::coeff(1,2), this->BaseClass::coeff(2,2));
    }

    inline Vector3<T> getRPY () const
    {
        return getEuler_ZYX();
    }

    inline Vector3<T> getEuler_XYZ () const
    {
        using std::atan2;
        using std::cos;
        using std::sin;
        Vector3<T> res;

        if (std::abs(this->BaseClass::coeff(2,2)) < T(2)*std::numeric_limits<T>::epsilon() && std::abs(this->BaseClass::coeff(1,2)) < T(2)*std::numeric_limits<T>::epsilon())
        {
            res.data()[0] = T(0);
            res.data()[1] = atan2(this->BaseClass::coeff(0,2), this->BaseClass::coeff(2,2));
            res.data()[2] = atan2(this->BaseClass::coeff(1,0), this->BaseClass::coeff(1,1));
        }
        else
        {
            res.data()[0] = atan2(-this->BaseClass::coeff(1,2), this->BaseClass::coeff(2,2));
            T sr = sin(res.data()[0]);
            T cr = cos(res.data()[0]);
            res.data()[1] = atan2(this->BaseClass::coeff(0,2), cr * this->BaseClass::coeff(2,2) - sr *this->BaseClass::coeff(1,2));
            res.data()[2] = atan2(-this->BaseClass::coeff(0,1), this->BaseClass::coeff(0,0));
        }
        return res;
    }

    inline Vector3<T> getEuler_ZYX () const
    {
        using std::atan2;
        using std::cos;
        using std::sin;
        Vector3<T> res;

        if (std::abs(this->BaseClass::coeff(0,0)) < T(2)*std::numeric_limits<T>::epsilon() && std::abs(this->BaseClass::coeff(1,0)) < T(2)*std::numeric_limits<T>::epsilon())
        {
            res.data()[0] = T(0);
            res.data()[1] = atan2(-this->BaseClass::coeff(2,0), this->BaseClass::coeff(0,0));
            res.data()[2] = atan2(-this->BaseClass::coeff(1,2), this->BaseClass::coeff(1,1));
        }
        else
        {
            res.data()[0] = atan2(this->BaseClass::coeff(1,0), this->BaseClass::coeff(0,0));
            T sp = sin(res.data()[0]);
            T cp = cos(res.data()[0]);
            res.data()[1] = atan2(-this->BaseClass::coeff(2,0), cp * this->BaseClass::coeff(0,0) + sp *this->BaseClass::coeff(1,0));
            res.data()[2] = atan2(sp * this->BaseClass::coeff(0,2) - cp * this->BaseClass::coeff(1,2), cp*this->BaseClass::coeff(1,1) - sp*this->BaseClass::coeff(0,1));
        }
        return res;
    }

    /// Implemented from Eigen 3.2.2
    inline Vector3<T> getEuler (const Axis &axis1, const Axis &axis2, const Axis &axis3) const
    {
        using std::atan2;
        using std::sin;
        using std::cos;

        Vector3<T> res;
        typedef Eigen::Matrix<T,2,1> _Vector2;

        const unsigned odd = (((unsigned)axis1+1)%3 == (unsigned)axis2) ? 0 : 1;
        const unsigned i = (unsigned)axis1;
        const unsigned j = ((unsigned)axis1 + 1 + odd)%3;
        const unsigned k = ((unsigned)axis1 + 2 - odd)%3;

        if (axis1==axis3)
        {
            res[0] = atan2(this->BaseClass::coeff(j,i), this->BaseClass::coeff(k,i));
            if((odd && res[0]<T(0)) || ((!odd) && res[0]>T(0)))
            {
                res[0] = (res[0] > T(0)) ? res[0] - T(M_PI) : res[0] + T(M_PI);
                T s2 = _Vector2(this->BaseClass::coeff(j,i), this->BaseClass::coeff(k,i)).norm();
                res[1] = -atan2(s2, this->BaseClass::coeff(i,i));
            }
            else
            {
                T s2 = _Vector2(this->BaseClass::coeff(j,i), this->BaseClass::coeff(k,i)).norm();
                res[1] = atan2(s2, this->BaseClass::coeff(i,i));
            }

            T s1 = sin(res[0]);
            T c1 = cos(res[0]);
            res[2] = atan2(c1*this->BaseClass::coeff(j,k)-s1*this->BaseClass::coeff(k,k), c1*this->BaseClass::coeff(j,j) - s1 * this->BaseClass::coeff(k,j));
        }
        else
        {
            res[0] = atan2(this->BaseClass::coeff(j,k), this->BaseClass::coeff(k,k));
            T c2 = _Vector2(this->BaseClass::coeff(i,i), this->BaseClass::coeff(i,j)).norm();
            if((odd && res[0]<T(0)) || ((!odd) && res[0]>T(0))) {
                res[0] = (res[0] > T(0)) ? res[0] - T(M_PI) : res[0] + T(M_PI);
                res[1] = atan2(-this->BaseClass::coeff(i,k), -c2);
            }
            else
                res[1] = atan2(-this->BaseClass::coeff(i,k), c2);
            T s1 = sin(res[0]);
            T c1 = cos(res[0]);
            res[2] = atan2(s1*this->BaseClass::coeff(k,i)-c1*this->BaseClass::coeff(j,i), c1*this->BaseClass::coeff(j,j) - s1 * this->BaseClass::coeff(k,j));
        }
        if (!odd)
            res = -res;

        return res;
    }

    inline void getAngleAxis(T &angle, Eigen::Matrix<T, 3, 1> &axis) const
    {
        getQuaternion().getAngleAxis(angle, axis);
    }

    inline void setIdentity()
    {
        (*this) = BaseClass::Identity();
    }

    inline bool hasNaN() const
    {
        return this->unaryExpr([](T e){return std::isnan(e);}).any();
    }

    inline bool isnormal() const
    {
        return (this->unaryExpr([](T e){return std::isnormal(e);}).all());
    }

    inline bool isfinite() const
    {
        return (this->unaryExpr([](T e){return std::isfinite(e);}).all());
    }

// Setters =======================================================================================================

#if (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_GENERAL)
    template <typename OtherType> inline void setColumn0 (const Eigen::Matrix<OtherType,3,1> &vector);
    template <typename OtherType> inline void setColumn1 (const Eigen::Matrix<OtherType,3,1> &vector);
    template <typename OtherType> inline void setColumn2 (const Eigen::Matrix<OtherType,3,1> &vector);
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_COL)
    template <typename OtherType> inline void setColumn0 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        this->BaseClass::data()[0] = vector(0);
        this->BaseClass::data()[1] = vector(1);
        this->BaseClass::data()[2] = vector(2);
    }
    template <typename OtherType> inline void setColumn1 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        this->BaseClass::data()[3] = vector(0);
        this->BaseClass::data()[4] = vector(1);
        this->BaseClass::data()[5] = vector(2);
    }
    template <typename OtherType> inline void setColumn2 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        this->BaseClass::data()[6] = vector(0);
        this->BaseClass::data()[7] = vector(1);
        this->BaseClass::data()[8] = vector(2);
    }
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_ROW)
    template <typename OtherType> inline void setColumn0 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        this->BaseClass::data()[0] = vector(0);
        this->BaseClass::data()[3] = vector(1);
        this->BaseClass::data()[6] = vector(2);
    }
    template <typename OtherType> inline void setColumn1 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        this->BaseClass::data()[1] = vector(0);
        this->BaseClass::data()[4] = vector(1);
        this->BaseClass::data()[7] = vector(2);
    }
    template <typename OtherType> inline void setColumn2 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        this->BaseClass::data()[2] = vector(0);
        this->BaseClass::data()[5] = vector(1);
        this->BaseClass::data()[8] = vector(2);
    }
#endif

    inline static Rotmat fromAngleAxis (const T &angle, const Eigen::Matrix<T, 3, 1> &axis)
    {
        return Rotmat(Quaternion<T>::fromAngleAxis(angle, axis));
    }

    inline static Rotmat fromAngleAxis (const T &angle, const T &x, const T &y, const T &z)
    {
        return Rotmat(Quaternion<T>::fromAngleAxis(angle, x, y, z));
    }

    /// Roll: X, Pitch: Y, Yaw: Z.
    /// R-P-Y corresponds to Rx, then Ry, then Rz in the global frame => Rz*Ry*Rx
    /// Euler ABC corresponds to Ra, then Rb, then Rc in the local frame => Ra*Rb*Rc
    /// Hence Euler ZYX => Rz*Ry*Rx => R-P-Y

    inline static Rotmat fromRPY (const T &r, const T &p, const T &y)
    {
        return fromEuler_ZYX(r, p, y);
    }

    inline static Rotmat fromEuler_ZYX (const T &a, const T &b, const T &c)
    {
        using std::sin;
        using std::cos;
        return Rotmat(cos(b)*cos(a), cos(a)*sin(b)*sin(c) - cos(c)*sin(a), sin(a)*sin(c) + cos(a)*cos(c)*sin(b),
                      cos(b)*sin(a), cos(a)*cos(c) + sin(b)*sin(a)*sin(c), cos(c)*sin(b)*sin(a) - cos(a)*sin(c),
                            -sin(b),                        cos(b)*sin(c),                        cos(b)*cos(c));
    }

    inline static Rotmat fromEuler_XYZ (const T &a, const T &b, const T &c)
    {
        using std::sin;
        using std::cos;
        std::cout << a << " " << b << " " << c << " ";
        std::cout << cos(b) << " " << cos(c) << " " << sin(c) << std::endl;
        return Rotmat(cos(b)*cos(c),                                              -cos(b)*sin(c),         sin(b),
                      cos(a)*sin(c) + cos(c)*sin(b)*sin(a), cos(a)*cos(c) - sin(b)*sin(a)*sin(c), -cos(b)*sin(a),
                      sin(a)*sin(c) - cos(a)*cos(c)*sin(b), cos(c)*sin(a) + cos(a)*sin(b)*sin(c),  cos(b)*cos(a));

    }

    inline static Rotmat fromEuler (const T &a, const T &b, const T &c, const Axis &axis1, const Axis &axis2, const Axis &axis3)
    {
        Quaternion<T> Q1(Quaternion<T>::fromAngleAxis(a, _getCorrespondingAxis(axis1)));
        Quaternion<T> Q2(Quaternion<T>::fromAngleAxis(b, _getCorrespondingAxis(axis2)));
        Quaternion<T> Q3(Quaternion<T>::fromAngleAxis(c, _getCorrespondingAxis(axis3)));

        return Rotmat(Q1*Q2*Q3);
    }

// Functions =======================================================================================================

protected:

    inline static Eigen::Matrix<T, 3, 1> _getCorrespondingAxis (const Axis &axis)
    {
        switch (axis)
        {
        case Axis::X:
            return Eigen::Matrix<T, 3, 1>(1,0,0);
            break;
        case Axis::Y:
            return Eigen::Matrix<T, 3, 1>(0,1,0);
            break;
        case Axis::Z:
            return Eigen::Matrix<T, 3, 1>(0,0,1);
            break;
        default:
            return Eigen::Matrix<T, 3, 1>(0,0,0);
        }
    }

#if (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_COL)
    template<typename OtherType> inline Rotmat fromQuaternion (const Quaternion<OtherType> &Q) const
    {
        Rotmat<T> mat;

        mat.data()[0] = Q.w()*Q.w() + Q.x()*Q.x() - Q.y()*Q.y() - Q.z()*Q.z();
        mat.data()[3] = 2*(Q.x()*Q.y() - Q.w()*Q.z());
        mat.data()[6] = 2*(Q.x()*Q.z() + Q.w()*Q.y());

        mat.data()[1] = 2*(Q.x()*Q.y() + Q.w()*Q.z());
        mat.data()[4] = Q.w()*Q.w() - Q.x()*Q.x() + Q.y()*Q.y() - Q.z()*Q.z();
        mat.data()[7] = 2*(Q.y()*Q.z() - Q.w()*Q.x());

        mat.data()[2] = 2*(Q.x()*Q.z() - Q.w()*Q.y());
        mat.data()[5] = 2*(Q.y()*Q.z() + Q.w()*Q.x());
        mat.data()[8] = Q.w()*Q.w() - Q.x()*Q.x() - Q.y()*Q.y() + Q.z()*Q.z();

        return mat;
    }
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_ROW)
    template<typename OtherType> inline Rotmat fromQuaternion (const Quaternion<OtherType> &Q) const
    {
        Rotmat<T> mat;

        mat.data()[0] = Q.w()*Q.w() + Q.x()*Q.x() - Q.y()*Q.y() - Q.z()*Q.z();
        mat.data()[1] = 2*(Q.x()*Q.y() - Q.w()*Q.z());
        mat.data()[2] = 2*(Q.x()*Q.z() + Q.w()*Q.y());

        mat.data()[3] = 2*(Q.x()*Q.y() + Q.w()*Q.z());
        mat.data()[4] = Q.w()*Q.w() - Q.x()*Q.x() + Q.y()*Q.y() - Q.z()*Q.z();
        mat.data()[5] = 2*(Q.y()*Q.z() - Q.w()*Q.x());

        mat.data()[6] = 2*(Q.x()*Q.z() - Q.w()*Q.y());
        mat.data()[7] = 2*(Q.y()*Q.z() + Q.w()*Q.x());
        mat.data()[8] = Q.w()*Q.w() - Q.x()*Q.x() - Q.y()*Q.y() + Q.z()*Q.z();

        return mat;
    }
#endif

public:

    inline Rotmat inv () const
    {
        return this->BaseClass::transpose();  /// Assumes a true (normalized) rotation matrix
    }

    inline Rotmat transpose () const
    {
        return this->BaseClass::transpose();
    }

    inline static Rotmat Identity ()
    {
        return BaseClass::Identity();
    }

    inline static Rotmat Zero ()
    {
        return BaseClass::Zero();
    }

    inline static Rotmat NaN ()
    {
        return BaseClass::Identity()*std::numeric_limits<T>::quiet_NaN();
    }

    /// Default names refer to global rotations
    inline static Rotmat rotx (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat(1,          0,          0,
                      0, cos(angle), -sin(angle),
                      0, sin(angle),  cos(angle));
    }

    /// Default names refer to global rotations
    inline static Rotmat roty (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat(cos(angle), 0, sin(angle),
                               0, 1,          0,
                     -sin(angle), 0, cos(angle));
    }

    /// Default names refer to global rotations
    inline static Rotmat rotz (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat(cos(angle), -sin(angle), 0,
                      sin(angle),  cos(angle), 0,
                               0,           0, 1);
    }

    inline static Rotmat rotx_global (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat(1,          0,          0,
                      0, cos(angle), -sin(angle),
                      0, sin(angle),  cos(angle));
    }

    inline static Rotmat roty_global (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat(cos(angle), 0, sin(angle),
                               0, 1,          0,
                     -sin(angle), 0, cos(angle));
    }

    inline static Rotmat rotz_global (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat(cos(angle), -sin(angle), 0,
                      sin(angle),  cos(angle), 0,
                               0,           0, 1);
    }

    inline static Rotmat rotx_local (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat(1,          0,          0,
                      0,  cos(angle), sin(angle),
                      0, -sin(angle), cos(angle));
    }

    inline static Rotmat roty_local (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat(cos(angle), 0, -sin(angle),
                               0, 1,          0,
                      sin(angle), 0,  cos(angle));
    }

    inline static Rotmat rotz_local (const T &angle)
    {
        using std::cos;
        using std::sin;
        return Rotmat( cos(angle), sin(angle), 0,
                      -sin(angle), cos(angle), 0,
                               0,           0, 1);
    }

    inline static bool verifyOrthogonal(const Rotmat& _M, const T& _thres) // decltype(_M(0,0)) <-> T
    {
       auto c_Col0 = _M.getColumn0();
       auto c_Col1 = _M.getColumn1();
       auto c_Col2 = _M.getColumn2();
       if(!(equal<T>(c_Col0.norm(),T(1),_thres) &&
            equal<T>(c_Col1.norm(),T(1),_thres) &&
            equal<T>(c_Col2.norm(),T(1),_thres))){return false;}

       if(!(equal<T>(c_Col0.dot(c_Col1),T(0),_thres) &&
            equal<T>(c_Col0.dot(c_Col2),T(0),_thres) &&
            equal<T>(c_Col1.dot(c_Col2),T(0),_thres))){return false;}

        return true;
    }
    inline static Rotmat nearestOrthogonal(const Rotmat& _M)
    {
           Eigen::JacobiSVD< Eigen::Matrix<T,3,3,ERL_MAJOR_TYPE_VALUE> > c_SVD (_M, Eigen::ComputeFullU | Eigen::ComputeFullV);
           return Rotmat( c_SVD.matrixU () *
                          c_SVD.matrixV ().transpose());
    }

};

}

#undef ERL_TEMPLATE_ARGS
#undef ERL_MAJOR_TYPE_VALUE
#undef ERL_CLASS_ARGS
#undef ERL_MAJOR_TYPE
