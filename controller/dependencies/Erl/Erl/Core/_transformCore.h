/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

/// Core class for homogeneous transforms. Do not include manually.
#ifndef ERL_TRANSFORM_H
    #error "Do not include _transformCore.h -> include transform.h instead"
#endif

#if (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_GENERAL)

#define ERL_TEMPLATE_ARGS class T, int MajorType
#define ERL_MAJOR_TYPE_VALUE MajorType
#define ERL_CLASS_ARGS

#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_COL)

#define ERL_TEMPLATE_ARGS class T
#define ERL_MAJOR_TYPE_VALUE ColMajor
#define ERL_CLASS_ARGS <T, ERL_MAJOR_TYPE_VALUE>

#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_ROW)

#define ERL_TEMPLATE_ARGS class T
#define ERL_MAJOR_TYPE_VALUE RowMajor
#define ERL_CLASS_ARGS <T, ERL_MAJOR_TYPE_VALUE>

#endif

namespace Erl
{
namespace details
{
    template <ERL_TEMPLATE_ARGS> class transform_determine_alignment ERL_CLASS_ARGS { protected: Rotmat<T, ERL_MAJOR_TYPE_VALUE> rotation; Vector3<T> translation; };
}
template <ERL_TEMPLATE_ARGS> class
ERL_ALIGNAS_TRANSFORM
Transform ERL_CLASS_ARGS
{

protected:

    Rotmat<T, ERL_MAJOR_TYPE_VALUE> rotation;

    Vector3<T> translation;

public:

// General constructors ===================================================================================

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline Transform()
        : rotation()
        , translation()
    {}

    template<typename OtherType> inline Transform (const Eigen::Matrix<OtherType,3,4>& other)
        : rotation(other.template topLeftCorner<3,3>())
        , translation(other.template topRightCorner<3,1>())
    {}

    template<typename OtherType> inline Transform & operator= (const Eigen::Matrix<OtherType,3,4>& other)
    {
        rotation = other.template topLeftCorner<3,3>();
        translation = other.template topRightCorner<3,1>();
        return *this;
    }

    template<typename OtherType> inline Transform (const Eigen::Matrix<OtherType,4,4>& other)
        : rotation(other.template topLeftCorner<3,3>())
        , translation(other.template topRightCorner<3,1>())
    {}

    template<typename OtherType> inline Transform & operator= (const Eigen::Matrix<OtherType,4,4>& other)
    {
        rotation = other.template topLeftCorner<3,3>();
        translation = other.template topRightCorner<3,1>();
        return *this;
    }

    template<int OtherMajor> inline Transform (const Transform<T,OtherMajor> &Tr)
        : rotation(Tr.getRotation())
        , translation(Tr.getTranslation())
    {}

    template<int OtherMajor> inline Transform & operator = (const Transform<T,OtherMajor> &Tr)
    {
        rotation = Tr.getRotation();
        translation = Tr.getTranslation();
        return *this;
    }

    inline Transform (const T& _00, const T& _01, const T& _02, const T& _03, const T& _10, const T& _11, const T& _12, const T& _13, const T& _20, const T& _21, const T& _22, const T& _23)
        : rotation(_00, _01, _02, _10, _11, _12, _20, _21, _22)
        , translation(_03, _13, _23)
    {}

//  Constructors from vector type ==========================================================================

    template<typename OtherType> inline Transform (const Eigen::Matrix<OtherType,3,1>& Translation)
        : rotation(Eigen::Matrix<T,3,3>::Identity())
        , translation(Translation)
    {}

//  Constructors from rotation type =========================================================================

    template<typename OtherType> inline Transform (const Eigen::Matrix<OtherType,3,3>& Rotation)
        : rotation(Rotation)
        , translation(0,0,0)
    {}

    template<typename OtherType> inline Transform (const Quaternion<OtherType> &quaternion)
        : rotation(Rotmat<T, ERL_MAJOR_TYPE_VALUE>(quaternion))
        , translation(0,0,0)
    {}

//  Constructors from both types ============================================================================

    template<typename OtherType1, typename OtherType2> inline Transform (const Eigen::Matrix<OtherType1, 3, 3>& Rotation, const Eigen::Matrix<OtherType2, 3, 1>& Translation)
        : rotation(Rotation)
        , translation(Translation)
    {}

    template<typename OtherType1, typename OtherType2> inline Transform (const Quaternion<OtherType1> &quaternion, const Eigen::Matrix<OtherType2, 3, 1>& Translation)
        : rotation(Rotmat<T, ERL_MAJOR_TYPE_VALUE>(quaternion))
        , translation(Translation)
    {}

//  Static constructors from pointer ========================================================================

#if (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_GENERAL)
    template<typename OtherType> inline static Transform fromPointer (const OtherType *data, int dataMajorType);
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_COL)
    template<typename OtherType> inline static Transform fromPointer (const OtherType *data, int dataMajorType)
    {
        Transform returnTransform;
        if (dataMajorType == Erl::ColMajor)
        {
            returnTransform.rotation.data()[0] = data[0];
            returnTransform.rotation.data()[1] = data[1];
            returnTransform.rotation.data()[2] = data[2];

            returnTransform.rotation.data()[3] = data[3];
            returnTransform.rotation.data()[4] = data[4];
            returnTransform.rotation.data()[5] = data[5];

            returnTransform.rotation.data()[6] = data[6];
            returnTransform.rotation.data()[7] = data[7];
            returnTransform.rotation.data()[8] = data[8];

            returnTransform.translation.data()[0] = data[9];
            returnTransform.translation.data()[1] = data[10];
            returnTransform.translation.data()[2] = data[11];
        }
        else
        {
            returnTransform.rotation.data()[0] = data[0];
            returnTransform.rotation.data()[1] = data[4];
            returnTransform.rotation.data()[2] = data[8];

            returnTransform.rotation.data()[3] = data[1];
            returnTransform.rotation.data()[4] = data[5];
            returnTransform.rotation.data()[5] = data[9];

            returnTransform.rotation.data()[6] = data[2];
            returnTransform.rotation.data()[7] = data[6];
            returnTransform.rotation.data()[8] = data[10];

            returnTransform.translation.data()[0] = data[3];
            returnTransform.translation.data()[1] = data[7];
            returnTransform.translation.data()[2] = data[11];
        }
        return returnTransform;
    }
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_ROW)
    template<typename OtherType> inline static Transform fromPointer (const OtherType *data, int dataMajorType)
    {
        Transform returnTransform;
        if (dataMajorType == Erl::ColMajor)
        {
            returnTransform.rotation.data()[0] = data[0];
            returnTransform.rotation.data()[1] = data[1];
            returnTransform.rotation.data()[2] = data[2];
            returnTransform.translation.data()[0] = data[3];

            returnTransform.rotation.data()[3] = data[4];
            returnTransform.rotation.data()[4] = data[5];
            returnTransform.rotation.data()[5] = data[6];
            returnTransform.translation.data()[1] = data[7];

            returnTransform.rotation.data()[6] = data[8];
            returnTransform.rotation.data()[7] = data[9];
            returnTransform.rotation.data()[8] = data[10];
            returnTransform.translation.data()[2] = data[11];
        }
        else
        {
            returnTransform.rotation.data()[0] = data[0];
            returnTransform.rotation.data()[1] = data[3];
            returnTransform.rotation.data()[2] = data[6];
            returnTransform.translation.data()[0] = data[9];

            returnTransform.rotation.data()[3] = data[1];
            returnTransform.rotation.data()[4] = data[4];
            returnTransform.rotation.data()[5] = data[7];
            returnTransform.translation.data()[1] = data[10];

            returnTransform.rotation.data()[6] = data[2];
            returnTransform.rotation.data()[7] = data[5];
            returnTransform.rotation.data()[8] = data[8];
            returnTransform.translation.data()[2] = data[11];
        }
        return returnTransform;
    }
#endif

// Operators =======================================================================================================

    inline Transform operator * (const T &scalar) const
    {
        return Transform(scalar*rotation, scalar*translation);
    }

    inline Transform operator / (const T &scalar) const
    {
        return Transform(rotation/scalar, translation/scalar);
    }

    inline Transform & operator *= (const T &scalar)
    {
        rotation *= scalar;
        translation *= scalar;
        return *this;
    }

    inline Transform & operator /= (const T &scalar)
    {
        rotation /= scalar;
        translation /= scalar;
        return *this;
    }

    inline friend Transform operator * (const T &scalar, const Transform Tr)
    {
        return Tr*scalar;
    }

    inline Transform operator * (const Transform &Tr) const
    {
        Transform newTr;
        newTr.rotation = rotation*Tr.rotation;
        newTr.translation = rotation*Tr.translation + translation;
        return newTr;
    }

    inline Transform & operator *= (const Transform &Tr)
    {
        translation += rotation*Tr.translation;
        rotation     = rotation*Tr.rotation;
        return *this;
    }

    inline Vector3<T> operator * (const Vector3<T> &Vin) const
    {
        return Vector3<T>(rotation*Vin + translation);
    }

    inline Transform operator + (const Transform &Tr) const
    {
        Transform newTr;
        newTr.rotation = rotation + Tr.rotation;
        newTr.translation = translation + Tr.translation;
        return newTr;
    }

    inline Transform operator - (const Transform &Tr) const
    {
        Transform newTr;
        newTr.rotation = rotation - Tr.rotation;
        newTr.translation = translation - Tr.translation;
        return newTr;
    }

    inline Transform operator + () const
    {
        return *this;
    }

    inline Transform operator - () const
    {
        Transform newTr;
        newTr.rotation = - rotation ;
        newTr.translation = - translation;
        return newTr;
    }

    inline Transform & operator += (const Transform &Tr)
    {
        rotation += Tr.rotation;
        translation += Tr.translation;
        return *this;
    }

    inline Transform & operator -= (const Transform &Tr)
    {
        rotation -= Tr.rotation;
        translation -= Tr.translation;
        return *this;
    }

    template<typename OtherType> inline friend Transform operator * (const Rotmat<OtherType> &Rot, const Transform Tr)
    {
        return Transform<T> (Rotmat<T>(Rot*Tr.getRotationReference()), Vector3<T>(Rot*Tr.getTranslationReference()));
    }

    inline T operator () (const unsigned &r, const unsigned &c) const
    {
        if (c < 3)
            return rotation(r, c);
        else
            return translation(r);
    }

    inline T& operator () (const unsigned &r, const unsigned &c)
    {
        if (c < 3)
            return rotation(r, c);
        else
            return translation(r);
    }


    inline bool operator == (const Transform &Tr) const
    {
        return (rotation == Tr.rotation) && (translation == Tr.translation);
    }

    inline bool operator != (const Transform &Tr) const
    {
        return (rotation != Tr.rotation) || (translation != Tr.translation);
    }

    inline friend std::ostream & operator<< (std::ostream& os, const Transform &obj)
    {
        os << obj.rotation.coeff(0,0) << " " << obj.rotation.coeff(0,1) << " " << obj.rotation.coeff(0,2) << " " << obj.translation.coeff(0) << "\n";
        os << obj.rotation.coeff(1,0) << " " << obj.rotation.coeff(1,1) << " " << obj.rotation.coeff(1,2) << " " << obj.translation.coeff(1) << "\n";
        os << obj.rotation.coeff(2,0) << " " << obj.rotation.coeff(2,1) << " " << obj.rotation.coeff(2,2) << " " << obj.translation.coeff(2) << std::endl;
        return os;
    }

// Getters =======================================================================================================

    inline Quaternion<T> getQuaternion () const
    {
        return rotation.getQuaternion();
    }

    inline Rotmat<T, ERL_MAJOR_TYPE_VALUE> getRotation () const
    {
        return rotation;
    }

    inline Vector3<T> getTranslation () const
    {
        return translation;
    }

    inline Rotmat<T, ERL_MAJOR_TYPE_VALUE> & getRotationReference ()
    {
        return rotation;
    }

    inline Rotmat<T, ERL_MAJOR_TYPE_VALUE> const & getRotationReference () const
    {
        return rotation;
    }

    inline Vector3<T> & getTranslationReference ()
    {
        return translation;
    }

    inline Vector3<T> const & getTranslationReference () const
    {
        return translation;
    }

#if (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_GENERAL)
    inline T* getData (T* data, const int &dataMajorType) const;
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_COL)
    inline T* getData (T* data, const int &dataMajorType) const
    {
        if (dataMajorType == ColMajor)
        {
            memcpy(data  ,rotation.data()   ,sizeof(T)*9);
            memcpy(data+9,translation.data(),sizeof(T)*3);
        }
        else
        {
            data[0] = rotation.data()[0];
            data[1] = rotation.data()[3];
            data[2] = rotation.data()[6];
            data[3] = translation.data()[0];

            data[4] = rotation.data()[1];
            data[5] = rotation.data()[4];
            data[6] = rotation.data()[7];
            data[7] = translation.data()[1];

            data[8]  = rotation.data()[2];
            data[9]  = rotation.data()[5];
            data[10] = rotation.data()[8];
            data[11] = translation.data()[2];
        }
        return data;
    }
#elif (ERL_MAJOR_TYPE == ERL_MAJOR_TYPE_ROW)
    inline T* getData (T* data, const int &dataMajorType) const
    {
        if (dataMajorType == ColMajor)
        {
            data[0] = rotation.data()[0];
            data[1] = rotation.data()[3];
            data[2] = rotation.data()[6];

            data[3] = rotation.data()[1];
            data[4] = rotation.data()[4];
            data[5] = rotation.data()[7];

            data[6] = rotation.data()[2];
            data[7] = rotation.data()[5];
            data[8] = rotation.data()[8];

            data[8] = translation.data()[0];
            data[10] = translation.data()[1];
            data[11] = translation.data()[2];
        }
        else
        {
            data[0] = rotation.data()[0];
            data[1] = rotation.data()[1];
            data[2] = rotation.data()[2];
            data[3] = translation.data()[0];

            data[4] = rotation.data()[3];
            data[5] = rotation.data()[4];
            data[6] = rotation.data()[5];
            data[7] = translation.data()[1];

            data[8]  = rotation.data()[6];
            data[9]  = rotation.data()[7];
            data[10] = rotation.data()[8];
            data[11] = translation.data()[2];
        }
        return data;
    }
#endif

    inline Vector3<T> getColumn0 () const
    {
        return rotation.getColumn0();
    }

    inline Vector3<T> getColumn1 () const
    {
        return rotation.getColumn1();
    }

    inline Vector3<T> getColumn2 () const
    {
        return rotation.getColumn2();
    }

    inline Vector6<T> getVector6_RPY () const
    {
        return Vector6<T>(translation, rotation.getRPY());
    }

    inline Vector6<T> getVector6_Euler (const Axis &axis1, const Axis &axis2, const Axis &axis3) const
    {
        return Vector6<T>(translation, rotation.getEuler(axis1, axis2, axis3));
    }

    inline T getX() const
    {
        return translation.x();
    }

    inline T getY() const
    {
        return translation.y();
    }

    inline T getZ() const
    {
        return translation.z();
    }

    template<typename EigenT>
    inline void toEigen(EigenT & _EigenType)
    {
        _EigenType.template block<3,3>(0,0) = rotation   ;
        _EigenType.template block<3,1>(0,3) = translation;
    }

// Setters =======================================================================================================

    template<typename OtherType> inline void setTranslation (const Eigen::Matrix<OtherType,3,1> &Translation)
    {
        translation = Translation;
    }

    template<typename OtherType> inline void setRotation (const Eigen::Matrix<OtherType,3,3> &Rotation)
    {
        rotation = Rotation;
    }

    template<typename OtherType> inline void setRotation (const Quaternion<OtherType> &quaternion)
    {
        rotation = quaternion.template getMatrix<ERL_MAJOR_TYPE_VALUE>();
    }

    template <typename OtherType> inline void setColumn0 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        rotation.setColumn0(vector);
    }

    template <typename OtherType> inline void setColumn1 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        rotation.setColumn1(vector);
    }

    template <typename OtherType> inline void setColumn2 (const Eigen::Matrix<OtherType,3,1> &vector)
    {
        rotation.setColumn2(vector);
    }

    template<typename OtherType> inline void setAngleAxis (const double &angle, const Eigen::Matrix<OtherType,3,1> &vector)
    {
        rotation = Quaternion<T>::fromAngleAxis(angle, vector);
    }

    template<typename OtherType> inline void setX(const OtherType &x)
    {
        translation.x() = x;
    }

    template<typename OtherType> inline void setY(const OtherType &y)
    {
        translation.y() = y;
    }

    template<typename OtherType> inline void setZ(const OtherType &z)
    {
        translation.z() = z;
    }

    template<typename OtherType> inline static Transform fromAngleAxis (const double &angle, const Eigen::Matrix<OtherType,3,1> &vector)
    {
        return Transform(Quaternion<T>::fromAngleAxis(angle, vector));
    }

    inline static Transform fromDHtable (T theta, T d, T a, T alpha)
    {
        return Transform(cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), cos(theta)*a,
                         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), sin(theta)*a,
                                  0,             sin(alpha),             cos(alpha),            d);
    }

    template<typename OtherType> inline static Transform fromVector6_RPY (const Eigen::Matrix<OtherType,6,1> &vector)
    {
        return Transform(Rotmat<T>::fromRPY(vector[3], vector[4], vector[5]),
                         Vector3<T>(vector[0], vector[1], vector[2]));
    }

    template<typename OtherType> inline static Transform fromVector6_Euler (const Eigen::Matrix<OtherType,6,1> &vector,
                                                                            const Axis &axis1, const Axis &axis2, const Axis &axis3)
    {
        return Transform(Rotmat<T>::fromEuler(vector[3], vector[4], vector[5], axis1, axis2, axis3),
                         Vector3<T>(vector[0], vector[1], vector[2]));
    }

// Functions =======================================================================================================

    template <typename OtherType> inline Transform<OtherType> cast () const
    {
        Transform<OtherType> Tr;
        Tr.setRotation(Rotmat<OtherType>(rotation.template cast<OtherType>()));
        Tr.setTranslation(Vector3<OtherType>(translation.template cast<OtherType>()));
        return Tr;
    }

    inline Transform inv () const
    {
        Rotmat<T> Rinv = rotation.inv();
        return Transform(Rinv, Vector3<T>(-Rinv*translation));
    }

    inline Transform transformTo (const Transform &T1) const
    {
        return T1*inv();
    }

    inline void setIdentity()
    {
        (*this) = Transform::Identity();
    }

    inline bool hasNaN() const
    {
        return (rotation.hasNaN() || translation.hasNaN());
    }

    inline bool isnormal() const
    {
        return (rotation.isnormal() && translation.isnormal());
    }

    inline bool isfinite() const
    {
        return (rotation.isfinite() && translation.isfinite());
    }

    inline static Transform minJerkInterpolation (const Transform &Ti, const Transform &Tf, const double &t)
    {
        Quaternion<T> Qi = Ti.getQuaternion();
        Quaternion<T> Qf = Tf.getQuaternion();
        Vector3<T> Vi = Ti.getTranslation();
        Vector3<T> Vf = Tf.getTranslation();

        return Transform(Quaternion<T>::slerp(Qi, Qf, t), Vector3<T>::minJerkInterpolation(Vi, Vf, t));
    }

    inline static Transform minVelocityInterpolation (const Transform &Ti, const Transform &Tf, const double &t)
    {
        Quaternion<T> Qi = Ti.getQuaternion();
        Quaternion<T> Qf = Tf.getQuaternion();
        Vector3<T> Vi = Ti.getTranslation();
        Vector3<T> Vf = Tf.getTranslation();

        return Transform(Quaternion<T>::slerp(Qi, Qf, t), Vector3<T>::minVelocityInterpolation(Vi, Vf, t));
    }

    inline static Transform minAccelerationInterpolation (const Transform &Ti, const Transform &Tf, const double &t)
    {
        Quaternion<T> Qi = Ti.getQuaternion();
        Quaternion<T> Qf = Tf.getQuaternion();
        Vector3<T> Vi = Ti.getTranslation();
        Vector3<T> Vf = Tf.getTranslation();

        return Transform(Quaternion<T>::slerp(Qi, Qf, t), Vector3<T>::minAccelerationInterpolation(Vi, Vf, t));
    }

    enum
    {
        MinimumJerk,
        MinimumVelocity,
        MinimumAcceleration
    };

    inline static Transform interpolation (const Transform &Ti, const Transform &Tf, const double &t, const int interpolationType)
    {
        if (interpolationType == MinimumJerk)
        {
            return minJerkInterpolation(Ti, Tf, t);
        }
        else if (interpolationType == MinimumAcceleration)
        {
            return minAccelerationInterpolation(Ti, Tf, t);
        }
        else if (interpolationType == MinimumVelocity)
        {
            return minVelocityInterpolation(Ti, Tf, t);
        }
        ERL_ASSERT(false && "invalid interpolation type");
        return Tf;
    }

    inline static Transform Identity ()
    {
        return Transform(Rotmat<T, ERL_MAJOR_TYPE_VALUE>::Identity(), Vector3<T>::Zero());
    }

    inline static Transform NaN ()
    {
        return Transform(Rotmat<T, ERL_MAJOR_TYPE_VALUE>::NaN(), Vector3<T>::NaN());
    }

    inline static Transform Zero ()
    {
        return Transform(Rotmat<T, ERL_MAJOR_TYPE_VALUE>::Zero(), Vector3<T>::Zero());
    }

    inline static bool verifyOrthogonal(const Transform& _M, const T& _thres)
    {
        return Rotmat<T, ERL_MAJOR_TYPE_VALUE>::verifyOrthogonal(_M.getRotation(),_thres);
    }
    inline static Transform nearestOrthogonal(const Transform & _M)
    {
           return Transform(Rotmat<T, ERL_MAJOR_TYPE_VALUE>::nearestOrthogonal(_M.getRotation() ), _M.getTranslation());
    }
};

}

#undef ERL_TEMPLATE_ARGS
#undef ERL_MAJOR_TYPE_VALUE
#undef ERL_CLASS_ARGS
#undef ERL_MAJOR_TYPE
