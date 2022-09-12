/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_PQ_PRIMITIVES_H
#define ERL_PQ_PRIMITIVES_H

#include <Erl/Core.h>
#include <Erl/Utility/angle.h>
#include "pq_math.h"
#include "pq_discretize.h"
#include <type_traits>


#include <vector>
// ************************************************
// Primitives:
// Point: x,y,z
// Line:  conisting of two points this is a segment
// Line_inf: consisting of one point and one UNITdirection vector infit length
// Ray: consisting of one STARTpoint and one UNITdirection vector infit length
// Triangle: consisting of three points
// Plane: consisting of one point and UNITnormal
// Sphere: consisting of a point and a radius
// Contour: consisting of a point a UNITnormal and a radius, the contour is not just a circle but fills out the area
// Cylinder: cylinder is spanned by multiple contours, it has a volume and can have different radius and normals for each segment
// ************************************************

#ifndef PQ_ALIGNMENT
    #define PQ_ALIGNMENT(n,a)
#endif


namespace PQ{
    using namespace Erl;
    typedef std::integral_constant<long,Eigen::Dynamic> Dynamic;
    // ****************************************************************************
    template<typename T>
    class ProxyRes
    {
    private:

    protected:
        PQ_ALIGNMENT(4,T) Vector3<T> m_pProxy;
        PQ_ALIGNMENT(4,T) Vector3<T> m_pObj;
    public:
        typedef T type;
        inline ProxyRes<T>():m_pProxy(),m_pObj()
        {}
        inline ProxyRes<T>(const Vector3<T> & _pProxy,const Vector3<T> & _pObj)
            : m_pProxy(_pProxy)
            , m_pObj  (_pObj  )
        { }
        //Copy Constructor
        inline ProxyRes<T>(const ProxyRes<T> & _pRes)
            : m_pProxy(_pRes.m_pProxy)
            , m_pObj  (_pRes.m_pObj  )
        { }

        //Assignment operator
        inline ProxyRes<T>& operator=(const ProxyRes<T> & _pRes)
        {
            //if(this==&_pRes) return (*this);
            m_pProxy = _pRes.m_pProxy;
            m_pObj   = _pRes.m_pObj;
            return (*this);
        }
        // Destructor
        inline ~ProxyRes<T>(){}

        inline Vector3<T>& pProxy(){return m_pProxy;}
        inline Vector3<T>& pObj()  {return m_pObj;  }

        inline const Vector3<T>& pProxy() const {return m_pProxy;}
        inline const Vector3<T>& pObj  () const {return m_pObj;  }

        inline T norm()        const {return (m_pProxy-m_pObj).norm();}
        inline T squaredNorm() const {return (m_pProxy-m_pObj).squaredNorm();}

        inline Vector3<T> getVecProxy2Obj() const {return (m_pObj-m_pProxy);}
        inline Vector3<T> getVecObj2Proxy() const {return (m_pProxy-m_pObj);}

        inline void setProxy(const Vector3<T> & _pProxy){m_pProxy = _pProxy;}
        inline void setObj  (const Vector3<T> & _pObj  ){m_pObj   = _pObj  ;}

        inline ProxyRes<T>& swap()
        {
            Erl::Vector3<T> _tmp = m_pProxy;
            m_pProxy = m_pObj;
            m_pObj   = _tmp;
            return (*this);
        }

        inline bool operator==(const ProxyRes<T> & _other) const
        {
            return  (m_pProxy == _other.m_pProxy) &&
                    (m_pObj   == _other.m_pObj  );
        }

        inline friend  std::ostream& operator<< (std::ostream& _os, const ProxyRes & _o)
        {
            _os<<_o.m_pProxy<<std::endl;
            _os<<_o.m_pObj;
            return _os;
        }

    };
    typedef ProxyRes<float>  ProxyResf;
    typedef ProxyRes<double> ProxyResd;

    // ****************************************************************************
    template<typename T>
    class PQ_ALIGNMENT(4,T) Point : public Vector3<T>
    {
    private:

    protected:

    public:
        typedef T type;
        inline Point<T>(const Vector3<T> & _p)
            : Vector3<T>(_p)
        { }
        inline Point<T>(void)
            : Vector3<T>()
        { }
        inline Point<T>(T  x, T  y, T  z)
            : Vector3<T>(x,y,z)
        { }
        inline Point<T>(T const (&_p)[3])
         : Vector3<T>(_p[0],_p[1],_p[2])
        { }

        //Copy Constructor
        inline Point<T>(const Point<T> & _p)
            : Vector3<T>(_p)
        { }

        //Assignment operator
        inline Point<T>& operator=(const Vector3<T> & _p)
        {
            this->Vector3<T>::operator=(_p);
            return *this;
        }
        // Destructor
        inline ~Point<T>(){}

        //inline Vector3<T> operator() (){return ;}

    };
    typedef Point<float>  Pointf;
    typedef Point<double> Pointd;
    // ****************************************************************************
    template<typename T> class Segment
    {
    private:
        Point<T> m_p[2];

    protected:
    public:
        typedef T type;
        inline Segment<T>(){}
        inline Segment<T>(const Vector3<T> & _p0,const Vector3<T> & _p1){m_p[0]=_p0;m_p[1]=_p1;}


        inline Segment<T>(const Vector3<T> (&_p)[2]):m_p({_p[0],_p[1]}){}
        inline Vector3<T> const& operator[](unsigned _nIndex) const{
            assert(_nIndex<2);
            return m_p[_nIndex];
        }
        inline Vector3<T>& operator[](unsigned _nIndex){
            assert(_nIndex<2);
            return m_p[_nIndex];
        }
        inline Vector3<T>& p0(){return m_p[0];}
        inline Vector3<T>& p1(){return m_p[1];}
        inline const Vector3<T>& p0() const {return m_p[0];}
        inline const Vector3<T>& p1() const {return m_p[1];}

        template<size_t i> inline const Vector3<T>& p() const { static_assert(i<2 && i>=0,"wrong index"); return m_p[i];}
        template<size_t i> inline       Vector3<T>& p()       { static_assert(i<2 && i>=0,"wrong index"); return m_p[i];}

        inline Vector3<T> centre()   const { return (m_p[0]+m_p[1])*T(0.5);}
        inline Vector3<T> direction()const { return (m_p[1]-m_p[0]).normalized(); }
        inline T          radius()   const { return (m_p[1]-m_p[0]).norm()*T(.5);}

        //Copy Constructor
        inline Segment<T>(const Segment<T> & _l)
        {
            m_p[0]=_l.m_p[0];
            m_p[1]=_l.m_p[1];
        }

        //Assignment operator
        inline Segment<T>& operator=(const Segment<T> & _l)
        {
            m_p[0]= _l.m_p[0];
            m_p[1]= _l.m_p[1];
            return (*this);
        }
        // Destructor
        inline ~Segment<T>(){}

        template<class _T>
        inline friend  std::ostream& operator<< (std::ostream& _os, const Segment<_T> & _o)
        {
            _os<<_o.m_p[0]<<std::endl;
            _os<<_o.m_p[1];
            return _os;
        }

        // Discretize
        inline int discretize(const T& _length, Eigen::Matrix<T,3,Eigen::Dynamic>& _discrPoints) const
        {
            Vector3<T> c_IncVector = m_p[1]-m_p[0];
            T c_overall_length = (c_IncVector).norm();
            size_t c_elements = std::ceil(c_overall_length/_length);
            if(size_t(_discrPoints.cols()) < (c_elements+1) )
                _discrPoints.resize(Eigen::NoChange,c_elements+1);
            c_IncVector /= c_elements;

            for(size_t iP(0);iP<(c_elements+1);iP++)
            {  _discrPoints.col(iP) = m_p[0]+iP*c_IncVector; }

            return c_elements+1;
        }

    };
    typedef Segment<float>  Segmentf;
    typedef Segment<double> Segmentd;
    // ****************************************************************************
    template<typename T> class Pill
    {
    private:
        Point<T> m_p[2];
        PQ_ALIGNMENT(4,T)
        T        m_r;
    protected:

    public:
        typedef T type;
        inline Pill<T>(){}
        inline Pill<T>(const Vector3<T> & _p0, const Vector3<T> & _p1, const T & _r) : m_p{_p0,_p1}, m_r(_r) {}
        inline Pill<T>(const Vector3<T> (&_p)[2],const T & _r):m_p({_p[0],_p[1]}), m_r(_r){}
        inline Vector3<T> const& operator[](unsigned _nIndex) const{
            assert(_nIndex<2);
            return m_p[_nIndex];
        }
        inline Vector3<T>& operator[](unsigned _nIndex){
            assert(_nIndex<2);
            return m_p[_nIndex];
        }
        template <size_t index>
        inline Vector3<T>& p(){return m_p[index];}
        template <size_t index>
        inline const Vector3<T>& p() const {return m_p[index];}

        inline Vector3<T> centre()      const { return (m_p[0]+m_p[1])*T(.5);}
        inline Vector3<T> direction()   const { return (m_p[1]-m_p[0]).normalized(); }
        inline T          radius()      const { return (m_p[1]-m_p[0]).norm()*T(.5);}
        inline T          pill_radius() const { return m_r;}

        //Copy Constructor
        inline Pill<T>(const Pill<T> & _l)
        { m_p[0]= _l.m_p[0];
          m_p[1]= _l.m_p[1];
          m_r   = _l.m_r;
        }

        //Assignment operator
        inline Pill<T>& operator=(const Pill<T> & _l)
        {
            m_p[0]= _l.m_p[0];
            m_p[1]= _l.m_p[1];
            m_r   = _l.m_r   ;
            return (*this);
        }
        // Destructor
        inline ~Pill<T>(){}

        template<class _T>
        inline friend  std::ostream& operator<< (std::ostream& _os, const Pill<_T> & _o)
        {
            _os<<_o.m_p[0]<<std::endl;
            _os<<_o.m_p[1]<<std::endl;
            _os<<Erl::Vector3<T>(_o.m_r,0,0);
            return _os;
        }

//        // Discretize
//        inline int discretize(const T& _length, Eigen::Matrix<T,3,Eigen::Dynamic>& _discrPoints) const
//        {
//            Vector3<T> c_IncVector = m_p[1]-m_p[0];
//            T c_overall_length = (c_IncVector).norm();
//            size_t c_elements = std::ceil(c_overall_length/_length);
//            if(size_t(_discrPoints.cols()) < (c_elements+1) )
//                _discrPoints.resize(Eigen::NoChange,c_elements+1);
//            c_IncVector /= c_elements;

//            for(size_t iP(0);iP<(c_elements+1);iP++)
//            {  _discrPoints.col(iP) = m_p[0]+iP*c_IncVector; }

//            return c_elements+1;
//        }

    };
    typedef Segment<float>  Segmentf;
    typedef Segment<double> Segmentd;
    // ****************************************************************************
    template<typename T> class Line
    {
    private:
        Point<T> m_p;
        Point<T> m_v;
    protected:
    public:
        typedef T type;
        inline Line<T>(){}
        inline Line<T>(const Vector3<T> & _p,const Vector3<T> & _v):m_p(_p),m_v(_v){}
        inline Vector3<T>& p(){return m_p;}
        inline Vector3<T>& v(){return m_v;}
        inline const Vector3<T>& p() const {return m_p;}
        inline const Vector3<T>& v() const {return m_v;}

        //Copy Constructor
        inline Line<T>(const Line<T> & _l)
        {
            m_p=_l.m_p;
            m_v=_l.m_v;
        }

        //Assignment operator
        inline Line<T>& operator=(const Line<T> & _l)
        {
            m_p= _l.m_p;
            m_v= _l.m_v;
            return (*this);
        }
        // Destructor
        inline ~Line<T>(){}

        inline Vector3<T> const& operator[](unsigned _nIndex) const{
            assert(_nIndex<2);
            return (_nIndex == 0) ? m_p : m_v;
        }
        inline Vector3<T>& operator[](unsigned _nIndex){
            assert(_nIndex<2);
            return (_nIndex == 0) ? m_p : m_v;
        }

        template<class _T>
        inline friend  std::ostream& operator<< (std::ostream& _os, const Line<_T> & _o)
        {
            _os<<_o.m_p<<std::endl;
            _os<<_o.m_v;
            return _os;
        }

    };
    typedef Line<float>  Linef;
    typedef Line<double> Lined;
    // ****************************************************************************
    template<typename T> class Ray
    {
    private:
        Point<T> m_p;
        Point<T> m_v;
    protected:
    public:
        typedef T type;
        inline Ray<T>(const Vector3<T> & _p,const Vector3<T> & _v):m_p(_p),m_v(_v){}
        inline Vector3<T>& p(){return m_p;}
        inline Vector3<T>& v(){return m_v;}
        inline const Vector3<T>& p() const {return m_p;}
        inline const Vector3<T>& v() const {return m_v;}
        //Copy Constructor
        inline Ray<T>(const Ray<T> & _l)
        {
            m_p=_l.m_p;
            m_v=_l.m_v;
        }

        //Assignment operator
        inline Ray<T>& operator=(const Ray<T> & _l)
        {
            //if(this==&_l) return (*this);
            m_p= _l.m_p;
            m_v= _l.m_v;
            return (*this);
        }
        // Destructor
        inline ~Ray<T>(){}

        template<class _T>
        inline friend  std::ostream& operator<< (std::ostream& _os, const Ray<_T> & _o)
        {
            _os<<_o.m_p<<std::endl;
            _os<<_o.m_v;
            return _os;
        }

    };
    typedef Ray<float>  Rayf;
    typedef Ray<double> Rayd;
    // ****************************************************************************
    template<typename T> class Triangle
    {
    private:
        Point<T> m_p[3];
    protected:
    public:
        typedef T type;
        inline Triangle<T>(const Vector3<T> & _p0,const Vector3<T> & _p1,const Vector3<T> & _p2){m_p[0]=_p0;m_p[1]=_p1;m_p[2]=_p2;}
        inline Triangle<T>(const Vector3<T> (&_p)[3]):m_p({_p[0],_p[1],_p[2]}){}
        inline Triangle<T>(const Vector3<T> *_p):m_p({_p[0],_p[1],_p[2]}){}
        inline Triangle<T>(){}
        inline Vector3<T> const& operator[](unsigned _nIndex) const{
            assert(_nIndex<3);
            return m_p[_nIndex];
        }
        inline Vector3<T>& operator[](unsigned _nIndex){
            assert(_nIndex<3);
            return m_p[_nIndex];
        }
        inline Vector3<T>& p0(){return m_p[0];}
        inline Vector3<T>& p1(){return m_p[1];}
        inline Vector3<T>& p2(){return m_p[2];}

        inline const Vector3<T>& p0() const {return m_p[0];}
        inline const Vector3<T>& p1() const {return m_p[1];}
        inline const Vector3<T>& p2() const {return m_p[2];}

        template<size_t i> inline const Vector3<T>& p() const { static_assert(i<3 && i>=0,"wrong index"); return m_p[i];}
        template<size_t i> inline       Vector3<T>& p()       { static_assert(i<3 && i>=0,"wrong index"); return m_p[i];}

        //Copy Constructor
        inline Triangle<T>(const Triangle<T> & _t)
            : m_p{_t.m_p[0],_t.m_p[1],_t.m_p[2]}
        {
        }

        //Assignment operator
        inline Triangle<T>& operator=(const Triangle<T> & _t)
        {
            m_p[0]=_t.m_p[0];
            m_p[1]=_t.m_p[1];
            m_p[2]=_t.m_p[2];
            return (*this);
        }
        // Destructor
        inline ~Triangle<T>(){}


        inline friend  std::ostream& operator<< (std::ostream& _os, const Triangle & _o)
        {
            _os<<_o.m_p[0]<<std::endl;
            _os<<_o.m_p[1]<<std::endl;
            _os<<_o.m_p[2];
            return _os;
        }


        // Discretize
        inline int discretize(const T& _length, Eigen::Matrix<T,3,Eigen::Dynamic>& _discrPoints) const
        {
            Vector3<T> c_IncVector0 = m_p[1]-m_p[0];
            Vector3<T> c_IncVector1 = m_p[2]-m_p[0];
            Vector3<T> c_IncVector2 = m_p[2]-m_p[1];

            T c_InvVD0  = c_IncVector0.norm();
            T c_InvVD1  = c_IncVector1.norm();
            T c_InvVD2  = c_IncVector2.norm();

            T c_R0 = std::min(c_InvVD0,c_InvVD1)/std::max(c_InvVD0,c_InvVD1);
            T c_R1 = std::min(c_InvVD0,c_InvVD2)/std::max(c_InvVD0,c_InvVD2);
            T c_R2 = std::min(c_InvVD1,c_InvVD2)/std::max(c_InvVD1,c_InvVD2);


            const Vector3<T> * c_P0   =&m_p[0];
            Vector3<T> * c_Vec0 = &c_IncVector0;
            Vector3<T> * c_Vec1 = &c_IncVector1;


            size_t c_NouterIter = std::ceil(std::max(c_InvVD0,c_InvVD1)/_length);

            if(c_R1 > c_R0)
            {
                if(c_R2 > c_R1) // R2 largest
                {
                    c_Vec0 = &c_IncVector1;
                    c_Vec1 = &c_IncVector2;
                    c_IncVector1 *= T(-1.);
                    c_IncVector2 *= T(-1.);
                    c_NouterIter = std::ceil(std::max(c_InvVD1,c_InvVD2)/_length);
                    c_P0   =&m_p[2];
                }
                else  // R1 largest
                {
                    c_Vec0 = &c_IncVector0;
                    c_Vec1 = &c_IncVector2;
                    c_IncVector0 *= T(-1.);
                    c_NouterIter = std::ceil(std::max(c_InvVD0,c_InvVD1)/_length);
                    c_P0   =&m_p[1];
                }
            }
            else // R0 > R1
            {
                if(c_R2 > c_R0) // R2 largest
                {
                    c_Vec0 = &c_IncVector1;
                    c_Vec1 = &c_IncVector2;
                    c_IncVector1 *= T(-1.);
                    c_IncVector2 *= T(-1.);
                    c_NouterIter = std::ceil(std::max(c_InvVD1,c_InvVD2)/_length);
                    c_P0   =&m_p[2];
                }
            }
            (*c_Vec0) /= c_NouterIter;
            (*c_Vec1) /= c_NouterIter;

            c_NouterIter++;

            size_t c_elements = 1;
            Vector3<T> c_IncVec;
            for(size_t iOuter(1);iOuter<c_NouterIter;iOuter++)
            {
                c_IncVec = (iOuter*(*c_Vec0))-(iOuter*(*c_Vec1));
                c_elements += (1+std::ceil(c_IncVec.norm()/_length));
            }
            if(size_t(_discrPoints.cols()) < c_elements)
                _discrPoints.resize(Eigen::NoChange,c_elements);



            size_t c_elementsInner;
            size_t c_iP(1);

            _discrPoints.col(0) = (*c_P0);
            for(size_t iOuter(1);iOuter<c_NouterIter;iOuter++)
            {
                c_IncVec = (iOuter*(*c_Vec1))- (iOuter*(*c_Vec0));
                c_elementsInner = std::ceil(c_IncVec.norm()/_length);
                c_IncVec /= c_elementsInner;
                c_elementsInner++;

                for(size_t iInner(0);iInner<c_elementsInner;iInner++)
                {
                    _discrPoints.col(c_iP) = (*c_P0)+(iOuter*(*c_Vec0))+iInner*c_IncVec;
                    c_iP++;
                }

            }
            return c_elements;
        }
    };
    typedef Triangle<float>  Trianglef;
    typedef Triangle<double> Triangled;
    // ****************************************************************************
    template<typename T> class Plane
{
private:
    Point<T> m_p;
    Point<T> m_n;
protected:
public:
    typedef T type;
    inline Plane<T>(){}
    inline Plane<T>(const Vector3<T> & _p,const Vector3<T> & _n):m_p(_p),m_n(_n){}

    inline Vector3<T>& p(){return m_p;}
    inline Vector3<T>& n(){return m_n;}

    inline const Vector3<T>& p() const {return m_p;}
    inline const Vector3<T>& n() const {return m_n;}

    //Copy Constructor
    inline Plane<T>(const Plane<T> & _plane)
        : m_p(_plane.m_p)
        , m_n(_plane.m_n)
    { }

    //Assignment operator
    inline Plane<T>& operator=(const Plane<T> & _plane)
    {
        m_p=_plane.m_p;
        m_n=_plane.m_n;
        return (*this);
    }
    // Destructor
    inline ~Plane<T>(){}

    template<class _T>
    inline friend  std::ostream& operator<< (std::ostream& _os, const Plane<_T> & _o)
    {
        _os<<_o.m_p<<std::endl;
        _os<<_o.m_n;
        return _os;
    }

    };
    typedef Plane<float>  Planef;
    typedef Plane<double> Planed;
    // ****************************************************************************
    template<typename T> class Sphere
    {
    private:
        Point<T> m_p;
        PQ_ALIGNMENT(4,T)
        T m_r;
    protected:
    public:
        typedef T type;
        inline Sphere<T>(){}
        inline Sphere<T>(const Vector3<T> & _p,const T& _r):m_p(_p),m_r(_r){}
        inline Vector3<T>& p(){return m_p;}
        inline T         & r(){return m_r;}
        inline const Vector3<T>& p() const {return m_p;}
        inline const T         & r() const {return m_r;}
        //Copy Constructor
        inline Sphere<T>(const Sphere<T> & _s)
            : m_p(_s.m_p)
            , m_r(_s.m_r)
        { }

        //Assignment operator
        inline Sphere<T>& operator=(const Sphere<T> & _s)
        {
            m_p=_s.m_p;
            m_r=_s.m_r;
            return (*this);
        }
        // Destructor
        inline ~Sphere<T>(){}

        template<class _T>
        inline friend  std::ostream& operator<< (std::ostream& _os, const Sphere<_T> & _o)
        {
            _os<<_o.m_p<<std::endl;
            _os<<Erl::Vector3<T>(_o.m_r,0,0);
            return _os;
        }

    };
    typedef Sphere<float>  Spheref;
    typedef Sphere<double> Sphered;
    // ****************************************************************************
    template<typename T> class Contour
    {
    private:
        Point<T> m_p;
        Point<T> m_n;
        T m_r;
    protected:
    public:
        typedef T type;
        inline Contour<T>(){}
        inline Contour<T>(const Vector3<T> & _p,const Vector3<T> & _n,const T& _r):m_p(_p),m_n(_n),m_r(_r){}
        inline Vector3<T>& p(){return m_p;}
        inline Vector3<T>& n(){return m_n;}
        inline T         & r(){return m_r;}

        inline const Vector3<T>& p() const {return m_p;}
        inline const Vector3<T>& n() const {return m_n;}
        inline const T         & r() const {return m_r;}
        //Copy Constructor
        inline Contour<T>(const Contour<T> & _c)
            : m_p(_c.m_p)
            , m_n(_c.m_n)
            , m_r(_c.m_r)
        {
        }

        //Assignment operator
        inline Contour<T>& operator=(const Contour<T> & _c)
        {
            m_p=_c.m_p;
            m_n=_c.m_n;
            m_r=_c.m_r;
            return (*this);
        }
        // Destructor
        inline ~Contour<T>(){}

        template<class _T>
        inline friend  std::ostream& operator<< (std::ostream& _os, const Contour<_T> & _o)
        {
            _os<<_o.m_p<<std::endl;
            _os<<_o.m_n<<std::endl;
            _os<<Erl::Vector3<T>(_o.m_r,0,0);
            return _os;
        }
        inline size_t discretize_size(const T& _length) const
        {
            size_t c_elements = 1;
            size_t c_NouterIter = std::ceil(m_r/_length);
            T c_lenRad = m_r/c_NouterIter;
            for(size_t iO(0);iO<c_NouterIter;iO++)
            { c_elements += std::ceil((T(2.)*ERL_PI*(iO+1)*c_lenRad)/_length); }
            return c_elements;
        }

        template <typename derived>
        inline size_t discretize_noresize(const T& _length, derived&& _discrPoints) const
        {
            size_t iP(1);
            size_t c_NouterIter = std::ceil(m_r/_length);
            T c_lenRad = m_r/c_NouterIter;
            _discrPoints.col(0) = m_p;

            Erl::Vector3<T> c_u,c_v;
            null(m_n,c_u,c_v);
            T c_su,c_sv;

            for(size_t iO(0);iO<c_NouterIter;iO++)
            {
                size_t c_NinnerIter = std::ceil((T(2.)*ERL_PI*(iO+1)*c_lenRad)/_length);
                T c_RadiusTmp = (iO+1)*c_lenRad;
                for(size_t iI(0);iI<c_NinnerIter;iI++)
                {
                    c_su = sin((2*ERL_PI*iI)/c_NinnerIter);
                    c_sv = cos((2*ERL_PI*iI)/c_NinnerIter);
                    _discrPoints.col(iP) = m_p+c_RadiusTmp*(c_su*c_u+c_sv*c_v);
                    iP++;
                }

            }
            return iP;

        }

        template <typename derived>
        inline void discretize_noresize_o(const T& _length, derived&& _discrPoints) const
        {
             size_t iP(1);
             size_t c_NouterIter = std::ceil(m_r/_length);
             T c_lenRad = m_r/c_NouterIter;
             _discrPoints.col(0) = m_p;
             Erl::Rotmat<T> c_RotMat = Erl::Rotmat<T>::Identity();
             Erl::Vector3<T> c_Z(0,0,1);		  // c_Viz: Z direction of Instrument
             Erl::Vector3<T> c_N(m_n);      // c_VtoRCM: Vector from current position to RCM position
             c_N.normalize();
             Erl::Vector3<T> c_ZcrossN = c_Z.cross(c_N);			               // c_VZcrossVtoRCM: Cross Product between c_Viz AND c_VtoRCM

             if(c_ZcrossN.norm()>PQ::Constants<T>::Zero_Tolerance)
             { c_RotMat = Erl::Rotmat<T>::fromAngleAxis(atan2(c_ZcrossN.norm(),c_Z.dot(c_N)),c_ZcrossN.normalized());}

             Erl::Vector3<T> c_PPlane(0,0,0);
             for(size_t iO(0);iO<c_NouterIter;iO++)
             {
                 size_t c_NinnerIter = std::ceil((T(2.)*ERL_PI*(iO+1)*c_lenRad)/_length);
                 T c_RadiusTmp = (iO+1)*c_lenRad;
                 Erl::Rotmat<T>::fromAngleAxis((T(2.)*ERL_PI)/c_NinnerIter,m_n);
                 for(size_t iI(0);iI<c_NinnerIter;iI++)
                 {
                     c_PPlane[0] = c_RadiusTmp*sin((2*ERL_PI*iI)/c_NinnerIter);
                     c_PPlane[1] = c_RadiusTmp*cos((2*ERL_PI*iI)/c_NinnerIter);
                     _discrPoints.col(iP) = m_p+c_RotMat*c_PPlane;
                     iP++;
                 }

             }
             return ;
        }

        template <typename matrix>
        inline int discretize(const T& _length, matrix& _discrPoints) const
        {
            size_t c_elements = discretize_size(_length);

            if(size_t(_discrPoints.cols()) < c_elements)
                _discrPoints.resize(Eigen::NoChange,c_elements);

            return ( discretize_noresize(_length,_discrPoints) , c_elements );
        }
    };
    typedef Contour<float>  Contourf;
    typedef Contour<double> Contourd;
    // ****************************************************************************
    template<typename T> class Circle
    {
    private:
        Point<T> m_p;
        Point<T> m_n;
        T m_r;
    protected:
    public:
        typedef T type;
        inline Circle<T>(){}
        inline Circle<T>(const Vector3<T> & _p,const Vector3<T> & _n,const T& _r):m_p(_p),m_n(_n),m_r(_r){}
        inline Vector3<T>& p(){return m_p;}
        inline Vector3<T>& n(){return m_n;}
        inline T         & r(){return m_r;}

        inline const Vector3<T>& p() const {return m_p;}
        inline const Vector3<T>& n() const {return m_n;}
        inline const T         & r() const {return m_r;}

        //Copy Constructor
        inline Circle<T>(const Circle<T> & _c)
            : m_p(_c.m_p)
            , m_n(_c.m_n)
            , m_r(_c.m_r)
        { }

        //Assignment operator
        inline Circle<T>& operator=(const Circle<T> & _c)
        {
            m_p=_c.m_p;
            m_n=_c.m_n;
            m_r=_c.m_r;
            return (*this);
        }
        // Destructor
        inline ~Circle<T>(){}


        template<class _T>
        inline friend  std::ostream& operator<< (std::ostream& _os, const Circle<_T> & _o)
        {
            _os<<_o.m_p<<std::endl;
            _os<<_o.m_n<<std::endl;
            _os<<Erl::Vector3<T>(_o.m_r,0,0);
            return _os;
        }

        inline int discretize(const T& _length, Eigen::Matrix<T,3,Eigen::Dynamic>& _discrPoints) const
        {
             size_t c_elements = std::ceil((T(2.)*ERL_PI*m_r)/_length);
             if(size_t(_discrPoints.cols()) < c_elements)
                 _discrPoints.resize(Eigen::NoChange,c_elements);

             Erl::Rotmat<T> c_RotMat = Erl::Rotmat<T>::Identity();
             Erl::Vector3<T> c_Z(0,0,1);		  // c_Viz: Z direction of Instrument
             Erl::Vector3<T> c_N(m_n);      // c_VtoRCM: Vector from current position to RCM position
             c_N.normalize();
             Erl::Vector3<T> c_ZcrossN = c_Z.cross(c_N);			               // c_VZcrossVtoRCM: Cross Product between c_Viz AND c_VtoRCM

             if(c_ZcrossN.norm()>PQ::Constants<T>::Zero_Tolerance)
             {
                 c_RotMat = Erl::Rotmat<T>::fromAngleAxis(atan2(c_ZcrossN.norm(),c_Z.dot(c_N)),c_ZcrossN.normalized());
             }
             Erl::Vector3<T> c_PPlane(0,0,0);
             //Erl::Rotmat<T>::fromAngleAxis((T(2.)*ERL_PI)/c_elements,m_n);
             for(size_t iI(0);iI<c_elements;iI++)
             {
                 c_PPlane[0] = m_r*sin((2*ERL_PI*iI)/c_elements);
                 c_PPlane[1] = m_r*cos((2*ERL_PI*iI)/c_elements);
                 _discrPoints.col(iI) = m_p+c_RotMat*c_PPlane;
             }
             return c_elements;
        }
    };
    typedef Circle<float>  Circlef;
    typedef Circle<double> Circled;
    // ****************************************************************************
    template<typename T> class Cylinder
    {
        private:
        protected:
            Point<T> m_p[2];
            PQ_ALIGNMENT(4,T)
            T m_r;
        public:
            typedef T type;
            inline Cylinder<T>()
                {}
            inline Cylinder<T>(const Erl::Vector3<T>& _p1, const Erl::Vector3<T>& _p2, const T& _r)
                : m_p{_p1,_p2}
                , m_r(_r) {}
            inline Cylinder<T>(Contour<T> _cs, T _h)
                : m_p{_cs.p(),(_cs.p()+_cs.n()*_h)}
                , m_r(_cs.r())
            {}
            inline Cylinder<T>(const Erl::Vector3<T>& _p, const Erl::Vector3<T>& _n, const T& _h, const T& _r)
                : m_p{_p,(_p+_n*_h)}
                , m_r(_r)
            {}

            inline Vector3<T> const& operator[](unsigned _n) const { assert(_n<2); return m_p[_n];}
            inline Vector3<T> &      operator[](unsigned _n)       { assert(_n<2); return m_p[_n];}

            inline Vector3<T>& p1(){return m_p[0];}
            inline Vector3<T>& p2(){return m_p[1];}
            inline T         & r() {return m_r;   }

            inline const Vector3<T>& p1() const { return m_p[0];}
            inline const Vector3<T>& p2() const { return m_p[1];}
            inline const T         & r()  const { return m_r;   }

            template<int i> inline const Vector3<T>& p() const { static_assert(((i<2)&&(i>=0)),"wrong index"); return m_p[i];}
            template<int i> inline       Vector3<T>& p()       { static_assert(((i<2)&&(i>=0)),"wrong index"); return m_p[i];}


            inline  T          h()  const { return (m_p[1]-m_p[0]).norm();   }
            inline  Vector3<T> n()  const { return (m_p[1]-m_p[0]).normalized();   }

            //Copy Constructor
            inline Cylinder<T>(const Cylinder<T> & _c)
                : m_p{_c.m_p[0],_c.m_p[1]}
                , m_r(_c.m_r)
            { }

            //Assignment operator
            inline Cylinder<T>& operator=(const Cylinder<T> & _c)
            {
                m_p[0]=_c.m_p[0];
                m_p[1]=_c.m_p[1];
                m_r=_c.m_r;
                return (*this);
            }
            // Destructor
            inline ~Cylinder<T>(){}


            template<class _T>
            inline friend  std::ostream& operator<< (std::ostream& _os, const Cylinder<_T> & _o)
            {
                _os<<_o.m_p[0]<<std::endl;
                _os<<_o.m_p[1]<<std::endl;
                _os<<Erl::Vector3<_T>(_o.m_r,0,0);
                return _os;
            }
            template <typename matrix>
            inline int discretize(const T& _length, matrix& _discrPoints) const
            {
                size_t c_NLines = std::ceil((T(2.)*ERL_PI*m_r)/_length);
                Erl::Vector3<T> c_n = n();
                T c_height          = h();
                Erl::Vector3<T> c_u,c_v;

                null(c_n,c_u,c_v);
                Contour<T> c_con(m_p[0],c_n,m_r);

                size_t c_sizeCont  = c_con.discretize_size(_length);
                size_t c_sizeLine  = std::ceil(c_height/_length);
                size_t c_sizeShaft = c_NLines*c_sizeLine;

                size_t c_elements(0);
                c_elements+=c_sizeCont;
                c_elements+=c_sizeShaft;
                c_elements+=c_sizeCont;

                if(size_t(_discrPoints.cols()) < c_elements)
                    _discrPoints.resize(Eigen::NoChange,c_elements);

                _discrPoints.setZero();
                c_con.discretize_noresize(_length, _discrPoints.block(0,0,3,c_sizeCont));
                c_con.p() = m_p[1];
                c_con.discretize_noresize(_length, _discrPoints.block(0,c_sizeCont+c_sizeShaft,3,c_sizeCont));

                size_t iCol = c_sizeCont;
                T c_su,c_sv;
                Erl::Vector3<T> c_PCircle, c_HeightVec;
                c_HeightVec = (m_p[1]-m_p[0]) / T(c_sizeLine);

                for(size_t iI(0);iI<c_NLines;iI++)
                {
                    c_su = sin(T(2.*ERL_PI*iI)/c_NLines);
                    c_sv = cos(T(2.*ERL_PI*iI)/c_NLines);
                    c_PCircle = m_p[0]+(m_r*(c_su*c_u+c_sv*c_v));
                    for(size_t iJ(0);iJ<c_sizeLine;iJ++)
                    {
                        _discrPoints.col(iCol++) = c_PCircle + c_HeightVec* T(iJ);
                    }
                }
                return c_elements;
            }


    };
    typedef Cylinder<float > Cylinderf;
    typedef Cylinder<double> Cylinderd;
    // ****************************************************************************
    enum class CylinderType
    {
        CENTRE = 0,
        FRONT  = 1,
        BACK   = 2,
        BOTH   = 3
    };
    template<typename T>
    class CylinderE : public Cylinder<T>
    {
        typedef Cylinder<T>  BaseClass;
        typedef CylinderType CType;
        public:
            inline CylinderE()
                : BaseClass()
                , m_CylinderType(CType::BOTH)
            {}
            inline CylinderE(const Erl::Vector3<T>& _p1, const Erl::Vector3<T>& _p2, const T& _r,
                             const CType& _ct = CType::BOTH)
                : BaseClass(_p1,_p2,_r)
                , m_CylinderType(_ct)
            {}
            inline CType getCylinderType() const
            { return m_CylinderType; }
            inline void setCylinderType(const CType& _CylinderType)
            {        m_CylinderType = _CylinderType; }

        protected:
            PQ_ALIGNMENT(4,T) CType m_CylinderType;
    };
    // ****************************************************************************
    namespace details {
        template<int Seg>
        struct seg_size
        {
            static constexpr long  S  = Seg  ;
            static constexpr long  Sp = Seg+1;
        };
        template<>
        struct seg_size<Eigen::Dynamic>
        {
            static constexpr long  S  = Eigen::Dynamic;
            static constexpr long  Sp = Eigen::Dynamic;
        };

    }


    template<typename T, typename Seg> class Cylinders
    {
        public:
            typedef typename Seg::value_type S_t;
            static constexpr S_t  S  = details::seg_size<Seg::value>::S ;
            static constexpr S_t  Sp = details::seg_size<Seg::value>::Sp;

            typedef Eigen::Matrix< Erl::Vector3<T>, Sp, 1> MatP;
            typedef Eigen::Matrix<              T , S , 1> MatR;

        protected:
            MatP m_p;
            MatR m_r;
        public:
            inline Cylinders<T,Seg>()
                {}
            inline Cylinders<T,Seg>(const MatP & _p, const T& _r)
                : m_p(_p)
                , m_r(MatR::One(_p.rows()-1,1)*_r)
                {std::fill(m_r.begin(),m_r.end(),_r);}
            inline Cylinders<T,Seg>(const MatP & _p, const  MatR & _r)
                : m_p(_p)
                , m_r(_r) {}

            template <class InitP, class InitR/*, typename std::enable_if< !std::is_void<typename InitP::const_iterator>::type>*/ >
            inline Cylinders<T,Seg>(const InitP& _p, const InitR& _r)
                : m_p(MatP(_p.size(),1))
                , m_r(MatR(_r.size(),1))
            {assert(m_r.rows() == (m_p.rows()-1));
             auto iOp  = _p.begin(); auto iSp  = 0;
             auto iOr  = _r.begin(); auto iSr  = 0;
             for(;iOp != _p.end()      && iSp != m_p.rows(); iOp++, iSp++){m_p(iSp,0) = *iOp;}
             for(;iOr != _r.end()      && iSr != m_r.rows(); iOr++, iSr++){m_r(iSr,0) = *iOr;} }

            template <class InitP>
            inline Cylinders<T,Seg>(const InitP& _p, const T& _r)
                : m_p(MatP(_p.size()   ,1))
                , m_r(MatR(m_p.rows()-1,1))
                {auto iOp  = _p.begin(); auto iSp  = 0;
                 for(;iOp != _p.end()      && iSp != m_p.rows(); iOp++, iSp++){m_p(iSp,0) = *iOp;}
                 std::fill(m_r.data(),m_r.data()+m_r.rows(),_r);}


            inline Vector3<T> const& operator[](size_t i) const { assert(i<S+1); return m_p[i];}
            inline Vector3<T> &      operator[](size_t i)       { assert(i<S+1); return m_p[i];}

            template<typename I> inline       Vector3<T>& p()       { static_assert(I::value < (S+1) || S<0,"wrong index"); return m_p[I::value]; }
            template<unsigned I> inline       Vector3<T>& p()       { static_assert(I        < (S+1) || S<0,"wrong index"); return m_p[I       ]; }
            template<typename I> inline const Vector3<T>& p() const { static_assert(I::value < (S+1) || S<0,"wrong index"); return m_p[I::value]; }
            template<unsigned I> inline const Vector3<T>& p() const { static_assert(I        < (S+1) || S<0,"wrong index"); return m_p[I       ]; }

            template<typename I> inline               T & r()       { static_assert(I::value < ( S ) || S<0,"wrong index"); return m_r[I::value]; }
            template<unsigned I> inline               T & r()       { static_assert(I        < ( S ) || S<0,"wrong index"); return m_r[I       ]; }
            template<typename I> inline const         T & r() const { static_assert(I::value < ( S ) || S<0,"wrong index"); return m_r[I::value]; }
            template<unsigned I> inline const         T & r() const { static_assert(I        < ( S ) || S<0,"wrong index"); return m_r[I       ]; }

            template<typename I> inline       T           h() const { static_assert(I::value < ( S ) || S<0,"wrong index"); return (m_p[I::value+1]-m_p[I::value]).norm();       }
            template<unsigned I> inline       T           h() const { static_assert(I        < ( S ) || S<0,"wrong index"); return (m_p[I       +1]-m_p[I       ]).norm();       }
            template<typename I> inline       Vector3<T>  n() const { static_assert(I::value < ( S ) || S<0,"wrong index"); return (m_p[I::value+1]-m_p[I::value]).normalized(); }
            template<unsigned I> inline       Vector3<T>  n() const { static_assert(I        < ( S ) || S<0,"wrong index"); return (m_p[I       +1]-m_p[I       ]).normalized(); }

            inline long                 size() const {return m_r.rows();}

            inline       Vector3<T>& p(long i)       { assert(i<=size() && "wrong index"); return m_p[i]; }
            inline const Vector3<T>& p(long i) const { assert(i<=size() && "wrong index"); return m_p[i]; }

            inline       T         & r(long i)       { assert(i< size() && "wrong index" ); return m_r[i]; }
            inline const T         & r(long i) const { assert(i< size() && "wrong index" ); return m_r[i]; }
            inline       T           h(long i) const { assert(i< size() && "wrong index" ); return (m_p[i+1]-m_p[i]).norm();       }
            inline       Vector3<T>  n(long i) const { assert(i< size() && "wrong index" ); return (m_p[i+1]-m_p[i]).normalized(); }


            //Copy Constructor
            inline Cylinders<T,Seg>(const Cylinders<T,Seg> & _c)
                : m_p(_c.m_p)
                , m_r(_c.m_r)
            { }

            //Assignment operator
            inline Cylinders<T,Seg>& operator=(const Cylinders<T,Seg> & _c)
            {
                m_p=_c.m_p;
                m_r=_c.m_r;
                return (*this);
            }
            // Destructor
            inline ~Cylinders<T,Seg>(){}


            template<class _T,class _Seg>
            inline friend  std::ostream& operator<< (std::ostream& _os, const Cylinders<_T,_Seg> & _o)
            {
                /*
                Erl::static_for<0,S+1,1>([&](int i){
                     _os<<_o.m_p[i]<<std::endl;
                });
                Erl::static_for<0,S-1,1>([&](int i){
                     _os<<Erl::Vector3<_T>(_o.m_r[i],0,0)<<std::endl;
                });
                _os<<Erl::Vector3<_T>(_o.m_r[S-1],0,0);
                */
                long _S = _o.size();
                 _os<<Erl::Vector3<_T>(_S,S,0)<<std::endl;
                for(long i(0);i<_S+1;i++){ _os<<_o.m_p[i]<<std::endl;}
                for(long i(0);i<_S-1;i++){ _os<<Erl::Vector3<_T>(_o.m_r[i],0,0)<<std::endl;};
                _os<<Erl::Vector3<_T>(_o.m_r[_S-1],0,0);
                return _os;
            }
            inline int discretize_size(const T& _length) const
            {
                size_t c_elements = 0;
                long S = size();
                c_elements += Contour<T>(m_p[0],n(0)  ,m_r[0  ]).discretize_size(_length);
                c_elements += Contour<T>(m_p[S],n(S-1),m_r[S-1]).discretize_size(_length);

                for(long iS(0);iS<S;iS++)
                {
                    c_elements +=
                      std::ceil(r(iS)*2.*ERL_PI/_length)
                   * (std::ceil(h(iS)          /_length)+1);
                }
                for(long iS(1);iS<S;iS++)
                {
                    c_elements += discHalfUVSphereSize(_length,m_p[iS],n(iS-1),m_r[iS-1]);
                    c_elements += discHalfUVSphereSize(_length,m_p[iS],n(iS  ),m_r[iS  ]);
                }
                return c_elements;

            }
            template <typename matrix>
            inline int discretize(const T& _length, matrix& _discrPoints) const
            {
                long S = size();
                size_t c_elements = discretize_size(_length);

                if(size_t(_discrPoints.cols()) < c_elements)
                    _discrPoints.resize(Eigen::NoChange,c_elements);

                _discrPoints.setZero();

                size_t c_iP = Contour<T>(m_p[0],n<0>(),m_r[0]).discretize_noresize(_length,_discrPoints);

                Erl::Vector3<T> c_n = (-1)*n(0);

                for(long iS(0);iS<S;iS++)
                {
                    size_t c_NC = std::ceil(h(iS)/_length)+1;
                    size_t c_NP = std::ceil(r(iS)*2.*ERL_PI/_length);

                    T c_su,c_sv;
                    Erl::Vector3<T> c_u,c_v,c_p,c_pv;c_n *= (-1);
                    null_n(c_n,c_u,c_v);

                    for(size_t iP(0);iP<c_NP;iP++)
                    {
                        c_su = std::sin(T(2.*ERL_PI*iP)/c_NP);
                        c_sv = std::cos(T(2.*ERL_PI*iP)/c_NP);
                        c_p  = m_p[iS  ] + m_r[iS]*(c_su*c_u + c_sv*c_v);
                        c_pv = m_p[iS+1] - m_p[iS];

                        for(size_t iC(0);iC<c_NC;iC++)
                        { _discrPoints.col(c_iP++) = c_p+c_pv*T(iC)/T(c_NC-1);} //Erl::Vector3<T>::Zero();
                    }
                    if(iS<(S-1))
                    {
                        c_iP+= discHalfUVSphere(_length, m_p[iS+1],c_n,m_r[iS  ], _discrPoints.block(0,c_iP,3,c_elements-c_iP));
                        c_n = (-1)*n(iS+1);
                        c_iP+= discHalfUVSphere(_length, m_p[iS+1],c_n,m_r[iS+1], _discrPoints.block(0,c_iP,3,c_elements-c_iP));
                    }
                }
                c_iP += Contour<T>(m_p[S],n(S-1),m_r[S-1]).discretize_noresize(_length,_discrPoints.block(0,c_iP,3,c_elements-c_iP));


                return c_iP;
            }
    };
    template <typename Seg>
    using Cylindersf = Cylinders<float , Seg>;

    template <typename Seg>
    using Cylindersd = Cylinders<double, Seg>;

    // ****************************************************************************
    template<typename T> class GCylinder
    {
    private:
        Contour<T> m_cs[2];
    protected:
    public:
        typedef T type;
        inline GCylinder<T>(){}
        inline GCylinder<T>(const Contour<T> & _cont0,const Contour<T> & _cont1):m_cs{_cont0,_cont1}{}
        inline GCylinder<T>(const Vector3<T> & _p0,const Vector3<T> & _n0,const T& _r0,
                            const Vector3<T> & _p1,const Vector3<T> & _n1,const T& _r1)
            : m_cs{Contour<T>(_p0,_n0,_r0),Contour<T>(_p1,_n1,_r1)}{}

        inline Contour<T> const& operator[](unsigned _nIndex) const{
            assert(_nIndex<2);
            return m_cs[_nIndex];
        }
        inline Contour<T>& operator[](unsigned _nIndex){
            assert(_nIndex<2);
            return m_cs[_nIndex];
        }
        //Copy Constructor
        inline GCylinder<T>(const GCylinder<T> & _s)
            : m_cs{_s.m_cs[0],_s.m_cs[1]}
        {
        }

        inline Contour<T>& con1(){return m_cs[0];}
        inline Contour<T>& con2(){return m_cs[1];}

        inline const Contour<T>& con1() const {return m_cs[0];}
        inline const Contour<T>& con2() const {return m_cs[1];}

        //Assignment operator
        inline GCylinder<T>& operator=(const GCylinder<T> & _s)
        {
            m_cs[0] = _s.m_cs[0];
            m_cs[1] = _s.m_cs[1];
            return (*this);
        }
        // Destructor
        inline ~GCylinder<T>(){}

        template<class _T>
        inline friend  std::ostream& operator<< (std::ostream& _os, const GCylinder<_T> & _o)
        {
            _os<<_o.m_cs[0]<<std::endl;
            _os<<_o.m_cs[1];
            return _os;
        }
        template <typename matrix>
        inline int discretize(const T& _length, matrix& _discrPoints) const
        {

            T c_r = std::max(m_cs[0].r(),m_cs[1].r());

            size_t c_NLines = std::ceil((T(2.)*ERL_PI*c_r)/_length);
            Erl::Vector3<T> c_u0,c_u1,c_v0,c_v1;

            null(m_cs[0].n(),c_u0,c_v0);
            null(m_cs[1].n(),c_u1,c_v1);

            size_t c_sizeCont1 = m_cs[0].discretize_size(_length);
            size_t c_sizeCont2 = m_cs[1].discretize_size(_length);
            size_t c_sizeShaft = 0;

            T c_su,c_sv;
            Erl::Vector3<T> c_PCircle1,c_PCircle2;

            for(size_t iI(0);iI<c_NLines;iI++)
            {
                c_su = sin(T(2.*ERL_PI*iI)/c_NLines);
                c_sv = cos(T(2.*ERL_PI*iI)/c_NLines);

                c_PCircle1 = m_cs[0].p()+(m_cs[0].r()*(c_su*c_u0+c_sv*c_v0));
                c_PCircle2 = m_cs[1].p()+(m_cs[1].r()*(c_su*c_u1+c_sv*c_v1));

                c_sizeShaft += std::ceil((c_PCircle2-c_PCircle1).norm()/_length)+1;
            }

            size_t c_elements(0);
            c_elements+=c_sizeCont1;
            c_elements+=c_sizeShaft;
            c_elements+=c_sizeCont2;

            if(size_t(_discrPoints.cols()) < c_elements)
                _discrPoints.resize(Eigen::NoChange,c_elements);

            _discrPoints.setZero();
            m_cs[0].discretize_noresize(_length, _discrPoints.block(0,0,3,c_sizeCont1));
            m_cs[1].discretize_noresize(_length, _discrPoints.block(0,c_sizeCont1+c_sizeShaft,3,c_sizeCont2));

            size_t iCol = c_sizeCont1;

            for(size_t iI(0);iI<c_NLines;iI++)
            {
                c_su = sin(T(2.*ERL_PI*iI)/c_NLines);
                c_sv = cos(T(2.*ERL_PI*iI)/c_NLines);

                c_PCircle1 = m_cs[0].p()+(m_cs[0].r()*(c_su*c_u0+c_sv*c_v0));
                c_PCircle2 = m_cs[1].p()+(m_cs[1].r()*(c_su*c_u1+c_sv*c_v1));

                c_PCircle2 -= c_PCircle1;

                size_t c_sizeLine = std::ceil(c_PCircle2.norm()/_length)+1;

                for(size_t iJ(0);iJ<c_sizeLine;iJ++)
                {
                    _discrPoints.col(iCol++) = c_PCircle1 + c_PCircle2* T(iJ)/(c_sizeLine-1);
                }
            }
            return c_elements;


        }

        template <typename matrix>
        inline int discretize_o0(const T& _length, matrix& _discrPoints) const
        {

            T c_r = std::max(m_cs[0].r(),m_cs[1].r());

            size_t c_NLines = std::ceil((T(2.)*ERL_PI*c_r)/_length);

            Erl::Rotmat<T> c_Rot1 = Erl::Rotmat<T>::Identity();
            Erl::Rotmat<T> c_Rot2 = Erl::Rotmat<T>::Identity();

            Erl::Vector3<T> c_Z(0,0,1);
            Erl::Vector3<T> c_ZcrossN = c_Z.cross(m_cs[0].n());

            if(c_ZcrossN.norm()>PQ::Constants<T>::Zero_Tolerance)
            { c_Rot1 = Erl::Rotmat<T>::fromAngleAxis(atan2(c_ZcrossN.norm(),c_Z.dot(m_cs[0].n())),c_ZcrossN.normalized());}

            c_ZcrossN = c_Z.cross(m_cs[1].n());
            if(c_ZcrossN.norm()>PQ::Constants<T>::Zero_Tolerance)
            { c_Rot2 = Erl::Rotmat<T>::fromAngleAxis(atan2(c_ZcrossN.norm(),c_Z.dot(m_cs[1].n())),c_ZcrossN.normalized());}

            Erl::Angle<T> c_Angle = Erl::Angle<T>::cuttingAngle(c_Rot2.getColumn1(),c_Rot1.getColumn1(),c_ZcrossN);
            if(c_ZcrossN.dot(m_cs[1].n())<0)
            { c_Angle*=-1.;}
            c_Rot2 = Erl::Rotmat<T>::fromAngleAxis(c_Angle.arg(),m_cs[1].n())*c_Rot2;

            size_t c_sizeCont1 = m_cs[0].discretize_size(_length);
            size_t c_sizeCont2 = m_cs[1].discretize_size(_length);
            size_t c_sizeShaft = 0;

            Erl::Vector3<T> c_PCircle (0,0,0);
            Erl::Vector3<T> c_PCircle1(0,0,0);
            Erl::Vector3<T> c_PCircle2(0,0,0);
            for(size_t iI(0);iI<c_NLines;iI++)
            {
                c_PCircle[0] = sin(T(2.*ERL_PI*iI)/c_NLines);
                c_PCircle[1] = cos(T(2.*ERL_PI*iI)/c_NLines);

                c_PCircle1 = m_cs[0].p()+c_Rot1*(m_cs[0].r()*c_PCircle);
                c_PCircle2 = m_cs[1].p()+c_Rot2*(m_cs[1].r()*c_PCircle);

                c_sizeShaft += std::ceil((c_PCircle2-c_PCircle1).norm()/_length)+1;
            }

            size_t c_elements(0);
            c_elements+=c_sizeCont1;
            c_elements+=c_sizeShaft;
            c_elements+=c_sizeCont2;

            if(size_t(_discrPoints.cols()) < c_elements)
                _discrPoints.resize(Eigen::NoChange,c_elements);

            _discrPoints.setZero();
            m_cs[0].discretize_noresize(_length, _discrPoints.block(0,0,3,c_sizeCont1));
            m_cs[1].discretize_noresize(_length, _discrPoints.block(0,c_sizeCont1+c_sizeShaft,3,c_sizeCont2));

            size_t iCol = c_sizeCont1;

            for(size_t iI(0);iI<c_NLines;iI++)
            {
                c_PCircle[0] = sin(T(2.*ERL_PI*iI)/c_NLines);
                c_PCircle[1] = cos(T(2.*ERL_PI*iI)/c_NLines);

                c_PCircle1 =  m_cs[0].p()+c_Rot1*(m_cs[0].r()*c_PCircle);
                c_PCircle2 = (m_cs[1].p()+c_Rot2*(m_cs[1].r()*c_PCircle))-c_PCircle1;

                size_t c_sizeLine = std::ceil(c_PCircle2.norm()/_length)+1;

                for(size_t iJ(0);iJ<c_sizeLine;iJ++)
                {
                    _discrPoints.col(iCol++) = c_PCircle1 + c_PCircle2* T(iJ)/(c_sizeLine-1);
                }
            }
            return c_elements;

        }

        template <typename matrix>
        inline int discretize_o1(const T& _length, matrix& _discrPoints) const
        {

            size_t c_sizeCont1 = m_cs[0].discretize_size(_length);
            size_t c_sizeCont2 = m_cs[1].discretize_size(_length);


            size_t c_Ncircle = std::ceil((m_cs[1].p()-m_cs[0].p()).norm() / _length)+1;
            T r1 = m_cs[0].r();
            T r2 = m_cs[1].r();
            T rc;
            size_t c_sizeShaft(0);


            for(size_t iC(0);iC<c_Ncircle;iC++)
            {
                rc = r1 + (r2-r1)*T(T(iC)/(c_Ncircle-1));
                c_sizeShaft += std::ceil((T(2.)*ERL_PI*rc)/_length);
            }

            size_t c_elements(0);
            c_elements+=c_sizeCont1;
            c_elements+=c_sizeShaft;
            c_elements+=c_sizeCont2;

            if(size_t(_discrPoints.cols()) < c_elements)
                _discrPoints.resize(Eigen::NoChange,c_elements);

            m_cs[0].discretize_noresize(_length, _discrPoints.block(0,0,3,c_sizeCont1));
            m_cs[1].discretize_noresize(_length, _discrPoints.block(0,c_sizeCont1+c_sizeShaft,3,c_sizeCont2));

            Erl::Rotmat<T>  c_RotMat = Erl::Rotmat<T>::Identity();
            Erl::Vector3<T> c_Z(0,0,1);
            Erl::Vector3<T> c_N(m_cs[0].n());

            size_t iCol = c_sizeCont1;

            for(size_t iC(0);iC<c_Ncircle;iC++)
            {
                c_N = m_cs[0].n() + (m_cs[1].n()-m_cs[0].n())*T(T(iC)/(c_Ncircle-1));
                c_N.normalize();

                // c_Viz: Z direction of Instrument
                // c_VtoRCM: Vector from current position to RCM position
                Erl::Vector3<T> c_ZcrossN = c_Z.cross(c_N);			               // c_VZcrossVtoRCM: Cross Product between c_Viz AND c_VtoRCM

                if(c_ZcrossN.norm()>PQ::Constants<T>::Zero_Tolerance)
                { c_RotMat = Erl::Rotmat<T>::fromAngleAxis(atan2(c_ZcrossN.norm(),c_Z.dot(c_N)),c_ZcrossN.normalized());}
                else
                { c_RotMat.setIdentity();}

                Erl::Vector3<T> c_PPlane(0,0,0);
                Erl::Vector3<T> c_CentreCircle = m_cs[0].p() + (m_cs[1].p()-m_cs[0].p())*T(T(iC)/(c_Ncircle-1));

                rc = r1 + (r2-r1)*T(T(iC)/(c_Ncircle-1));
                size_t c_circ = std::ceil((T(2.)*ERL_PI*rc)/_length);
                for(size_t iI(0);iI<c_circ;iI++)
                {
                    c_PPlane[0] = rc*sin((2*ERL_PI*iI)/c_circ);
                    c_PPlane[1] = rc*cos((2*ERL_PI*iI)/c_circ);
                    _discrPoints.col(iCol++) = c_CentreCircle+c_RotMat*c_PPlane;
                }
            }

            return c_elements;
        }


    };
    typedef GCylinder<float>  GCylinderf;
    typedef GCylinder<double> GCylinderd;
    // ****************************************************************************
    template<typename T> class GCylinders
    {
    private:
        std::vector<Contour<T> > m_cs;
        unsigned m_seg_idx[2];
    protected:
    public:
        typedef T type;
        inline GCylinders<T>(const std::vector<Contour<T> > & _contours):m_cs(_contours){m_seg_idx[0]=unsigned(0);m_seg_idx[1]=unsigned((((int)_contours.size())-1)); }
        inline GCylinders<T>(const std::vector<Contour<T> > & _contours,const unsigned & _seg_idx_start,const unsigned & _seg_idx_stop):m_cs(_contours)
        {m_seg_idx[0]=_seg_idx_start;m_seg_idx[1]=_seg_idx_stop;}
        inline Contour<T> const& operator[](unsigned _nIndex) const {
            assert(_nIndex<m_cs.size());
            return m_cs[_nIndex];
        }
        inline Contour<T>& operator[](unsigned _nIndex){
            assert(_nIndex<m_cs.size());
            return m_cs[_nIndex];
        }
        //Copy Constructor
        inline GCylinders<T>(const GCylinders<T> & _s)
            : m_cs(_s)
            , m_seg_idx{_s.m_seg_idx[0],_s.m_seg_idx[1]}
        { }

        //Assignment operator
        inline GCylinders<T>& operator=(const GCylinders<T> & _s)
        {
            m_cs         = _s.m_cs        ;
            m_seg_idx[0] = _s.m_seg_idx[0];
            m_seg_idx[1] = _s.m_seg_idx[1];
            return (*this);
        }
        // Destructor
        inline ~GCylinders<T>(){}

        inline unsigned getSegIdx(const unsigned &_idx ) const {
            assert(_idx<2);
            return m_seg_idx[_idx];
        }
    };
    typedef GCylinders<float>  GCylindersf;
    typedef GCylinders<double> GCylindersd;
}
#endif // ERL_PQ_PRIMITIVES_H
