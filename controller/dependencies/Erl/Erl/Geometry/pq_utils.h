/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_PQ_UTILS_H
#define ERL_PQ_UTILS_H

#include <Erl.h>


namespace Erl {

#define RM_MAX(a,b) ((a) > (b) ? (a) : (b))
#define RM_MIN(a,b) ((a) < (b) ? (a) : (b))

// ************** FUNCTIONS
template<typename T> inline void ComplementOrthoNormalBasis(const Vector3<T> & _in_w, Vector3<T>& _out_u, Vector3<T>& _out_v )
{
    T invL;

    if (fabs(_in_w[0]) >= fabs(_in_w[1]))
    {
        // _in_w.x or _in_w.z is the largest magnitude component, swap them
        invL = T(1)/sqrt(_in_w[0]*_in_w[0] +
            _in_w[2]*_in_w[2]);

        _out_u[0] = -_in_w[2]*invL;
        _out_u[1] = T(0);
        _out_u[2] = +_in_w[0]*invL;

        _out_v[0] = _in_w[1]*_out_u[2];
        _out_v[1] = _in_w[2]*_out_u[0] - _in_w[0]*_out_u[2];
        _out_v[2] = -_in_w[1]*_out_u[0];
    }
    else
    {
        // _in_w.y or _in_w.z is the largest magnitude component, swap them
        invL = T(1)/sqrt(_in_w[1]*_in_w[1] +
            _in_w[2]*_in_w[2]);
        _out_u[0] = T(0);
        _out_u[1] = +_in_w[2]*invL;
        _out_u[2] = -_in_w[1]*invL;

        _out_v[0] = _in_w[1]*_out_u[2] - _in_w[2]*_out_u[1];
        _out_v[1] = -_in_w[0]*_out_u[2];
        _out_v[2] = _in_w[0]*_out_u[1];
    }

}
template<typename T> inline unsigned root_poly2(const T (&in_coeff)[3], T (&out_root)[2]){
    return root_poly2(in_coeff,out_root);
}
template<typename T> inline unsigned root_poly2(T const * const in_coeff, T * const out_root){

    if (in_coeff[0] == 0) /* Handle linear case */
      {
        if (in_coeff[1] == 0)
          {
            return 0;
          }
        else
          {
            out_root[0] = -in_coeff[2] / in_coeff[1];
            return 1;
          };
      }

    {
      T disc = in_coeff[1] * in_coeff[1] - 4 * in_coeff[0] * in_coeff[2];

      if (disc > 0)
        {
          if (in_coeff[1] == 0)
            {
              T r = sqrt (-in_coeff[2] / in_coeff[0]);
              out_root[0] = -r;
              out_root[1] =  r;
            }
          else
            {
              T temp = -0.5 * (in_coeff[1] + (in_coeff[1] > 0 ? 1 : -1) * sqrt (disc));
              T r1 = temp / in_coeff[0] ;
              T r2 = in_coeff[2] / temp ;

              if (r1 < r2)
                {
                  out_root[0] = r1 ;
                  out_root[1] = r2 ;
                }
              else
                {
                  out_root[0] = r2 ;
                  out_root[1] = r1 ;
                }
            }
          return 2;
        }
      else if (disc == 0)
        {
          out_root[0] = -0.5 * in_coeff[1] / in_coeff[0] ;
          out_root[1] = -0.5 * in_coeff[1] / in_coeff[0] ;
          return 2 ;
        }
      else
        {
          return 0;
        }
    }
}
template<typename T> inline unsigned root_poly3(const T (&in_coeff)[3], T (&out_root)[3]){
    return root_poly3(in_coeff,out_root);
}
template<typename T> inline unsigned root_poly3(T const * const in_coeff, T * const out_root){
    T q(in_coeff[0] * in_coeff[0] - 3 * in_coeff[1]);
    T r(2 * in_coeff[0] * in_coeff[0] * in_coeff[0] - 9 * in_coeff[0] * in_coeff[1] + 27 * in_coeff[2]);

      T Q = q / 9;
      T R = r / 54;

      T Q3 = Q * Q * Q;
      T R2 = R * R;
      T c_CR2(729*r*r);
      T CQ3(2916*q*q*q);

      if (R == 0 && Q == 0)
        {
          out_root[0] = - in_coeff[0] / 3 ;
          out_root[1] = - in_coeff[0] / 3 ;
          out_root[2] = - in_coeff[0] / 3 ;
          return 3 ;
        }
      else if (c_CR2 == CQ3)
        {
          /* this test is actually R2 == Q3, written in a form suitable
             for exact computation with integers */

          /* Due to finite precision some double roots may be missed, and
             considered to be a pair of complex roots z = x +/- epsilon i
             close to the real axis. */

          T sqrtQ = sqrt (Q);

          if (R > 0)
            {
              out_root[0] = -2 * sqrtQ  - in_coeff[0] / 3;
              out_root[1] = sqrtQ - in_coeff[0] / 3;
              out_root[2] = sqrtQ - in_coeff[0] / 3;
            }
          else
            {
              out_root[0] = - sqrtQ  - in_coeff[0] / 3;
              out_root[1] = - sqrtQ - in_coeff[0] / 3;
              out_root[2] = 2 * sqrtQ - in_coeff[0] / 3;
            }
          return 3 ;
        }
      else if (R2 < Q3)
        {
          T sgnR = (R >= 0 ? 1 : -1);
          T ratio = sgnR * sqrt (R2 / Q3);
          T theta = acos (ratio);
          T norm = -2 * sqrt (Q);
          out_root[0]= norm * cos (theta / 3) - in_coeff[0] / 3;
          out_root[1] = norm * cos ((theta + 2.0 * M_PI) / 3) - in_coeff[0] / 3;
          out_root[2] = norm * cos ((theta - 2.0 * M_PI) / 3) - in_coeff[0] / 3;

          /* Sort *x0, *x1, *x2 into increasing order

          if (*x0 > *x1)
            SWAP(*x0, *x1) ;

          if (*x1 > *x2)
            {
              SWAP(*x1, *x2) ;

              if (*x0 > *x1)
                SWAP(*x0, *x1) ;
            }*/

          return 3;
        }
      else
        {
          T sgnR = (R >= 0 ? 1 : -1);
          T A = -sgnR * pow (fabs (R) + sqrt (R2 - Q3), 1.0/3.0);
          T B = Q / A ;
          out_root[0] = A + B - in_coeff[0] / 3;
          return 1;
        }

}
template<typename T> inline unsigned root_poly4(const T (&in_coeff)[4], T (&out_root)[4]){
    return root_poly4(in_coeff,out_root);
}
template<typename T> inline unsigned root_poly4(T const * const in_coeff, T * const out_root){
    /*
       * This code is based on a simplification of
       * the algorithm from zsolve_quartic.c for real roots
       */
      T u[3];
      T aa, pp, qq, rr, rc, sc, tc, mt;
      T w1r, w1i, w2r, w2i, w3r;
      T v[3], v1, v2, arg, theta;
      T disc, h;
      int k1(0), k2(0);
      T zarr[4];

      /* Deal easily with the cases where the quartic is degenerate. The
       * ordering of solutions is done explicitly. */
      if (0 == in_coeff[1] && 0 == in_coeff[2])
        {
          if (0 == in_coeff[3])
            {
              if (in_coeff[0] > 0)
                {
                  out_root[0] = -in_coeff[0];
                  out_root[1] = 0.0;
                  out_root[2] = 0.0;
                  out_root[3] = 0.0;
                }
              else
                {
                  out_root[0] = 0.0;
                  out_root[1] = 0.0;
                  out_root[2] = 0.0;
                  out_root[3] = -in_coeff[0];
                }
              return 4;
            }
          else if (0 == in_coeff[0])
            {
              if (in_coeff[3] > 0)
                {
                  return 0;
                }
              else
                {
                  out_root[1] = sqrt (sqrt (-in_coeff[3]));
                  out_root[0] = -(out_root[1]);
                  return 2;
                }
            }
        }

      if (0.0 == in_coeff[2] && 0.0 == in_coeff[3])
        {
          out_root[0]=0.0;
          out_root[1]=0.0;

          //if (gsl_poly_solve_quadratic(1.0,in_coeff[0],in_coeff[1],out_root[2],out_root[3])==0) {
          if (root_poly2(in_coeff,out_root+2)==0) {
        mt=3;
          } else {
        mt=1;
          }
        }
      else
        {
          /* For non-degenerate solutions, proceed by constructing and
           * solving the resolvent cubic */
          aa = in_coeff[0] * in_coeff[0];
          pp = in_coeff[1] - (3.0/8.0) * aa;
          qq = in_coeff[2] - (1.0/2.0) * in_coeff[0] * (in_coeff[1] - (1.0/4.0) * aa);
          rr = in_coeff[3] - (1.0/4.0) * (in_coeff[0] * in_coeff[2] - (1.0/4.0) * aa * (in_coeff[1] - (3.0/16.0) * aa));
          rc = (1.0/2.0) * pp;
          sc = (1.0/4.0) * ((1.0/4.0) * pp * pp - rr);
          tc = -((1.0/8.0) * qq * (1.0/8.0) * qq);

          /* This code solves the resolvent cubic in a convenient fashion
           * for this implementation of the quartic. If there are three real
           * roots, then they are placed directly into u[].  If two are
           * complex, then the real root is put into u[0] and the real
           * and imaginary part of the complex roots are placed into
           * u[1] and u[2], respectively. Additionally, this
           * calculates the discriminant of the cubic and puts it into the
           * variable disc. */
          {
        T qcub(rc * rc - 3 * sc);
        T rcub(2 * rc * rc * rc - 9 * rc * sc + 27 * tc);

        T Q = qcub / 9;
        T R = rcub / 54;

        T Q3 = Q * Q * Q;
        T R2 = R * R;

        T c_CR2(T(729)*rcub*rcub);
        T CQ3(T(2916)*qcub*qcub*qcub);

        disc = (c_CR2 - CQ3) / 2125764.0;

        if (0 == R && 0 == Q)
          {
            u[0] = -rc / 3;
            u[1] = -rc / 3;
            u[2] = -rc / 3;
          }
        else if (c_CR2 == CQ3)
          {
            T sqrtQ = sqrt (Q);
            if (R > 0)
              {
            u[0] = -2 * sqrtQ - rc / 3;
            u[1] = sqrtQ - rc / 3;
            u[2] = sqrtQ - rc / 3;
              }
            else
              {
            u[0] = -sqrtQ - rc / 3;
            u[1] = -sqrtQ - rc / 3;
            u[2] = 2 * sqrtQ - rc / 3;
              }
          }
        else if (c_CR2 < CQ3)
          {
            T sqrtQ = sqrt (Q);
            T sqrtQ3 = sqrtQ * sqrtQ * sqrtQ;
            T theta = acos (R / sqrtQ3);
            if (R / sqrtQ3 >= 1.0) theta = 0.0;
            {
              T norm = -2 * sqrtQ;

              u[0] = norm * cos (theta / 3) - rc / 3;
              u[1] = norm * cos ((theta + 2.0 * M_PI) / 3) - rc / 3;
              u[2] = norm * cos ((theta - 2.0 * M_PI) / 3) - rc / 3;
            }
          }
        else
          {
            T sgnR = (R >= 0 ? 1 : -1);
            T modR = fabs (R);
            T sqrt_disc = sqrt (R2 - Q3);
            T A = -sgnR * pow (modR + sqrt_disc, 1.0 / 3.0);
            T B = Q / A;
            T mod_diffAB = fabs (A - B);

            u[0] = A + B - rc / 3;
            u[1] = -0.5 * (A + B) - rc / 3;
            u[2] = -(sqrt (3.0) / 2.0) * mod_diffAB;
          }
          }
          /* End of solution to resolvent cubic */

          /* Combine the square roots of the roots of the cubic
           * resolvent appropriately. Also, calculate 'mt' which
           * designates the nature of the roots:
           * mt=1 : 4 real roots (disc == 0)
           * mt=2 : 0 real roots (disc < 0)
           * mt=3 : 2 real roots (disc > 0)
           */

          if (0.0 == disc)
        u[2] = u[1];

          if (0 >= disc)
        {
          mt = 2;

          /* One would think that we could return 0 here and exit,
           * since mt=2. However, this assignment is temporary and
           * changes to mt=1 under certain conditions below.
           */

          v[0] = fabs (u[0]);
          v[1] = fabs (u[1]);
          v[2] = fabs (u[2]);

          v1 = RM_MAX(RM_MAX(v[0], v[1]), v[2]);
          /* Work out which two roots have the largest moduli */
          k1 = 0, k2 = 0;
          if (v1 == v[0])
            {
              k1 = 0;
              v2 = RM_MAX (v[1], v[2]);
            }
          else if (v1 == v[1])
            {
              k1 = 1;
              v2 = RM_MAX (v[0], v[2]);
            }
          else
            {
              k1 = 2;
              v2 = RM_MAX (v[0], v[1]);
            }

          if (v2 == v[0])
            {
              k2 = 0;
            }
          else if (v2 == v[1])
            {
              k2 = 1;
            }
          else
            {
              k2 = 2;
            }

          if (0.0 <= u[k1])
            {
              w1r=sqrt(u[k1]);
              w1i=0.0;
            }
          else
            {
              w1r=0.0;
              w1i=sqrt(-u[k1]);
            }
          if (0.0 <= u[k2])
            {
              w2r=sqrt(u[k2]);
              w2i=0.0;
            }
          else
            {
              w2r=0.0;
              w2i=sqrt(-u[k2]);
            }
        }
          else
        {
          mt = 3;

          if (0.0 == u[1] && 0.0 == u[2])
            {
              arg = 0.0;
            }
          else
            {
              arg = sqrt(sqrt(u[1] * u[1] + u[2] * u[2]));
            }
          theta = atan2(u[2], u[1]);

          w1r = arg * cos(theta / 2.0);
          w1i = arg * sin(theta / 2.0);
          w2r = w1r;
          w2i = -w1i;
        }

          /* Solve the quadratic to obtain the roots to the quartic */
          w3r = qq / 8.0 * (w1i * w2i - w1r * w2r) /
        (w1i * w1i + w1r * w1r) / (w2i * w2i + w2r * w2r);
          h = in_coeff[0] / 4.0;

          zarr[0] = w1r + w2r + w3r - h;
          zarr[1] = -w1r - w2r + w3r - h;
          zarr[2] = -w1r + w2r - w3r - h;
          zarr[3] = w1r - w2r - w3r - h;

          /* Arrange the roots into the variables z0, z1, z2, z3 */
          if (2 == mt)
            {
              if (u[k1] >= 0 && u[k2] >= 0)
                {
                  mt = 1;
              out_root[0] = zarr[0];
              out_root[1] = zarr[1];
              out_root[2] = zarr[2];
              out_root[3] = zarr[3];
                }
          else
            {
              return 0;
            }
        }
          else
            {
          out_root[0] = zarr[0];
          out_root[1] = zarr[1];
            }
        }

      /* Sort the roots as usual */
      if (1 == mt)
          return 4;

      return 2;
        /* {
          Roots are all real, sort them by the real part *//*
          if (*out_root[0] > *out_root[1])
            SWAPD (*out_root[0], *out_root[1]);
          if (*out_root[0] > *out_root[2])
            SWAPD (*out_root[0], *out_root[2]);
          if (*out_root[0] > *out_root[3])
            SWAPD (*out_root[0], *out_root[3]);

          if (*out_root[1] > *out_root[2])
            SWAPD (*out_root[1], *out_root[2]);
          if (*out_root[2] > *out_root[3])
            {
              SWAPD (*out_root[2], *out_root[3]);
              if (*out_root[1] > *out_root[2])
                SWAPD (*out_root[1], *out_root[2]);
            }
          return 4;
        }
      else
        {
          // 2 real roots
          if (*out_root[0] > *out_root[1])
            SWAPD (*out_root[0], *out_root[1]);
        }

      return 2;*/

}
}




#endif // ERL_PQ_UTILS_H
