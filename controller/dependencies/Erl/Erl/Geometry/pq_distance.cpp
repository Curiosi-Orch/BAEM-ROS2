/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#include "pq_distance.h"

namespace PQ{
using namespace RM;

template <> const float  CONSTANT<float>::ZERO_TOLERANCE  = 1e-06f;
template <> const float  CONSTANT<float>::MAX_REAL        = std::numeric_limits<float>::max(); //FLT_MAX;
template <> const double CONSTANT<double>::ZERO_TOLERANCE = 1e-08;
template <> const double CONSTANT<double>::MAX_REAL       = std::numeric_limits<double>::max(); //DBL_MAX;
}


