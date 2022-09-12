/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_TRANSFORM_H
#define ERL_TRANSFORM_H

#include <Eigen/Core>
#include "Erl/_forwards.h"

#define ERL_MAJOR_TYPE_GENERAL 1
#define ERL_MAJOR_TYPE_COL     2
#define ERL_MAJOR_TYPE_ROW     3

#define ERL_MAJOR_TYPE ERL_MAJOR_TYPE_GENERAL
#include "_transformCore.h"
#define ERL_MAJOR_TYPE ERL_MAJOR_TYPE_COL
#include "_transformCore.h"
#define ERL_MAJOR_TYPE ERL_MAJOR_TYPE_ROW
#include "_transformCore.h"
#undef  ERL_MAJOR_TYPE

namespace Erl
{

typedef Transform<double> Transformd;
typedef Transform<float>  Transformf;

}

#endif
