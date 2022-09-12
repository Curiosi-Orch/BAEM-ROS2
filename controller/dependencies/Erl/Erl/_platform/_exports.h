/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_PLATFORM_H
#define ERL_PLATFORM_H

#if !defined( ERL_BUILD_SYMBOLS )
    #ifdef WIN32
        #define _ERL_EXPORT __declspec( dllimport )
    #else
        #define _ERL_EXPORT
    #endif
#elif defined( ERL_STATIC_BUILD )
    #define _ERL_EXPORT
#else
    #if defined(WIN32) && !defined( __MINGW32__ )
        #define _ERL_EXPORT __declspec( dllexport )
    #else
        #define _ERL_EXPORT
    #endif
#endif

#endif
