/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#if _MSC_VER && !__INTEL_COMPILER

#define ERL_PRAGMA(x) __pragma(x)

#define ERL_RMV_UNPARA_BEGIN    ERL_PRAGMA(warning(push)) \
                                ERL_PRAGMA(warning(disable: 4100))
#define ERL_RMV_UNPARA_END      ERL_PRAGMA(warning(pop))
#define ERL_RMV_UNPARA_SINGLE   ERL_PRAGMA(warning(suppress: 4100))
#define ERL_RMV_UNVAR_SINGLE    ERL_PRAGMA(warning(suppress: 4101))

#define ERL_RMV_UNINIT_SINGLE   ERL_PRAGMA(warning(suppress: 4701))


#elif __GNUC__

#define ERL_PRAGMA(x) _Pragma(#x)

#define ERL_RMV_UNPARA_BEGIN    ERL_PRAGMA(GCC diagnostic push) \
                                ERL_PRAGMA(GCC diagnostic ignored "-Wunused-parameter")
#define ERL_RMV_UNPARA_END      ERL_PRAGMA(GCC diagnostic pop)
#define ERL_RMV_UNPARA_SINGLE   ERL_PRAGMA(GCC diagnostic ignored "-Wunused-parameter")
#define ERL_RMV_UNVAR_SINGLE    ERL_PRAGMA(GCC diagnostic ignored "-Wunused-variable")

    #ifdef  __clang__
        #define ERL_RMV_UNINIT_SINGLE   ERL_PRAGMA(GCC diagnostic ignored "-Wconditional-uninitialized")
    #else
        #define ERL_RMV_UNINIT_SINGLE   ERL_PRAGMA(GCC diagnostic ignored "-Wmaybe-uninitialized")
    #endif
#else
    #define ERL_RMV_UNPARA_BEGIN
    #define ERL_RMV_UNPARA_END
    #define ERL_RMV_UNPARA_SINGLE
    #define ERL_RMV_UNVAR_SINGLE
    #define ERL_RMV_UNINIT_SINGLE
#endif

#if _MSC_VER && (_MSC_VER < 1900 && !__INTEL_COMPILER)
    static_assert(false, "THIS COMPILER DOES NOT SUPPORT SOME NECESSARY C++11 FEATURES, ERL DOES NOT SUPPORT IT.");
#else
    #ifndef ERL_USE_CONSTEXPR
        #define ERL_CONSTEXPR_ACTIVE ERL_TRUE
        #define ERL_USE_CONSTEXPR constexpr
    #endif
#endif
