/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_STATIC_H
#define ERL_STATIC_H

#include "Erl/_forwards.h"

namespace Erl
{

/// Static FOR loop --- Variant 1
///
/// Erl::static_for<0,10,3>()([&](int i){
///    std::cout<<i<<"   "<<std::flush;
/// });
///
/// Static FOR loop --- Variant 2
///
/// template<int i> int IValue(){ return i; }
/// auto c_tuple = std::make_tuple(float(1.),int(2),double(3.));
/// constexpr size_t tuple_size = std::tuple_size<decltype(c_tuple)>::value;
/// Erl::static_for<0,tuple_size>()([&](auto&& tup_i, auto &&i)
/// {
///     typedef typename std::decay<decltype(i)>::type IT;
///     std::cout << "Element "<<IValue<IT::value>()<<" is of type: "<< typeid(tup_i).name() << ", and value: "<<tup_i<<std::endl;
/// },
/// c_tuple);

template <int First, int Last, int Inc = 1>
struct static_for
{
    template <typename Fn, typename ...Tup>
    inline void operator()(Fn&& fn, Tup&&... tup) const noexcept
    {
        static_assert((Inc!= 0)               ,"Inc!=0 required.");
        static_assert((Inc < 0)||(First<=Last),"When Inc>0,  First < Last required");
        static_assert((Inc > 0)||(Last<=First),"When Inc<0,  First > Last required");

        fn(std::get<First>(std::forward<Tup>(tup))..., std::integral_constant<int,First>() ); /// Call Function
        static_for< First+Inc,  /// Recursive Call
                     (Inc>0)*(First+Inc*(1+((Last-First-1)/ Inc))) /// Positive Increments
                    +(Inc<0)*(First+Inc*(1+((First-Last-1)/-Inc))),/// Negative Increments
                    Inc>()(std::forward<Fn>(fn),std::forward<Tup>(tup)...);
    }
};

template <int Last, int Inc>
struct static_for<Last,Last,Inc>
{
    /// Termination Criteria
    template <typename Fn, typename ...Tup> inline void operator()(Fn&&, Tup&&... ) const noexcept{ }
};

///  Static IF
///
///  double i = 10.4;
///  Erl::static_if<true>()([&i]{
///      std::cout<<i<<" "<<std::flush;
///  });std::cout<<std::endl;
///  i =10.2;
///  Erl::static_if<false>()([&i]{
///      std::cout<<i<<" "<<std::flush;
///  });std::cout<<std::endl;

template <bool valid>
struct static_if
{
    /// Call FunctionPointer with parameters
    template <typename Fn, typename... Args> void operator()(Fn&& fn, Args&&... args) const
    { fn(std::forward<Args>(args)...); }
};

template <>
struct static_if<false>
{
    /// Partial Specialisation - Don't call function at false
    template <typename Fn, typename... Args> void operator()(Fn&&, Args&&...) const {}
};

template <bool valid>
struct static_if_else
{
    /// Call Function - Call if function
    template <typename FnIf,typename FnElse, typename... Args> auto operator()(FnIf&& fnif, FnElse&&, Args&&... args) const
    -> decltype(fnif(std::forward<Args>(args)...))
    { return fnif(std::forward<Args>(args)...); }
};

template <>
struct static_if_else<false>
{
    /// Partial Specialisation - Call else function
    template <typename FnIf,typename FnElse, typename... Args> auto operator()(FnIf&&, FnElse&& fnelse, Args&&... args) const
    -> decltype(fnelse(std::forward<Args>(args)...))
    { return fnelse(std::forward<Args>(args)...); }
};

}

#endif // ERL_STATIC_H
