/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_STRINGCALC_H
#define ERL_STRINGCALC_H

#include "Erl/_forwards.h"

namespace Erl
{

namespace details
{
    template<class T> inline T    expression   (std::string&);
    template<class T> inline T    term         (std::string&);
    template<class T> inline T    factor       (std::string&);
    template<class T> inline T    simple_expr  (std::string&);
    template<class T> inline T    math_function(std::string&);
    template<class T> inline T    string_sign  (std::string&);
    template<class T> inline T    extract_value(std::string&, const int& _base);
                      inline void delete_char  (std::string&, size_t = 1);
                      inline bool starts_with  (const std::string&, const std::string& );

    template<class T> struct str ;
    template<>        struct str<float>
    { static inline float tovalue  (const std::string& _str){return std::stof(_str); }
      static inline float tovalue  (const std::string& _str, size_t& _length){return std::stof(_str,&_length); }
      static inline float toRadiant(      std::string& _str, const float& _value, const int& _del){ delete_char(_str,_del); return Erl::toRadian(_value);} };
    template<>        struct str<double>
    { static inline double tovalue  (const std::string& _str){return std::stod(_str); }
      static inline double tovalue  (const std::string& _str, size_t& _length){return std::stod(_str,&_length); }
      static inline double toRadiant(      std::string& _str, const double& _value, const int& _del){ delete_char(_str,_del); return Erl::toRadian(_value);} };
    template<>        struct str<long double>
    { static inline long double tovalue  (const std::string& _str){return std::stold(_str); }
      static inline long double tovalue  (const std::string& _str, size_t& _length){return std::stold(_str,&_length); }
      static inline long double toRadiant(      std::string& _str, const long double& _value, const int& _del){ delete_char(_str,_del); return Erl::toRadian(_value);} };
    template<>        struct str<int>
    { static inline int tovalue  (const std::string& _str){return std::stoi(_str); }
      static inline int tovalue  (const std::string& _str, size_t& _length){return std::stoi(_str,&_length); }
      static inline int toRadiant(      std::string& , const int& _value, const int&){return _value;} };
    template<>        struct str<unsigned>
    { static inline unsigned tovalue  (const std::string& _str){return std::stoul(_str); }
      static inline unsigned tovalue  (const std::string& _str, size_t& _length){return std::stoul(_str,&_length); }
      static inline unsigned toRadiant(      std::string& , const unsigned& _value, const int&){return _value;} };
    template<>        struct str<long>
    { static inline long tovalue  (const std::string& _str){return std::stol(_str); }
      static inline long tovalue  (const std::string& _str, size_t& _length){return std::stol(_str,&_length); }
      static inline long toRadiant(      std::string& , const long& _value, const int&){return _value;} };
    template<>        struct str<unsigned long>
    { static inline unsigned long tovalue  (const std::string& _str){return std::stoul(_str); }
      static inline unsigned long tovalue  (const std::string& _str, size_t& _length){return std::stoul(_str,&_length); }
      static inline unsigned long tovalue  (const std::string& _str, size_t& _length, const int& _base){return std::stoul(_str,&_length,_base); }
      static inline unsigned long toRadiant(      std::string& , const unsigned long& _value, const int&){return _value;}
    };
    inline void delete_char(std::string& _str, size_t _n )
    {
        if(_str.size()>= _n)
            _str = _str.substr(_n);
    }
    inline bool starts_with(const std::string& _str, const std::string& _comp)
    {
        if(_str.size()>=_comp.size())
        { return ( _str.substr(0,_comp.size()).compare(_comp) == 0);}
        return false;
    }
    template<class T>
    inline T expression(std::string& _str)
    {
        T c_Value;
        if(starts_with(_str,"("))
        {
            delete_char(_str);
            c_Value = string_sign<T>(_str)*term<T>(_str);
            bool c_success(true);

            while (c_success)
            {
                c_success = false;
                if (starts_with(_str,"+"))
                {
                    c_success = true;
                    delete_char(_str);
                    c_Value += term<T>(_str);
                }
                else if (starts_with(_str,"-"))
                {
                    c_success = true;
                    delete_char(_str);
                    c_Value -= term<T>(_str);
                }
            }
            if (starts_with(_str,")"))
            { delete_char(_str); }
            else
            { throw std::invalid_argument("Invalid expression: "+ _str); }
        }
        else
        { throw std::invalid_argument("Invalid expression: "+ _str); }
        return c_Value;
    }
    template<class T>
    inline T term(std::string& _str)
    {
        T c_Value(0);
        c_Value = factor<T>(_str);
        bool c_success(true);
        while (c_success)
        {
            c_success = false;
            if (starts_with(_str,"*"))
            {
                c_success = true;
                delete_char(_str);
                c_Value *= factor<T>(_str);
            }
            else if (starts_with(_str,"/"))
            {
                c_success = true;
                delete_char(_str);
                T c_Value_tmp = factor<T>(_str);
                if(c_Value_tmp == T(0))
                { throw std::invalid_argument("divded by '0': "+_str);}
                else{ c_Value /= c_Value_tmp;}
            }
        }
        return c_Value;
    }
    template<class T>
    inline T  factor(std::string& _str)
    {
         T c_Value(0);
         c_Value = simple_expr<T>(_str);
         while(starts_with(_str,"^"))
         {
             delete_char(_str);
             c_Value = std::pow(c_Value,simple_expr<T>(_str));
         }
         return c_Value;
    }
    template<class T>
    inline T  simple_expr(std::string& _str)
    {
        T c_Value(0);
        if (starts_with(_str,"("))
        { c_Value = expression<T>(_str);}
        else if(starts_with(_str,"sin"  ) ||
                starts_with(_str,"cos"  ) ||
                starts_with(_str,"tan"  ) ||
                starts_with(_str,"ln"   ) ||
                starts_with(_str,"e^"   ) ||
                starts_with(_str,"exp"  ) ||
                starts_with(_str,"asin" ) ||
                starts_with(_str,"acos" ) ||
                starts_with(_str,"atan" ) ||
                starts_with(_str,"lg"   ) ||
                starts_with(_str,"log10") ||
                starts_with(_str,"sqrt" ) )
        { c_Value=math_function<T>(_str); }
        else if(starts_with(_str,"pi") ||
                starts_with(_str,"Pi") ||
                starts_with(_str,"PI") )
        { delete_char(_str,2);
          c_Value = T(ERL_PI);
        }
        else if(starts_with(_str,"x") ||
                starts_with(_str,"X") )
        { delete_char(_str,1);
          c_Value = extract_value<T>(_str,16);
        }
        else if(starts_with(_str,"b") ||
                starts_with(_str,"B") )
        { delete_char(_str,1);
          c_Value = extract_value<T>(_str,2);
        }
        else
        {
            c_Value = extract_value<T>(_str,10);
            if      (starts_with(_str,"deg")) { c_Value = str<T>::toRadiant(_str,c_Value,3);}
            else if (starts_with(_str,"d"  )) { c_Value = str<T>::toRadiant(_str,c_Value,1);}
        }
        return c_Value;
    }
    template<class T>
    inline T extract_value(std::string& _str,const int& _base)
    {
        T c_value;
        try
        {
            size_t c_length;
            if  (_base==10) { c_value= str<   T  >::tovalue(_str,c_length);}
            else            { c_value= str<size_t>::tovalue(_str,c_length,_base); }
            delete_char(_str,c_length);
        }
        catch(const std::invalid_argument& _e)
        { throw std::invalid_argument("str to value failed: "+ _str+", what: "+_e.what());}
        return c_value;
    }
    template<class T>
    inline T math_function(std::string& _str)
    {
        if(starts_with(_str,"sin"))
        { delete_char(_str,3); return std::sin(expression<T>(_str));}
        if(starts_with(_str,"cos"))
        { delete_char(_str,3); return std::cos(expression<T>(_str));}
        if(starts_with(_str,"tan"))
        { delete_char(_str,3); return std::tan(expression<T>(_str));}
        if(starts_with(_str,"asin"))
        { delete_char(_str,4); return std::asin(expression<T>(_str)); }
        if(starts_with(_str,"acos"))
        { delete_char(_str,4); return std::acos(expression<T>(_str));}
        if(starts_with(_str,"atan"))
        { delete_char(_str,4); return std::atan(expression<T>(_str));}
        if(starts_with(_str,"e^"))
        { delete_char(_str,2); return std::exp(expression<T>(_str));}
        if(starts_with(_str,"exp"))
        { delete_char(_str,3); return std::exp(expression<T>(_str));}
        if(starts_with(_str,"ln"))
        { delete_char(_str,2); return std::log(expression<T>(_str));}
        if(starts_with(_str,"lg"))
        { delete_char(_str,2); return std::log10(expression<T>(_str));}
        if(starts_with(_str,"log10"))
        { delete_char(_str,5); return std::log10(expression<T>(_str));}
        if(starts_with(_str,"sqrt"))
        { delete_char(_str,4); return std::sqrt(expression<T>(_str));}
        throw std::invalid_argument("unknown function: "+_str);
        return 0;
    }
    template<class T>
    inline T string_sign(std::string& _str)
    {
        bool c_success(true);
        T c_sign(1);
        while(c_success)
        {
            c_success = false;
            if     (starts_with(_str,"+")){delete_char(_str); c_success = true; }
            else if(starts_with(_str,"-")){delete_char(_str); c_success = true; c_sign*=T(-1);}
        }
        return c_sign;
    }

}
template <class T>
inline T calculate_string(const std::string& _string)
{
    std::string c_expression("("+_string+")");
    return details::expression<T>(c_expression);
}

}

#endif // ERL_STRINGCALC_H
