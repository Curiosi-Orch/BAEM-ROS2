/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_CIRCULARBUFFER_H
#define ERL_CIRCULARBUFFER_H

#include <vector>
#include <chrono>

namespace Erl
{

template <typename T>
class CircularBuffer
{

public:

    CircularBuffer (const unsigned& size, const T& defaultValue = T())
        : data_(size, defaultValue)
        , index_(0)
    {}

    inline virtual void set(const T& value)
    {
        data_[index_] = value;
        index_++;
        if (index_ == data_.size())
            index_ = 0;
    }

    inline unsigned size() const { return data_.size(); }

    inline T operator [] (unsigned i) const
    {
        unsigned readIndex = index_ + i;
        if (readIndex >= data_.size())
            readIndex -= data_.size();
        return data_[readIndex];
    }

    inline T first () const
    {
        return data_[index_];
    }

    inline T last () const
    {
        if (index_ == 0)
            return data_[data_.size() - 1];
        else
            return data_[index_ - 1];
    }

protected:

    std::vector<T> data_;
    unsigned index_;

};

template <typename T>
class TimedCircularBuffer : public CircularBuffer<T>
{

public:

    inline TimedCircularBuffer (const unsigned& size, const T& defaultValue = T())
        : CircularBuffer<T>(size, defaultValue)
        , timestamps_(size, -1)
        , start_(std::chrono::high_resolution_clock::now())
    {}

    inline virtual void set(const T& value)
    {
        this->data_[this->index_] = value;
        t_ = std::chrono::high_resolution_clock::now() - start_;
        timestamps_[this->index_] = t_.count();

        this->index_++;
        if (this->index_ == this->data_.size())
            this->index_ = 0;
    }

    inline std::vector<T> getPrevious_ns(const int64_t& ns) const
    {
        t_ = std::chrono::high_resolution_clock::now() - start_;
        int64_t t_curr = t_.count();

        unsigned startIndex;
        if (this->index_ == 0)
            startIndex = this->data_.size() - 1;
        else
            startIndex = this->index_ - 1;
        unsigned index = startIndex;
        for (unsigned i = 0; i < this->data_.size() - 1; i++)
        {
            if ( t_curr - timestamps_[index] > ns )
                break;
            if (index == 0)
                index = this->data_.size() - 1;
            else
                index--;
        }


        if (index == startIndex)
        {
            return std::vector<T>();
        }
        else
        {
            unsigned beginIndex = index + 1;
            if (beginIndex == this->data_.size())
                beginIndex = 0;

            if (beginIndex <= startIndex)
            {
                std::vector<T> ret(startIndex - beginIndex + 1);
                for (unsigned i = beginIndex; i <= startIndex; i++)
                    ret[i - beginIndex] = this->data_[i];
                return ret;
            }
            else
            {
                std::vector<T> ret( (startIndex + 1) + (this->data_.size() - beginIndex) );
                unsigned counter = 0;
                for (unsigned i = beginIndex; i < this->data_.size(); i++)
                {
                    ret[counter] = this->data_[i];
                    counter++;
                }
                for (unsigned i = 0; i <= startIndex; i++)
                {
                    ret[counter] = this->data_[i];
                    counter++;
                }
                return ret;
            }
        }

    }

    inline T getNearestTo_ns(const int64_t& ns) const
    {
        t_ = std::chrono::high_resolution_clock::now() - start_;
        int64_t t_curr = t_.count();

        unsigned startIndex;
        if (this->index_ == 0)
            startIndex = this->data_.size() - 1;
        else
            startIndex = this->index_ - 1;
        unsigned index = startIndex;
        for (unsigned i = 0; i < this->data_.size() - 1; i++)
        {
            if ( t_curr - timestamps_[index] > ns )
                break;
            if (index == 0)
                index = this->data_.size() - 1;
            else
                index--;
        }

        if (index == startIndex)
            return this->data_[startIndex];
        index++;
        if (index == this->data_.size())
            return this->data_[0];
        else
            return this->data_[index];
    }

    inline T getLatest() const
    {
        return this->last();
    }

    inline T getOldest() const
    {
        return this->first();
    }

    inline std::vector<T> getPrevious_us(const int64_t& us) const
    {
        return getPrevious_ns(us * (int64_t)1000);
    }

    inline std::vector<T> getPrevious_ms(const int64_t& ms) const
    {
        return getPrevious_ns(ms * (int64_t)1000000);
    }

    inline T getNearestTo_us(const int64_t& us) const
    {
        return getNearestTo_ns(us * (int64_t)1000);
    }

    inline T getNearestTo_ms(const int64_t& ms) const
    {
        return getNearestTo_ns(ms * (int64_t)1000000);
    }

protected:

    std::vector<int64_t> timestamps_;
    std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> start_;
    mutable std::chrono::nanoseconds t_;

};

}

#endif
