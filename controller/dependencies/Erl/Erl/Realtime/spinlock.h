/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_SPINLOCK_H
#define ERL_SPINLOCK_H

#include <atomic>

namespace Erl
{
namespace RT
{

class Spinlock
{

private:

  std::atomic_flag lockFlag;

public:

  Spinlock()
  {
      lockFlag.clear();
  }

  inline void lock()
  {
      while (lockFlag.test_and_set(std::memory_order_acquire))
          ;
  }
  inline bool try_lock()
  {
      return !lockFlag.test_and_set(std::memory_order_acquire);
  }
  inline void unlock()
  {
    lockFlag.clear(std::memory_order_release);
  }

};
}
}
#endif
