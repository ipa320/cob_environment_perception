// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_PARALLELIZER_DECL_H
#define NUKLEI_PARALLELIZER_DECL_H

#include <nuklei/Random.h>
#include <nuklei/Common.h>
#include <nuklei/BoostSerialization.h>

#include <cstdlib>
#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace nuklei {
  
  struct parallelizer
  {
    typedef enum { OPENMP = 0, FORK, PTHREAD, SINGLE, UNKNOWN } Type;
    static const Type defaultType = OPENMP;
    static const std::string TypeNames[];
    
    /**
     * If chosing the fork()-based implementation, make sure that your program
     * consists of a single thread at the time run() is called, or you will run
     * into problems (Google "forking a multithreaded program" to see why).
     */
    parallelizer(const int n,
                 const Type& type = OPENMP,
                 const unsigned long seed = 0) :
    n_(n), type_(type), seed_(seed) {}
    
    struct na_print_accessor
    {
      template<typename T>
      std::string operator()(const T& t)
      {
        return "n/a";
      }
    };

    struct print_accessor
    {
      template<typename T>
      const T& operator()(const T& t)
      {
        return t;
      }
    };
    
    template
    <typename R, typename Callable>
    std::vector<R> run(Callable callable) const
    {
      return run<R>(callable, na_print_accessor());
    }

    template
    <typename R, typename Callable, typename PrintAccessor>
    std::vector<R> run(Callable callable,
                       PrintAccessor pa) const
    {
      switch (type_)
      {
        case OPENMP:
          return run_openmp<R>(callable, pa);
          break;
        case FORK:
          return run_fork<R>(callable, pa);
          break;
        case PTHREAD:
          return run_pthread<R>(callable, pa);
          break;
        case SINGLE:
          return run_single<R>(callable, pa);
          break;
        default:
          NUKLEI_THROW("Unknown parallelization method.");
      }
      return std::vector<R>();
    }
    
  private:
    
    template<typename R, typename Callable, typename PrintAccessor>
    std::vector<R> run_openmp(Callable callable,
                              PrintAccessor pa) const;
    
    template<typename R, typename Callable, typename PrintAccessor>
    std::vector<R> run_fork(Callable callable,
                            PrintAccessor pa) const;
    
    template<typename R, typename Callable>
    struct pthread_wrapper
    {
      pthread_wrapper(Callable callable) : callable_(callable) {}
      void operator()(R& ret)
      {
        ret = callable_();
      }
    private:
      Callable callable_;
    };
    
    template<typename R, typename Callable, typename PrintAccessor>
    std::vector<R> run_pthread(Callable callable,
                               PrintAccessor pa) const;
    
    template<typename R, typename Callable, typename PrintAccessor>
    std::vector<R> run_single(Callable callable,
                              PrintAccessor pa) const;
    
    static void reap();
    
    int n_;
    Type type_;
    unsigned long seed_;
  };
  
}

#endif
