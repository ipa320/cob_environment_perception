// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_COMMON_H
#define NUKLEI_COMMON_H

#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <stdexcept>
#include <vector>

#include <nuklei/Definitions.h>
#include <nuklei/Log.h>
#include <nuklei/LoggingMacros.h>
#include <nuklei/LinearAlgebraTypes.h>

/** @brief Public namespace. */
namespace nuklei {
  
  // When using the macros below, remember that destructors should never
  // throw!
  
  /**
   * @brief Name-value pair.
   *
   * Example usage: NUKLEI_THROW( NUKLEI_NVP(variable) );
   */
# define NUKLEI_NVP(x) #x << "=" << x
  
  /**
   * @brief Throws an Error.
   *
   * Example usage: NUKLEI_THROW("int i is equal to " << i);
   *
   * Note that if a function throws, calling it is slow. Do not throw from
   * functions that are supposed to execute fast.
   *
   * Do not use from within a destructor!
   */
# define NUKLEI_THROW(x) \
{ \
std::ostringstream oss; \
oss << x; \
throw nuklei::Error(oss.str()); \
}
  
  /**
   * @brief Throws an Error if expression is not true.
   *
   * Example usage: NUKLEI_ASSERT(i != 0);
   *
   * Note that if a function throws, calling it is slow. Do not throw from
   * functions that are supposed to execute fast.
   *
   * For functions that have to go fast, use NUKLEI_FAST_ASSERT instead.
   *
   * Do not use from within a destructor!
   */
# define NUKLEI_ASSERT(expression) \
{ \
if (!(expression)) \
throw nuklei::Error \
(std::string(__FILE__) + \
":" + nuklei::stringify(__LINE__) + \
": failed to assert `" + #expression + "'"); \
}
  
# ifdef NDEBUG
#   define NUKLEI_DEBUG_ASSERT(expression) ((void)0)
# else
#   define NUKLEI_DEBUG_ASSERT NUKLEI_ASSERT
# endif
  
  // Throwing slows down function calls.
# define NUKLEI_FAST_ASSERT(expression) \
assert(expression)
  
# ifdef NDEBUG
#   define NUKLEI_FAST_DEBUG_ASSERT(expression) ((void)0)
# else
#   define NUKLEI_FAST_DEBUG_ASSERT NUKLEI_FAST_ASSERT
# endif
  
# define NUKLEI_ASSERT_AFE(e1, e2) \
{ \
double tol = 1e-6; \
if (!(nuklei::afe(e1, e2, tol))) \
NUKLEI_THROW(nuklei::stringify(e1, 20, 0) << " != " << nuklei::stringify(e2, 20, 0) << " (TOL=" << tol << ")"); \
}
  
# define NUKLEI_ASSERT_AFE_TOL(e1, e2, tol) \
{ \
if (!(nuklei::afe(e1, e2, tol))) \
NUKLEI_THROW(nuklei::stringify(e1, 20, 0) << " != " << nuklei::stringify(e2, 20, 0) << " (TOL=" << tol << ")"); \
}
  
  /** @brief String (C string) file:line. */
# define NUKLEI_HERE() ((std::string(__FILE__) + ":" + nuklei::stringify(__LINE__)))
  
# ifdef NDEBUG
#   define NUKLEI_TRACE_BEGIN() {}
#   define NUKLEI_TRACE_END() {}
# else
#   define NUKLEI_TRACE_BEGIN() try {
#   define NUKLEI_TRACE_END() \
} catch (nuklei::Error &e) { \
e.addTrace(__PRETTY_FUNCTION__, NUKLEI_HERE()); \
throw; \
} \
catch (std::exception &e) { \
throw nuklei::Error(e, __PRETTY_FUNCTION__, NUKLEI_HERE()); \
}
# endif
  
  /*
   # define NUKLEI_TRACE_CONTEXT_END() \
   } catch (nuklei::TracedError &e) { \
   e.addTrace(__PRETTY_FUNCTION__, NUKLEI_HERE(), context()); \
   throw; \
   } \
   catch (std::exception &e) { \
   throw nuklei::TracedError(e, __PRETTY_FUNCTION__, NUKLEI_HERE(), context()); \
   }
   */
  
# define NUKLEI_LBLACK       "\033[0;30m"
# define NUKLEI_LRED         "\033[0;31m"
# define NUKLEI_LGREEN       "\033[0;32m"
# define NUKLEI_LBROWN       "\033[0;33m"
# define NUKLEI_LBLUE        "\033[0;34m"
# define NUKLEI_LPURPLE      "\033[0;35m"
# define NUKLEI_LCYAN        "\033[0;36m"
# define NUKLEI_LWHITE       "\033[0;37m"
# define NUKLEI_DBLACK       "\033[1;30m"
# define NUKLEI_DRED         "\033[1;31m"
# define NUKLEI_DGREEN       "\033[1;32m"
# define NUKLEI_DBROWN       "\033[1;33m"
# define NUKLEI_DBLUE        "\033[1;34m"
# define NUKLEI_DPURPLE      "\033[1;35m"
# define NUKLEI_DCYAN        "\033[1;36m"
# define NUKLEI_DWHITE       "\033[1;37m"
  
# define NUKLEI_NOCOLOR      "\033[0m"
  
  /** @brief Provides a const reference to an object.  */
  template<class T>
  inline T const& as_const(T const& t)
  { return t; }
  
  /** @brief Converts an object to a std::string using operator<<. */
  template<typename T>
  inline std::string stringify(const T& x, int precision = -1, int width = 0);
  
  /** @brief Nuklei-related errors. */
  class Error : public std::exception
  {
  public:
    Error() : count_(0)
    {
      init();
      compileWhatString();
    }
    explicit Error(const std::string& msg) : msg_(msg), count_(0)
    {
      init();
      compileWhatString();
    }
    explicit Error(const std::exception& error,
                   const std::string& function,
                   const std::string& file,
                   const std::string& msg = "") : count_(0)
    {
      init();
      type_ = typeid(error).name();
      msg_ = error.what();
      addTrace(function, file, msg);
      compileWhatString();
    }
    
    // The constructor calls this method, keep it non-virtual.
    void init()
    {
      if (INTERACTIVE_SHELL)
      {
        color_start_ = NUKLEI_LRED;
        color_stop_ = NUKLEI_NOCOLOR;
      }
    }
    
    // The constructor calls this method, keep it non-virtual.
    void addTrace(const std::string& function,
                  const std::string& file,
                  const std::string& msg = "")
    {
      trace_ += Log::breakLines("  " + color_start_ + stringify(count_++) +
                                ":\n    " + color_stop_ + function +
                                color_start_ + " @[" + color_stop_ + file +
                                color_start_ + "] [" +
                                color_stop_ + msg + color_start_ + "]" +
                                color_stop_);
      compileWhatString();
    }
    
    void compileWhatString()
    {
      if (type_.empty()) type_ = typeid(*this).name();
      what_ =
      color_start_ + "[Exception Trace]" + color_stop_ +
      Log::breakLines(color_start_ + "Type: " + color_stop_ + type_ +
                      color_start_ + "\nMessage:\n  " + color_stop_ + msg_ +
                      color_start_ + "\nTrace:" + color_stop_) + trace_;
    }
    
    // The following methods are virtual in std::exception.
    ~Error() throw() {}
    const char* what() const throw()
    {
      return what_.c_str();
    }
  private:
    std::string type_;
    std::string trace_;
    std::string msg_;
    std::string what_;
    std::string color_start_;
    std::string color_stop_;
    int count_;
  };
  
  class VerificationFailure : public Error
  {
  public:
    VerificationFailure(const std::string& s) : Error(s) { }
  };
  
  class BadStrNumConversion : public Error
  {
  public:
    BadStrNumConversion(const std::string& s) : Error(s) { }
  };
  
  inline std::istream& operator>>(std::istream &in, Vector3 &l);  
  inline std::istream& operator>>(std::istream &in, Quaternion &q);
  inline std::ostream& operator<<(std::ostream &out, const Quaternion &q);  
  inline std::ostream& operator<<(std::ostream &out, const Matrix3 &m);
  inline std::istream& operator>>(std::istream &in, Matrix3 &m);

  
  /** @brief Converts a range to a std::string using operator<< on elements. */
  template<typename ForwardIterator>
  inline std::string stringifyRange(ForwardIterator first, ForwardIterator last,
                                    int precision = -1, int width = 0)
  {
    std::ostringstream o;
    
    if (first != last)
    {
      if (precision != -1) o.precision(precision);
      char fill = o.fill();
      o.fill('0');
      if (width != 0) o.width(width);
      
      ForwardIterator i = first;
      
      if (!(o << *i))
        throw BadStrNumConversion
        (std::string("stringify(") + typeid(*i).name() + ")");
      i++;
      for ( ; i != last; i++)
        if (!(o << " " << *i))
          throw BadStrNumConversion
          (std::string("stringify(") + typeid(*i).name() + ")");
      
      o.fill(fill);
    }
    
    return o.str();
  }
  
  
  /** @brief Converts an object to a std::string using operator<<. */
  template<typename T>
  inline std::string stringify(const T& x, int precision, int width)
  {
    std::ostringstream o;
    if (precision != -1) o.precision(precision);
    char fill = o.fill();
    o.fill('0');
    if (width != 0) o.width(width);
    if (!(o << x))
      throw BadStrNumConversion(std::string("stringify(")
                                + typeid(x).name() + ")");
    o.fill(fill);
    return o.str();
  }
  
  /** @brief Converts a string to an object using operator>>. */
  template<typename T>
  inline T numify(const std::string& s,
                  bool failIfLeftoverChars = true)
  {
    T x;
    std::istringstream i(s);
    char c;
    if (!(i >> x) || (failIfLeftoverChars && i.get(c)))
      throw BadStrNumConversion(s);
    return x;
  }
  
  inline std::string& cleanLine(std::string& line)
  {
    if(!line.empty() && *line.rbegin() == '\r')
      line.erase( line.length()-1, 1);
    return line;
  }
  
}

#endif
