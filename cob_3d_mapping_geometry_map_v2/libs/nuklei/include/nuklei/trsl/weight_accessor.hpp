// (C) Copyright Renaud Detry   2007-2008.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

/** @file */

#ifndef NUKLEI_TRSL_WEIGHT_ACCESSOR_HPP
#define NUKLEI_TRSL_WEIGHT_ACCESSOR_HPP

namespace nuklei_trsl {
  
  /**
   * @brief Weight accessor that always returns 1.
   *
   * This accessor can be passed to sample from a population of
   * equal-weight elements. The total weight of the population should
   * naturally be set to the number of elements in the population.
   * This accessor is of type <em>functor</em>, see @ref accessor for
   * more details.
   */
  template<typename WeightType, typename ElementType>
  struct weight_accessor
  {
    /**
     * @brief Functor implementation.
     *
     * @return 1.
     */ 
    WeightType operator()(ElementType const& e) const
      {
        return 1;
        // You can create your own accessor by copying this class
        // and returning something like
        //   return e.getWeight();
      }
  };

  /**
   * @brief Method Pointer weight accessor.
   *
   * Weight accessor for element classes that provide access to
   * their weight through a method signed <tt>WeightType
   * (ElementType::*)() const</tt>.
   *
   * This class is very similar to
   * <tt>std::const_mem_fun_ref_t</tt>. The only two differences are
   * <ul><li>mp_weight_accessor provides a default constructor that
   * initializes the method pointer to NULL;</li>
   * <li>mp_weight_accessor checks if the method pointer is NULL
   * before dereferencing it.</li></ul> These differences allow
   * mp_weight_accessor to be used as a default type for
   * is_picked_systematic weight accessor, since a default-value
   * initialization won't imply segfault. However, access is a bit
   * slower because of the extra check for a non-NULL pointer.  If the
   * pointer is NULL, <tt>operator()</tt> returns 1.
   *
   * See @ref accessor for more details.
   */
  template<typename WeightType, typename ElementType>
  class mp_weight_accessor
  {
  public:
    /** @brief Pointer to a const method of ElementType that returns double */
    typedef WeightType (ElementType::*WeightAccessorMethodPointer)() const;
    
    /**
     * @brief Constructor from a WeightAccessorMethodPointer.
     *
     * @param wptr Pointer to the method of ElementType that
     * returns the element weight. If no pointer is passed, operator()
     * will return 1 all the time.
     *
     */
    mp_weight_accessor(WeightAccessorMethodPointer wptr = NULL) :
      wptr_(wptr) {}
    
    /**
     * @brief Functor implementation.
     *
     * @return 1 if wptr_ is NULL, <tt>e.*wptr_()</tt> else.
     */ 
    WeightType operator()(ElementType const& e) const
      {
        if (wptr_ == NULL)
          return 1;
        return (e.*wptr_)();
      }
  private:
    WeightAccessorMethodPointer wptr_;
  };

}

#endif // include guard
