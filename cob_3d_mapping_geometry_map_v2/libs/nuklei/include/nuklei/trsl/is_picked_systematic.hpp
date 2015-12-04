// (C) Copyright Renaud Detry   2007-2008.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

/** @file */

#ifndef NUKLEI_TRSL_IS_PICKED_SYSTEMATIC_HPP
#define NUKLEI_TRSL_IS_PICKED_SYSTEMATIC_HPP

#include <nuklei/trsl/common.hpp>
#include <nuklei/trsl/weight_accessor.hpp>

#include <algorithm>
#include <functional>
#include <utility>
#include <cstdlib> // rand & random
#include <limits>
#include <cassert>
#include <boost/static_assert.hpp>

/** @brief Public namespace. */
namespace nuklei_trsl {
    
  /**
   * @brief Functor to use with persistent_filter_iterator for
   * systematic sampling of a range.
   *
   * The sampling method is systematic sampling [1, 2].
   *
   * This class is intended to be used as a predicate functor to
   * trsl::persistent_filter_iterator to form a <em>sample
   * iterator</em>. The sample iterator accesses a population of
   * elements through a range defined by a pair of Forward Iterators
   * (begin/end), and provides on-the-fly iteration through a sample
   * of the population.
   *
   * Systematic sampling may perform very badly if the order in which
   * the input population is presented is following a pattern. If a
   * pattern is likely to occur in the population, the user may want to
   * pipe the sample iterator <em>after</em> a
   * trsl::random_permutation_iterator, or use is_picked_systematic with
   * ppfilter_iterator.  The resulting iterator effectively implements
   * <em>probability sampling</em>.  The price to pay for probability
   * sampling is a <em>Random Access Iterator</em>. While
   * persistent_filter_iterator will work e.g. with <tt>std::list</tt>,
   * ppfilter_iterator requires e.g. <tt>std::vector</tt> or
   * <tt>std::deque</tt>.
   *
   * @param ElementType Type of the elements in the 
   * population. Constness and
   * reference modifiers are handled internally; this parameter should be
   * a bare type, e.g. <tt>Particle</tt> (<em>not</em> <tt>const Particle&</tt>).
   *
   * @param WeightType Element weight type, should be a floating point type.
   * Defaults to <tt>double</tt>.
   *
   * @param WeightAccessor Type of the accessor that will allow to
   * extract weights from elements. Defaults to mp_weight_accessor,
   * see @ref accessor for further details on accessors.
   *
   * <b>References:</b>
   *
   * - [1] R. Douc, O. Cappe, and E. Moulines. Comparison of
   * resampling schemes for particle filtering. International
   * Symposium on Parallel and Distributed Processing and
   * Applications, 2005:64, 2005.
   *
   * - [2] J. Hol, T. Sch&ouml;n, and F. Gustafsson. On resampling
   * algorithms for particle filters. In Nonlinear Statistical Signal
   * Processing Workshop, 2006.
   */
  template<
    typename ElementType,
    typename WeightType = double,
    typename WeightAccessor = mp_weight_accessor<WeightType, ElementType>
  > class is_picked_systematic
  {
  private:
    BOOST_STATIC_ASSERT((std::numeric_limits<WeightType>::is_integer == false));
  public:
    typedef ElementType element_type;
    typedef WeightType weight_type;
    typedef WeightAccessor weight_accessor_type;
    
    /**
     * @brief Default constructor, shoud not be used explicitely.
     *
     * This constructor makes an invalid predicate. It should only be used in
     * cases where the predicate is never used.
     */
    is_picked_systematic() :
      sampleSize_(0),
      populationWeight_(0)
      {
        initialize( 0 );
      }
    
    /**
     * @brief Construction with system-provided random number.
     *
     * The systematic sampling predicate initialization needs a random
     * number in <tt>[0,1[</tt>.  This constructor uses
     * trsl::rand_gen::uniform_01 to generate that number.  See @ref
     * random for more details.
     *
     * @param sampleSize Number of elements in the sample, within
     * <tt>[0, infinity[</tt>.
     *
     * @param populationWeight Total weight of the
     * population, within <tt>]0, infinity[</tt>. Generally equal to 1.
     *
     * @param wac Weight accessor. Defaults to
     * mp_weight_accessor. Note that if you don't pass explicitly a
     * mp_weight_accessor(<tt>&ElementType::GETWEIGHTMETHOD</tt>), the
     * default constructor for mp_weight_accessor will setup to always
     * return 1, ignoring element weights. See @ref accessor for
     * further details.
     *
     * The population weight has to be strictly larger than 0. Also,
     * @p WeightType should be fine enough to allow the sum of all
     * element weights to be close to @p populationWeight.
     */
    is_picked_systematic(size_t sampleSize,
                         WeightType populationWeight,
                         WeightAccessor const& wac = WeightAccessor()) :
      wac_(wac), sampleSize_(sampleSize),
      populationWeight_(populationWeight)
      {
        initialize( rand_gen::uniform_01<WeightType>() );
      }

    /**
     * @brief Construction with user-provided random number.
     *
     * The systematic sampling predicate initialization needs a random
     * number in <tt>[0,1[</tt>.  This constructor allows the user to
     * provide that number directly; the user is free to choose a
     * fancy random number generator, such as the <a
     * href="http://www.boost.org/libs/random/index.html" >Boost
     * Random Number Library</a> or <a
     * href="http://www.gnu.org/software/gsl/manual/html_node/Random-Number-Generation.html"
     * >GSL</a>.
     *
     * @param sampleSize Number of elements in the sample, within
     * <tt>[0, infinity[</tt>.
     *
     * @param populationWeight Total weight of the
     * population, within <tt>]0, infinity[</tt>. Generally equal to 1.
     *
     * @param uniform01 Random number in <tt>[0,1[</tt>.
     *
     * @param wac Weight accessor. Defaults to
     * mp_weight_accessor. Note that if you don't pass explicitly a
     * mp_weight_accessor(<tt>&ElementType::GETWEIGHTMETHOD</tt>), the
     * default constructor for mp_weight_accessor will setup to always
     * return 1, ignoring element weights. See @ref accessor for
     * further details.
     *
     * The population weight has to be strictly larger than 0. Also,
     * @p WeightType should be fine enough to allow the sum of all
     * element weights to be close to @p populationWeight.
     */
    is_picked_systematic(size_t sampleSize,
                         WeightType populationWeight,
                         WeightType uniform01,
                         WeightAccessor const& wac = WeightAccessor()) :
      wac_(wac), sampleSize_(sampleSize),
      populationWeight_(populationWeight)
      {
        initialize(uniform01);
      }

    /**
     * @brief Decides whether <tt>e</tt> should be picked or not (used
     * by persistent_filter_iterator).
     *
     * Part of the requirements for persistent_filter_iterator
     * predicates.
     */
    bool operator()(const ElementType & e)
      {
        if (sampleSize_ == 0) return false;
        
#ifdef NUKLEI_TRSL_USE_SYSTEMATIC_INTUITIVE_ALGORITHM
        // This algorithm is the intuitive implementation of
        // systematic sampling, where one pictures a wheel with
        // N_SAMPLE spokes and distributes the population around the
        // wheel as segments of tire of length proportional to their
        // weight; the spokes point to picked elements.
        WeightType arrow = k_*step_;
        assert(cumulative_ <= arrow);
        if (arrow < cumulative_ + wac_(e))
        {
          k_++;
          return true;
        }
        cumulative_ += wac_(e);
        return false;
#else
        // This algorithm is a massaged version of the intuitive
        // algorithm.  Both algorithms are conceptually identical, but
        // this version is faster.
        assert(position_ >= 0);
        if (position_ < wac_(e))
        {
          position_ += step_;
          return true;
        }
        position_ -= wac_(e);
        return false;
#endif
      }

    /**
     * @brief Return whether @p e has been picked already.
     *
     * This method will return a coherent value only if the last
     * call of operator()(const ElementType&) was on @p e and returned true.
     * When is_picked_systematic is used with a presistent_filter_iterator @p i,
     * this assumption is always valid for @p *i as long as @p i is not the end.
     *
     * This method is awkward to use directly; it is meant to be called by
     * trsl::is_first_pick.
     */
    bool is_first_pick(const ElementType & e) const
    {
      if (wac_(e) <= step_) return true;
#ifdef NUKLEI_TRSL_USE_SYSTEMATIC_INTUITIVE_ALGORITHM
      return k_*step_ - cumulative_ < 2*step_;
#else
      return position_ < 2*step_;
#endif      
    }

    /**
     * @brief Returns whether two predicates are at the same sampling
     * advancement.
     *
     * Part of the requirements for persistent_filter_iterator
     * predicates.
     */
    bool operator== (const is_picked_systematic<ElementType, WeightType, WeightAccessor> &p) const
      {
        if (sampleSize_ != p.sampleSize_ ||
            populationWeight_ != p.populationWeight_) return false;

#ifdef NUKLEI_TRSL_USE_SYSTEMATIC_INTUITIVE_ALGORITHM
        if (cumulative_ != p.cumulative_ ||
            k_ != p.k_) return false;
#else
        if (position_ != p.position_) return false;
#endif
        
        return true;
      }
      
  private:
    void initialize(WeightType randomReal)
      {
        // If one could rely on
        //
        //   (x!=0.0)/0.0 == inf (with the sign of x)
        //
        // and on
        //
        //   0.0 * inf == NaN,
        //
        // neither of these throwing exceptions, then the following
        // block could be avoided, and the case sampleSize_ == 0 would
        // be implicitly handled by inf/NaN arithmetic.
        //
        // However, it would be necessary to maintain two different
        // implementations depending on
        // std::numeric_limits<double>::is_iec559, and even then I'm
        // not sure if inf/NaN arithmetic is reliable.
        if (sampleSize_ != 0)
          step_ = populationWeight_ / sampleSize_;
        else
          // A semantically correct value for step_ would be inf, but
          // we don't want to get into has_infinity trouble...
          // When sampleSize_==0, step_ should be considered undefined.
          step_ = 0;
        
        WeightType randomWeight = randomReal * step_;
#ifdef NUKLEI_TRSL_USE_SYSTEMATIC_INTUITIVE_ALGORITHM
        cumulative_ = -randomWeight;
        k_ = 0;
#else
        position_ = randomWeight;
#endif
      }

  private:
    WeightAccessor wac_;
    size_t sampleSize_;
    WeightType populationWeight_;
    WeightType step_;
  
#ifdef NUKLEI_TRSL_USE_SYSTEMATIC_INTUITIVE_ALGORITHM
    WeightType cumulative_;
    size_t k_;
#else
    WeightType position_;
#endif
  };

  /**
   * @brief Return whether @p *i has been picked already.
   *
   * This function calls <tt>i.predicate().is_first_pick(*i)</tt>.
   *
   * When sampling from a population, elements with a large weight may be selected several times.
   * This function checks if the element pointed by @p *i has been selected already, e.g. at <tt>*(i-1)</tt>.
   *
   * @param Iterator Will typically be trsl::pp_filter_iterator or trsl::persitent_filter_iterator.
   *
   * @param i Instance of pp_filter_iterator or persitent_filter_iterator. Should not be the end.
   */
  template<class Iterator>
  bool is_first_pick(const Iterator& i)
  {
    return i.predicate().is_first_pick(*i);
  }
  
}

#endif // include guard
