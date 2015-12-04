// (C) Copyright Renaud Detry   2007-2008.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

/** @file */

#ifndef NUKLEI_TRSL_SORT_ITERATOR_HPP
#define NUKLEI_TRSL_SORT_ITERATOR_HPP

#include <nuklei/trsl/reorder_iterator.hpp>
#include <nuklei/trsl/common.hpp>
#include <nuklei/trsl/error_handling.hpp>

#include <functional>

namespace nuklei_trsl
{

  namespace detail {
  
    template<
      class RandomIterator,
      class Comparator
      > class at_index_comp
      {
      public:
      
        at_index_comp(const RandomIterator &first, const Comparator &comp) :
          elements_(first), comp_(comp)
          {}
      
        bool operator() (unsigned i, unsigned j)
          {
            return comp_(*(elements_+i), *(elements_+j));
          }
      
        RandomIterator elements_;
        Comparator comp_;
      };
  
  }

  /**
   * @brief Constructs a reorder_iterator that will iterate through
   * the first @p permutationSize elements of a sorted permutation of
   * the population referenced by @p first and @p last.
   *
   * The @p permutationSize should be smaller or equal to the size of
   * the population. If it is not the case, a bad_parameter_value is
   * thrown.
   *
   * A comparator is provided through @p comp. <tt>Comparator</tt>
   * has to model <a
   * href="http://www.sgi.com/tech/stl/StrictWeakOrdering.html"
   * >Strict Weak Ordering</a>.  In particular <a
   * href="http://www.sgi.com/tech/stl/less.html"
   * ><tt>std::less<ElementType>()</tt></a> and <a
   * href="http://www.sgi.com/tech/stl/greater.html"
   * ><tt>std::greater<ElementType>()</tt></a> will work, whereas <a
   * href="http://www.sgi.com/tech/stl/less_equal.html"
   * ><tt>std::less_equal<ElementType>()</tt></a> and <a
   * href="http://www.sgi.com/tech/stl/greater_equal.html"
   * ><tt>std::greater_equal<ElementType>()</tt></a> will
   * <em>not</em>.
   *
   * @p ElementIterator should model <em>Random Access Iterator</em>.
   *
   * Creating such a reorder_iterator and iterating through it is
   * generally much faster than re-ordering the population itself (or
   * a copy thereof), especially when elements are large, have a
   * complex copy-constructor, or a tall class hierarchy.
   */
  template<class ElementIterator, class ElementComparator>
  reorder_iterator<ElementIterator>
  sort_iterator(ElementIterator first,
                ElementIterator last,
                ElementComparator comp,
                unsigned permutationSize)
  {
    ptrdiff_t size = std::distance(first, last);
    if (size < 0)
      throw bad_parameter_value(
        "sort_iterator: "
        "bad input range.");
    if (permutationSize > unsigned(size))
      throw bad_parameter_value(
        "sort_iterator: "
        "parameter permutationSize out of range.");
        
    typedef
      typename reorder_iterator<ElementIterator>::index_container
      index_container;
    typedef
      typename reorder_iterator<ElementIterator>::index_container_ptr
      index_container_ptr;
    typedef
      typename reorder_iterator<ElementIterator>::index_t
      index_t;
  
    index_container_ptr index_collection(new index_container);
        
    index_collection->resize(size);
    for (index_t i = 0; i < index_t(size); ++i)
      (*index_collection)[i] = i;
    
    if (permutationSize == unsigned(size))
      std::sort(index_collection->begin(),
                index_collection->end(),
                detail::at_index_comp
                <ElementIterator, ElementComparator>(first, comp));
    else
    {
      std::partial_sort(index_collection->begin(),
                        index_collection->begin()+permutationSize,
                        index_collection->end(),
                        detail::at_index_comp
                        <ElementIterator, ElementComparator>(first, comp));
      index_collection->resize(permutationSize);
    }
    
    return reorder_iterator<ElementIterator>(first, index_collection);
  }

  /**
   * @brief Constructs a reorder_iterator that will iterate through a
   * sorted permutation of the population referenced by @p first and
   * @p last.
   *
   * A comparator is provided through @p comp. <tt>Comparator</tt>
   * has to model <a
   * href="http://www.sgi.com/tech/stl/StrictWeakOrdering.html"
   * >Strict Weak Ordering</a>.  In particular <a
   * href="http://www.sgi.com/tech/stl/less.html"
   * ><tt>std::less<ElementType>()</tt></a> and <a
   * href="http://www.sgi.com/tech/stl/greater.html"
   * ><tt>std::greater<ElementType>()</tt></a> can work, whereas <a
   * href="http://www.sgi.com/tech/stl/less_equal.html"
   * ><tt>std::less_equal<ElementType>()</tt></a> and <a
   * href="http://www.sgi.com/tech/stl/greater_equal.html"
   * ><tt>std::greater_equal<ElementType>()</tt></a> will
   * <em>not</em>.
   *
   * @p ElementIterator should model <em>Random Access Iterator</em>.
   *
   * Creating such a reorder_iterator and iterating through it is
   * generally much faster than re-ordering the population itself (or
   * a copy thereof), especially when elements are large, have a
   * complex copy-constructor, or a tall class hierarchy.
   */
  template<class ElementIterator, class ElementComparator>
  reorder_iterator<ElementIterator>
  sort_iterator(ElementIterator first,
                ElementIterator last,
                ElementComparator comp)
  {
    return sort_iterator(first,
                         last,
                         comp,
                         std::distance(first, last));
  }

  /**
   * @brief Constructs a reorder_iterator that will iterate through a
   * sorted permutation of the population referenced by @p first and
   * @p last.
   *
   * The population is sorted using <tt>std::less<>()</tt>, i.e. in
   * ascending order.
   *
   * @p ElementIterator should model <em>Random Access Iterator</em>.
   *
   * Creating such a reorder_iterator and iterating through it is
   * generally much faster than re-ordering the population itself (or
   * a copy thereof), especially when elements are large, have a
   * complex copy-constructor, or a tall class hierarchy.
   */
  template<class ElementIterator>
  reorder_iterator<ElementIterator>
  sort_iterator(ElementIterator first,
                ElementIterator last)
  {
    return sort_iterator(first,
                         last,
                         std::less
                         <typename std::iterator_traits
                         <ElementIterator>::value_type>());
  }

} // namespace nuklei_trsl

#endif // include guard
