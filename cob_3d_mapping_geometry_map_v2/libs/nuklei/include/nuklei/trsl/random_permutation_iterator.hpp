// (C) Copyright Renaud Detry   2007-2008.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

/** @file */

#ifndef NUKLEI_TRSL_RANDOM_PERMUTATION_ITERATOR_HPP
#define NUKLEI_TRSL_RANDOM_PERMUTATION_ITERATOR_HPP

#include <nuklei/trsl/reorder_iterator.hpp>
#include <nuklei/trsl/common.hpp>
#include <nuklei/trsl/error_handling.hpp>

namespace nuklei_trsl
{

  /**
   * @brief Constructs a reorder_iterator that will iterate through a
   * random subset of size @p permutationSize of a random permutation
   * of the population referenced by @p first and @p last.
   *
   * The @p permutationSize should be smaller or equal to the
   * size of the population. If it is not the case, a bad_parameter_value
   * is thrown.
   *
   * Performing a random permutation requires a series of random
   * integers, these are provided by rand_gen::uniform_int; see @ref
   * random for further details.
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
  random_permutation_iterator(ElementIterator first,
                              ElementIterator last,
                              unsigned permutationSize)
  {
    ptrdiff_t size = std::distance(first, last);
    if (size < 0)
      throw bad_parameter_value(
        "random_permutation_iterator: "
        "bad input range.");
    if (permutationSize > unsigned(size))
      throw bad_parameter_value(
        "random_permutation_iterator: "
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
      std::random_shuffle(index_collection->begin(),
                          index_collection->end(),
                          rand_gen::uniform_int);
    else
    {
      detail::partial_random_shuffle(index_collection->begin(),
                                     index_collection->begin()+permutationSize,
                                     index_collection->end(),
                                     rand_gen::uniform_int);
      index_collection->resize(permutationSize);
    }
    
    return reorder_iterator<ElementIterator>(first, index_collection);
  }

  /**
   * @brief Constructs a reorder_iterator that will iterate through a
   * random permutation of the population referenced by @p first and
   * @p last.
   *
   * Performing a random permutation requires a series of random
   * integers, these are provided by rand_gen::uniform_int; see @ref
   * random for further details.
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
  random_permutation_iterator(ElementIterator first,
                              ElementIterator last)
  {
    return random_permutation_iterator(first,
                                       last,
                                       std::distance(first, last));
  }

} // namespace nuklei_trsl

#endif // include guard
