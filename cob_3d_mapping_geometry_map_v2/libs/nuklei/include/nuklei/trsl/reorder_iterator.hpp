// (C) Copyright Toon Knapen    2001.
// (C) Copyright David Abrahams 2003.
// (C) Copyright Roland Richter 2003.
// (C) Copyright Renaud Detry   2007-2008.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

// This file is a fork of boost/iterator/permutation_iterator.hpp

/** @file */

#ifndef NUKLEI_TRSL_REORDER_ITERATOR_HPP
#define NUKLEI_TRSL_REORDER_ITERATOR_HPP

#include <nuklei/trsl/error_handling.hpp>
#include <nuklei/trsl/common.hpp>

#include <iterator>
#include <vector>
#include <algorithm>
#include <boost/iterator.hpp>
#include <boost/iterator/iterator_categories.hpp>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/shared_ptr.hpp>

namespace nuklei_trsl
{
  template<class ElementIterator>
  class reorder_iterator;
  
  namespace detail
  {
    /** @brief Used internally. */
    template<class ElementIterator>
    struct reorder_iterator_base
    {
      typedef ElementIterator element_iterator;
      typedef size_t index_t;
      typedef std::vector<index_t> index_container;
      typedef boost::shared_ptr<index_container> index_container_ptr;
      typedef typename index_container::const_iterator index_iterator;
    
      typedef boost::iterator_adaptor< 
        reorder_iterator<ElementIterator>,
        index_iterator,
        typename boost::detail::iterator_traits<ElementIterator>::value_type,
        boost::use_default,
        typename boost::detail::iterator_traits<ElementIterator>::reference
      > type;
    };
  }
  
  
  /**
   * @brief Provides an iterator over a permutation of a range.
   *
   * This class is a fork of <a
   * href="http://www.boost.org/libs/iterator/doc/permutation_iterator.html"
   * >boost::permutation_iterator</a>.  With <a
   * href="http://www.boost.org/libs/iterator/doc/permutation_iterator.html"
   * >boost::permutation_iterator</a>, the user provides a population,
   * and a range of index that defines a permutation over the
   * population. It allows for much flexibility, but leaves the user
   * responsible for storing an array of index. This
   * class allows to store the array internally, in the same way as <a
   * href="http://www.boost.org/libs/utility/shared_container_iterator.html"
   * >boost::shared_container_iterator</a>.
   *
   * The index array is stored within the iterator, by means of a <a
   * href="http://www.boost.org/libs/smart_ptr/shared_ptr.htm"
   * >boost::shared_ptr</a>; thus, all copies of a reorder iterator
   * share the same index array. One drawback is that reorder_iterator
   * copy is somewhat slower than ElementIterator copy. Incrementation
   * is still plainly efficient, nevertheless.
   *
   * When iterating over a permutation of a population range using an
   * index range, the iteration is actually performed over the index
   * range; the population range is only used when
   * dereferencing. Thus, every trsl::reorder_iterator knows where it
   * begins and where it ends, hence provided begin() and end()
   * methods.
   *
   * TRSL provides several functions that generate reoder iterators
   * for common reorderings. See random_permutation_iterator() and
   * sort_iterator().
   *
   * @p ElementIterator should model <em>Random Access Iterator</em>.
   * See the doc on <a
   * href="http://www.boost.org/libs/iterator/doc/permutation_iterator.html"
   * >boost::permutation_iterator</a> for further details.
   */
  template<class ElementIterator>
  class reorder_iterator
    : public detail::reorder_iterator_base<ElementIterator>::type
  {
    typedef detail::reorder_iterator_base<ElementIterator> base_t;
    typedef typename base_t::type super_t;

    friend class boost::iterator_core_access;
    
  public:

    typedef typename base_t::index_t index_t;
    typedef typename base_t::index_container index_container;
    typedef typename base_t::index_container_ptr index_container_ptr;
    typedef typename base_t::index_iterator index_iterator;

    
    typedef typename base_t::element_iterator element_iterator;

    reorder_iterator() :
      m_elt_iter(),
      m_index_collection(new index_container)
      {}
      
    /**
     * @brief Constructs an iterator that will walk through the elements
     * of the range that begins at @p first, follwing the order defined
     * by @p index_collection.
     */
    explicit reorder_iterator(ElementIterator first,
                              const index_container_ptr& index_collection)
      : super_t(index_collection->begin()),
        m_elt_iter(first),
        m_index_collection(index_collection)
      {}
    
    /**
     * @brief Allows conversion from a reorder_iterator to
     * a const reorder_iterator, won't allow conversion
     * from a const reorder_iterator to a
     * reorder_iterator.
     *
     * By &ldquo;const reorder_iterator&rdquo;, we mean that the @p
     * ElementIterator is const, e.g.
     * <tt>std::vector<Particle>::const_iterator</tt>.
     */
    template<class OtherElementIterator>
    reorder_iterator
    (reorder_iterator<OtherElementIterator> const& r,
     typename boost::enable_if_convertible<OtherElementIterator, ElementIterator>::type* = 0) :
      super_t(r.base()), m_elt_iter(r.m_elt_iter),
      m_index_collection(r.m_index_collection)
      {}

    /**
     * @brief Returns a reorder_iterator pointing to
     * the begining of the permutation.
     */
    reorder_iterator<ElementIterator> begin() const
      {
        reorder_iterator<ElementIterator> indexIterator(*this);
        indexIterator.base_reference() =
          indexIterator.m_index_collection->begin();
        return indexIterator;
      }
    
    /**
     * @brief Returns a reorder_iterator pointing to
     * the end of the permutation.
     */
    reorder_iterator<ElementIterator> end() const
      {
        reorder_iterator<ElementIterator> indexIterator(*this);
        indexIterator.base_reference() =
          indexIterator.m_index_collection->end();
        return indexIterator;
      }
    
    /**
     * @brief Returns the index of the element that the iterator is
     * currently pointing to.
     */
    index_t index() const
      {
        return *this->base();
      }
      
  private:
      
    typename super_t::reference dereference() const
      { return *(m_elt_iter + *this->base()); }
    
#ifndef BOOST_NO_MEMBER_TEMPLATE_FRIENDS
    template <class> friend class reorder_iterator;
#else
  public:
#endif 
    ElementIterator m_elt_iter;
  protected:
    index_container_ptr m_index_collection;
  };
  
} // namespace nuklei_trsl

#endif // include guard
