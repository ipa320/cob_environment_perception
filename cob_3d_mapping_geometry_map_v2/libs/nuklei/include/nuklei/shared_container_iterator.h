// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

// (C) Copyright Toon Knapen    2001.
// (C) Copyright David Abrahams 2003.
// (C) Copyright Roland Richter 2003.
// (C) Copyright Renaud Detry   2007-2008.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

/** @file */

#ifndef NUKLEI_SHARED_CONTAINER_ITERATOR_HPP
#define NUKLEI_SHARED_CONTAINER_ITERATOR_HPP

#include <iterator>
#include <vector>
#include <algorithm>
#include <boost/iterator.hpp>
#include <boost/iterator/iterator_categories.hpp>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/shared_ptr.hpp>

namespace nuklei
{
  template<class Container, class ContainerIterator>
  class shared_container_iterator;
  
  namespace detail
  {
    /** @brief Used internally. */
    template<class Container, class ContainerIterator>
    struct shared_container_iterator_base
    {
      typedef Container container_t;
      typedef ContainerIterator container_iterator;
      typedef boost::shared_ptr<Container> container_ptr;
    
      typedef boost::iterator_adaptor<
        shared_container_iterator<Container, ContainerIterator>,
        ContainerIterator,
        typename boost::detail::iterator_traits<ContainerIterator>::value_type,
        boost::use_default,
        typename boost::detail::iterator_traits<ContainerIterator>::reference
      > type;
    };
  }
  
  
  template<class Container, class ContainerIterator>
  class shared_container_iterator
    : public detail::shared_container_iterator_base<Container, ContainerIterator>::type
  {
    typedef detail::shared_container_iterator_base<Container, ContainerIterator> base_t;
    typedef typename base_t::type super_t;

    friend class boost::iterator_core_access;
    
  public:

    typedef typename base_t::container_t container_t;
    typedef typename base_t::container_ptr container_ptr;
    typedef typename base_t::container_iterator container_iterator;
    
    shared_container_iterator() :
      m_collection(new container_t)
      {}
      
    shared_container_iterator(container_ptr& index_collection)
      : super_t(index_collection->begin()),
        m_collection(index_collection)
      {}
    
    /**
     * @brief Allows conversion from a shared_container_iterator to
     * a const shared_container_iterator, won't allow conversion
     * from a const shared_container_iterator to a
     * shared_container_iterator.
     *
     * By &ldquo;const shared_container_iterator&rdquo;, we mean that the @p
     * ContainerIterator is const, e.g.
     * <tt>std::vector<Particle>::const_iterator</tt>.
     */
    template<class OtherContainerIterator>
    shared_container_iterator
    (shared_container_iterator<Container, OtherContainerIterator> const& r,
     typename boost::enable_if_convertible<OtherContainerIterator, ContainerIterator>::type* = 0) :
      super_t(r.base()),
      m_collection(r.m_collection)
      {}

    /**
     * @brief Returns a shared_container_iterator pointing to
     * the begining of the range.
     */
    shared_container_iterator<Container, ContainerIterator> begin() const
      {
        shared_container_iterator<Container, ContainerIterator> iterator(*this);
        iterator.base_reference() =
          iterator.m_collection->begin();
        return iterator;
      }
    
    /**
     * @brief Returns a shared_container_iterator pointing to
     * the end of the range.
     */
    shared_container_iterator<Container, ContainerIterator> end() const
      {
        shared_container_iterator<Container, ContainerIterator> iterator(*this);
        iterator.base_reference() =
          iterator.m_collection->end();
        return iterator;
      }
      
  private:
      
#ifndef BOOST_NO_MEMBER_TEMPLATE_FRIENDS
    template <class, class> friend class shared_container_iterator;
#else
  public:
#endif 
  protected:
    container_ptr m_collection;
  };
  
} // namespace nuklei

#endif // include guard
