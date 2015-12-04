// (C) Copyright David Abrahams 2002.
// (C) Copyright Jeremy Siek    2002.
// (C) Copyright Thomas Witt    2002.
// (C) Copyright Renaud Detry   2007-2008.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

// This file is a near copy of boost/iterator/filter_iterator.hpp.
// Conceptual modifications to boost/iterator/filter_iterator.hpp
// are reported /* - */ and /* + */
// Doxygen comments are not part of filter_iterator.hpp.
// Name changes are not reported.

/** @file */

#ifndef NUKLEI_TRSL_PERSISTENT_FILTER_ITERATOR_23022003THW_HPP
#define NUKLEI_TRSL_PERSISTENT_FILTER_ITERATOR_23022003THW_HPP

#include <boost/iterator.hpp>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/iterator/iterator_categories.hpp>

#include <boost/type_traits/is_class.hpp>
#include <boost/static_assert.hpp>

namespace nuklei_trsl
{
  template <class Predicate, class Iterator>
  class persistent_filter_iterator;

  namespace detail
  {
    /** @brief Used internally. */
    template <class Predicate, class Iterator>
    struct persistent_filter_iterator_base
    {
      typedef boost::iterator_adaptor<
        persistent_filter_iterator<Predicate, Iterator>
        , Iterator
        , boost::use_default
        , typename boost::mpl::if_<
        boost::is_convertible<
        typename boost::iterator_traversal<Iterator>::type
        , boost::random_access_traversal_tag
        >
/* -    , bidirectional_traversal_tag*/
        , boost::forward_traversal_tag/* + */
/* -    , use_default*/
        , boost::forward_traversal_tag/* + */
        >::type
      > type;
    };
  }
  
  /**
   * @brief Adaptation of <a
   * href="http://www.boost.org/libs/iterator/doc/filter_iterator.html"
   * >boost::filter_iterator</a> to allow an element to be selected
   * multiple times.
   *
   * The class <a
   * href="http://www.boost.org/libs/iterator/doc/filter_iterator.html"
   * >boost::filter_iterator</a> is an example of <a
   * href="http://www.boost.org/libs/iterator/doc/iterator_adaptor.html"
   * >boost::iterator_adaptor</a> that allows iterating through a
   * range, skipping elements that don't verify a predicate. In <a
   * href="http://www.boost.org/libs/iterator/doc/filter_iterator.html"
   * >boost::filter_iterator</a>, incrementing the iterator will
   * <em>always</em> advance of at least one element. In
   * persistent_filter_iterator, an incrementation will not advance to
   * a next element until the predicate becomes false on the current
   * element. This implies that the predicate has either some memory,
   * or a pseudo-random behavior.
   *
   * A persistent_filter_iterator thus iterates over a
   * <em>virtual</em> range.  Consequently, equality of two
   * persistent_filter_iterator is only verified if they point to the
   * same element in the original range <em>and</em> if their
   * predicates are equal. Predicates must thus implement
   * <tt>operator==</tt>.  To see whether two different
   * persistent_filter_iterator actually point to the same input range
   * element, one can compare the underlying iterators available
   * through the <tt>base()</tt> method.
   * 
   *
   * The doc on <a
   * href="http://www.boost.org/libs/iterator/doc/filter_iterator.html"
   * >boost::filter_iterator</a> applies for this class, except
   * for the small differences noted above.
   */
  template <class Predicate, class Iterator>
  class persistent_filter_iterator
    : public detail::persistent_filter_iterator_base<Predicate, Iterator>::type
  {
    typedef typename detail::persistent_filter_iterator_base<
      Predicate, Iterator
      >::type super_t;

    friend class boost::iterator_core_access;

  public:
    persistent_filter_iterator() { }

    persistent_filter_iterator(Predicate f, Iterator x, Iterator end_ = Iterator())
      : super_t(x), m_predicate(f), m_end(end_)
      {
        satisfy_predicate();
      }

    persistent_filter_iterator(Iterator x, Iterator end_ = Iterator())
      : super_t(x), m_predicate(), m_end(end_)
      {
        // Pro8 is a little too aggressive about instantiating the
        // body of this function.
#if !BOOST_WORKAROUND(__MWERKS__, BOOST_TESTED_AT(0x3003))
        // Don't allow use of this constructor if Predicate is a
        // function pointer type, since it will be 0.
        BOOST_STATIC_ASSERT(boost::is_class<Predicate>::value);
#endif 
        satisfy_predicate();
      }

    template<class OtherIterator>
    persistent_filter_iterator(
      persistent_filter_iterator<Predicate, OtherIterator> const& t
      , typename boost::enable_if_convertible<OtherIterator, Iterator>::type* = 0
      )
      : super_t(t.base()), m_predicate(t.predicate()), m_end(t.end()) {}

    Predicate predicate() const { return m_predicate; }

    Iterator end() const { return m_end; }

  private:
    void increment()
      {
/* -      ++(this->base_reference());*/
        satisfy_predicate();
      }

/* -  void decrement()*/
/* -  {*/
/* -    while(!this->m_predicate(*--(this->base_reference()))){};*/
/* -  }*/

    void satisfy_predicate()
      {
        while (this->base() != this->m_end && !this->m_predicate(*this->base()))
          ++(this->base_reference());
      }

    template<class OtherIterator>/* + */
    bool equal(/* + */
      persistent_filter_iterator<Predicate, OtherIterator> const& t/* + */
      , typename boost::enable_if_convertible<OtherIterator, Iterator>::type* = 0/* + */
      ) const/* + */
      {/* + */
        return (this->base() == t.base()) &&/* + */
          // There is always at most one end iterator/* + */
          ((this->base() == m_end) ||/* + */
           (this->predicate() == t.predicate()));/* + */
      }/* + */

    // Probably should be the initial base class so it can be
    // optimized away via EBO if it is an empty class.
    Predicate m_predicate;
    Iterator m_end;
  };

  template <class Predicate, class Iterator>
  persistent_filter_iterator<Predicate,Iterator>
  make_persistent_filter_iterator(Predicate f, Iterator x, Iterator end = Iterator())
  {
    return persistent_filter_iterator<Predicate,Iterator>(f,x,end);
  }

  template <class Predicate, class Iterator>
  persistent_filter_iterator<Predicate,Iterator>
  make_persistent_filter_iterator(
    typename boost::iterators::enable_if<
    boost::is_class<Predicate>
    , Iterator
    >::type x
    , Iterator end = Iterator()
#if BOOST_WORKAROUND(BOOST_MSVC, < 1300)
    , Predicate* = 0
#endif 
    )
  {
    return persistent_filter_iterator<Predicate,Iterator>(x,end);
  }

} // namespace nuklei_trsl

#endif // NUKLEI_TRSL_PERSISTENT_FILTER_ITERATOR_23022003THW_HPP
