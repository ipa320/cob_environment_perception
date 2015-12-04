// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_TYPES_H
#define NUKLEI_TYPES_H

#include <vector>

#include <nuklei/Common.h>

namespace nuklei {
  
  /**
   * @addtogroup type
   * @{
   */
  
  /**
   * @brief Cats all type names into a string.
   *
   * Names are separated by a bar.
   * The default name is followed by a star.
   */
  template<class T>
  std::string catTypeNames()
  {
    std::string s;
    for (int i = 0; i < T::UNKNOWN; i++)
    {
      s += T::TypeNames[i];
      if (i == T::defaultType) s += "*";
      if (i != T::UNKNOWN-1) s += std::string("|");
    }
    return s;
  }
  
  /** @brief Fills a std::vector with all type names. */
  template<class T>
  std::vector<std::string> listTypeNames()
  {
    std::vector<std::string> l;
    for (int i = 0; i < T::UNKNOWN; i++)
      l.push_back(T::TypeNames[i]);
    return l;
  }
  
  /** @brief Returns the default type name. */
  template<class T>
  std::string defaultTypeName()
  {
    return T::TypeNames[T::defaultType];
  }
  
  /** @brief Returns the default type. */
  template<class T>
  typename T::Type defaultType()
  {
    return T::defaultType;
  }
  
  /** @brief Returns the name of type @p t. */
  template<class T>
  std::string nameFromType(int t)
  {
    NUKLEI_ASSERT(0 <= t && t <= T::UNKNOWN);
    if (t == T::UNKNOWN) return "unknown";
    else return T::TypeNames[t];
  }
  
  /** @brief Returns the type whose name is @p s. */
  template<class T>
  typename T::Type typeFromName(std::string s)
  {
    for (int i = 0; i < T::UNKNOWN; i++)
    {
      if (s == T::TypeNames[i]) return typename T::Type(i);
    }
    if (s == "unknown") return T::UNKNOWN;
    NUKLEI_THROW("Invalid type `" << s << "'.");
  }
  
  /** @} */
}
#endif
