// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */


#ifndef NUKLEI_DEFINITIONS_H
#define NUKLEI_DEFINITIONS_H

#include <limits>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdexcept>
#include <cstdio>
#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/version.hpp>

namespace nuklei {

  /** @brief Type for point coordinates */
  typedef double coord_t;
  /** @brief Pair of coord_t */
  typedef std::pair<coord_t, coord_t> coord_pair;
  /** @brief Type for appearance-related values (e.g., color) */
  typedef double appear_t;
  /** @brief Type for particle weights */
  typedef double weight_t;
  /** @brief Type for bitfield */
  typedef unsigned int bitfield_t;
  /** @brief Type for identifier label */
  typedef unsigned int id_t;

  extern const coord_t FLOATTOL;

  extern const std::string INFOSTRING;

  // Default precision in text files.
  extern const int PRECISION;

  extern const bool KDTREE_DENSITY_EVAL;
  extern const bool KDTREE_NANOFLANN;

  extern const unsigned int KDE_KTH_NEAREST_NEIGHBOR;

  extern const appear_t HSV_METRIC_VALUE_WEIGHT;

  // Be nice to other processes.
  extern const int NICEINC;

  // For object IO.
  extern const std::string SERIALIZATION_DEFAULT_BOOST_ARCHIVE;

  extern const unsigned IMAGE_PROJECTION_RADIUS;

  extern const std::string PARALLELIZATION;

  extern const bool ENABLE_CONSOLE_BACKSPACE;
  
  extern const unsigned LOG_LEVEL;

  extern bool LAST_OUTPUT_LINE_IS_PROGRESS;
  
  extern const bool INTERACTIVE_SHELL;
  
  bool hasOpenMP();
}

#endif


