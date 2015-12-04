// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_LINEAR_ALGEBRA_DEF_H
#define NUKLEI_LINEAR_ALGEBRA_DEF_H

#include <nuklei/Wm5/Wm5Vector2.h>
#include <nuklei/Wm5/Wm5Vector3.h>
#include <nuklei/Wm5/Wm5GVector.h>
#include <nuklei/Wm5/Wm5Matrix3.h>
#include <nuklei/Wm5/Wm5GMatrix.h>
#include <nuklei/Wm5/Wm5Plane3.h>
#include <nuklei/Wm5/Wm5Quaternion.h>
#include <nuklei/Definitions.h>

namespace nuklei {

  typedef nuklei_wmf::Vector2<coord_t> Vector2;
  typedef nuklei_wmf::Vector3<coord_t> Vector3;
  typedef nuklei_wmf::GVector<coord_t> GVector;
  typedef nuklei_wmf::Quaternion<coord_t> Quaternion;
  typedef nuklei_wmf::Matrix3<coord_t> Matrix3;
  typedef nuklei_wmf::GMatrix<coord_t> GMatrix;
  typedef nuklei_wmf::Plane3<coord_t> Plane3;

}

#endif
