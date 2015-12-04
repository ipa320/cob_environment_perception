// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_COVIS3DOBSERVATION_H
#define NUKLEI_COVIS3DOBSERVATION_H


#include <vector>
#include <string>
#include <utility>
#include <stdexcept>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <nuklei/Definitions.h>
#include <nuklei/Color.h>
#include <nuklei/LinearAlgebra.h>
#include <nuklei/Observation.h>
#include <nuklei/nullable.h>

namespace nuklei {

  class CoViS3DObservation : public Observation
    {
    public:
  
      static const double TOL;
 
      Type type() const { return COVIS3D; }
 
      std::auto_ptr<kernel::base> getKernel() const;
      void setKernel(const kernel::base& k);
 
      CoViS3DObservation();
      CoViS3DObservation(const kernel::r3xs2p& k);
      ~CoViS3DObservation() {};
    
      void setLoc(Vector3 loc);
      Vector3 getLoc() const;
  
      void setDirection(Vector3 direction);
      Vector3 getDirection() const;
      void setPhiPsi(coord_t phi, coord_t psi);
      coord_pair getPhiPsi() const;
    
      void setCovMatrix(const Matrix3& cov);
      nullable<Matrix3> getCovMatrix() const;
    
      void setGamma(const Vector3& gamma);
      const Vector3& getGamma() const;
      
      void setWeight(weight_t weight);
      weight_t getWeight() const;
  
      void setLeftColor(const Color& color);
      const Color& getLeftColor() const;
      void setRightColor(const Color& color);
      const Color& getRightColor() const;
    
    private:
      kernel::r3xs2p k_;
      nullable<Matrix3> cov_;
      Vector3 gamma_;
    };

}

#endif
