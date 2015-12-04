// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_POSE_ESTIMATOR_H
#define NUKLEI_POSE_ESTIMATOR_H

#include <nuklei/KernelCollection.h>
#include <nuklei/ObservationIO.h>
#include <nuklei/Types.h>
#include <nuklei/ProgressIndicator.h>
#include <nuklei/parallelizer_decl.h>

#define NUKLEI_POSE_ESTIMATOR_POLYMORPHIC

namespace nuklei {
  
  const bool WEIGHTED_SUM_EVIDENCE_EVAL = false;
  const double WHITE_NOISE_POWER = 1e-4;
  
  /**
   * @brief Allows to use an external integrand factor, or test reachability
   *
   */
  struct CustomIntegrandFactor
  {
    virtual ~CustomIntegrandFactor() {}
    /**
     * @brief returns true if pose @p k is reachable by the robot */
    virtual bool test(const kernel::se3& k) const = 0;
    /**
     * @brief returns the evaluation at @p k of an additional integrand factor */
    virtual double factor(const kernel::se3& k) const = 0;
  };
  
  struct PoseEstimator
  {
    PoseEstimator(const double locH = 0,
                  const double oriH = .2,
                  const int nChains = -1,
                  const int n = -1,
                  boost::shared_ptr<CustomIntegrandFactor> cif = boost::shared_ptr<CustomIntegrandFactor>(),
                  const bool partialview = false,
                  const bool progress = true);
    
    void load(const std::string& objectFilename,
              const std::string& sceneFilename,
              const std::string& meshfile = "",
              const std::string& viewpointfile = "",
              const bool light = true,
              const bool computeNormals = true);
    
    void load(const KernelCollection& objectModel,
              const KernelCollection& sceneModel,
              const std::string& meshfile = "",
              const Vector3& viewpoint = Vector3::ZERO,
              const bool light = true,
              const bool computeNormals = true);
    
    void usePartialViewEstimation(const Vector3& viewpoint)
    {
      viewpoint_ = viewpoint;
      partialview_ = true;
    }
    
    void setMeshToVisibilityTol(const double meshTol) { meshTol_ = meshTol; }
    
    void setParallelization(const parallelizer::Type t) { parallel_ = t; }
    parallelizer::Type getParallelization() const { return parallel_; }
    
    void setCustomIntegrandFactor(boost::shared_ptr<CustomIntegrandFactor> cif);
    boost::shared_ptr<CustomIntegrandFactor> getCustomIntegrandFactor() const;

    const KernelCollection& getObjectModel() const { return objectModel_; }
    const KernelCollection& getSceneModel() const { return sceneModel_; }

    kernel::se3 modelToSceneTransformation(const boost::optional<kernel::se3>& gtTransfo = boost::none) const;
    
    double findMatchingScore(const kernel::se3& pose) const;
    
    void writeAlignedModel(const std::string& filename,
                           const kernel::se3& t) const;
    
  private:
    
    Vector3 viewpointInFrame(const kernel::se3& frame) const;
    
    // Temperature function (cooling factor)
    static double Ti(const unsigned i, const unsigned F);
    
    /**
     * This function implements the algorithm of Fig. 2: Simulated annealing
     * algorithm of the ACCV paper.
     * - T_j is given by @c temperature
     * - u is given by Random::uniform()
     * - w_j is @c currentPose
     */
    void
    metropolisHastings(kernel::se3& currentPose,
                       weight_t &currentWeight,
                       const weight_t temperature,
                       const bool firstRun,
                       const int n) const;
    
    kernel::se3
    mcmc(const int n) const;
    bool recomputeIndices(std::vector<int>& indices,
                          const kernel::se3& nextPose,
                          const int n) const;

    KernelCollection objectModel_;
    double objectSize_;
    KernelCollection sceneModel_;
    Vector3 viewpoint_;
    KernelCollection::EvaluationStrategy evaluationStrategy_;
    double loc_h_;
    double ori_h_;
    int nChains_;
    int n_;
    boost::shared_ptr<CustomIntegrandFactor> cif_;
    bool partialview_;
    boost::shared_ptr<ProgressIndicator> pi_;
    bool progress_;
    parallelizer::Type parallel_;
    double meshTol_;
  };
  
}

#endif
