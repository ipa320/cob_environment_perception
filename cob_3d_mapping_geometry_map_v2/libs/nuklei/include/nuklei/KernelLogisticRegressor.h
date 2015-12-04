// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */


#ifndef NUKLEI_KERNEL_LOGISTIC_REGRESSOR_H
#define NUKLEI_KERNEL_LOGISTIC_REGRESSOR_H


#include <nuklei/KernelCollection.h>

namespace nuklei
{
  
  /**
   * @ingroup kernels
   * @brief Implements kernel logistic regression.
   *
   * This class implements a two-class nonlinear classifier for @f$
   * SE(3) @f$ data. KLR is discussed in @ref kernels_klr.
   *
   * This class is based on @c libklr, which is provided with Makoto Yamada's <a
   * href="http://sugiyama-www.cs.titech.ac.jp/~yamada/iwklr.html">IWKLR</a>.
   */
  struct KernelLogisticRegressor
  {
    /** @brief */
    KernelLogisticRegressor() {}
    /**
     * @brief Imports input from @p data and @p labels, and computes the Gram
     * matrix of the data.
     *
     * The two structures @p data and @p labels should contain the same number
     * of elements. The @f$ i^{\mathrm{th}} @f$ element of @p labels indicates
     * the class of the @f$ i^{\mathrm{th}} @f$ element of @p data. The two
     * allowed values in @p labels are @c 1 and @c 2.
     */
    KernelLogisticRegressor(const KernelCollection &data,
                             const std::vector<int>& labels);    
    /**
     * @brief Imports input from @p data and @p labels and the Gram matrix @p
     * gramMatrix.
     *
     * The two structures @p data and @p labels should contain the same number
     * of elements. The @f$ i^{\mathrm{th}} @f$ element of @p labels indicates
     * the class of the @f$ i^{\mathrm{th}} @f$ element of @p data. The two
     * allowed values in @p labels are @c 1 and @c 2.
     *
     * The data held in @p gramMatrix should correspond to the Gram matrix of @p
     * data.
     */
    KernelLogisticRegressor(const KernelCollection &data,
                             const GMatrix& gramMatrix,
                             const std::vector<int>& labels);    
    /**
     * @brief Imports input from @p data and @p labels, and computes the Gram
     * matrix of the data.
     *
     * The two structures @p data and @p labels should contain the same number
     * of elements. The @f$ i^{\mathrm{th}} @f$ element of @p labels indicates
     * the class of the @f$ i^{\mathrm{th}} @f$ element of @p data. The two
     * allowed values in @p labels are @c 1 and @c 2.
     *
     * This method also resets all the other member variables of the class.
     */
    void setData(const KernelCollection &data,
                 const std::vector<int>& labels);
    
    /**
     * @brief Computes KLR weights.
     *
     * The implementation follows <a
     * href="http://dx.doi.org/10.1016/j.sigpro.2009.06.001">Semi-supervised
     * speaker identification under covariate shift</a>.
     */
    void train(const double delta = 0.0001, const unsigned itrNewton = 5);
    
    /**
     * @brief Returns @c true if the classifier has been trained.
     */
    bool isTrained()
    {
      return vklr_;
    };
    
    /**
     * @brief Returns a pair of values which indicate the probability of classes
     * @c 1 and @c 2.
     */
    Vector2 test(const kernel::base &t) const;
    /**
     * @brief Returns the probabilities of classes @c 1 and @c 2 for all data
     * points in @p testSet.
     *
     * This method returns a @f$ 2 \times n @f$ matrix, where @f$ n @f$ is the
     * number of elements in @p testSet.
     */
    GMatrix test(const KernelCollection &testSet) const;
        
    /**
     * @brief Returns KLR weights.
     *
     * The parameter @p delta is the regularization constant.
     */
    const GMatrix& vklr() const
    {
      NUKLEI_TRACE_BEGIN();
      NUKLEI_ASSERT(vklr_);
      return *vklr_;
      NUKLEI_TRACE_END();
    }
  private:
    void computeGramMatrix();
    
    KernelCollection trainSet_;
    GMatrix gramMatrix_;
    std::vector<int> labels_;
    boost::optional<GMatrix> vklr_;
  };
}


#endif
