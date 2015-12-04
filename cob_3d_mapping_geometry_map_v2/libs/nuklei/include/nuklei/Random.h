// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */


#ifndef NUKLEI_RANDOM_H
#define NUKLEI_RANDOM_H

#include <nuklei/Definitions.h>
#include <nuklei/LinearAlgebraTypes.h>

namespace nuklei {
  
  /**
   * @brief Implements random variate generators for various distributions.
   */
  class Random
  {
  public:
    Random() {};
    ~Random() {};
    
    /**
     * @brief Seeds the random generators with @p s.
     *
     * This method will seed a built-in GSL generator, and the standard
     * library's generator.
     */
    static void seed(unsigned s);
    
    /**
     * @brief This function returns a double precision floating point number
     * uniformly distributed in the range @f$ [0,1) @f$.
     *
     * The range includes 0.0 but excludes 1.0.
     */
    static double uniform();
    
    /**
     * @brief This function returns a double precision floating point number
     * uniformly distributed in the range @f$ [a,b) @f$.
     *
     * The range includes a but excludes b.
     */
    static double uniform(double a, double b);
    
    /**
     * @brief This function returns a random integer from 0 to n-1 inclusive
     *
     * All integers in the range @f$ [0,n-1] @f$ are produced with equal
     * probability.
     */
    static unsigned long int uniformInt(unsigned long int n);
    
    /**
     * @brief This function returns a random variate from the triangle
     * distribution of zero mean and base @p b.
     *
     * Use the transformation @f$ z = \mu + x @f$ on the numbers returned by
     * this method to obtain a triangle distribution with mean @f$ \mu @f$.
     */
    static double triangle(double b);

    /**
     * @brief This function returns a Gaussian random variate, with mean zero and
     * standard deviation @p sigma.
     *
     * Use the transformation @f$ z = \mu + x @f$ on the numbers returned by
     * this method to obtain a Gaussian distribution with mean @f$ \mu @f$.
     */
    static double gaussian(double sigma);

    /**
     * @brief This function returns a random variate from the beta distribution.
     */
    static double beta(double a, double b);
    
    /**
     * @brief This function returns a random direction vector @f$ v = (x,y) @f$
     * in two dimensions.
     *
     * The vector is normalized such that @f$ |v|^2 = x^2 + y^2 = 1 @f$.
     */
    static Vector2 uniformDirection2d();
    
    /**
     * @brief This function returns a random direction vector @f$ v = (x,y,z)
     * @f$ in three dimensions.
     * 
     * The vector is normalized such that @f$ |v|^2 = x^2 + y^2 + z^2 = 1 @f$.
     */
    static Vector3 uniformDirection3d();
    
    /**
     * @brief This function returns rotation uniformly distributed on @f$ SO(3)
     * @f$.
     */
    static Quaternion uniformQuaternion();
    
    static void printRandomState();
    
    /**
     * @brief Used internally.
     */
    static bool init();
  private:
    static bool initialized_;
  };
  
}

#endif
