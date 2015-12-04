// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_GENERIC_KERNEL_H
#define NUKLEI_GENERIC_KERNEL_H

#include <nuklei/Definitions.h>
#include <nuklei/LinearAlgebra.h>

namespace nuklei {


  // ****************************************** //
  // Selectors
  // ****************************************** //

  namespace groupS
  {
    struct r3 { typedef Vector3 element_t; };
    struct so3 { typedef Quaternion element_t; };
    struct s2 { typedef Vector3 element_t; };
    struct s2p { typedef Vector3 element_t; };
  }

  // Use fast function implementations (e.g. FastACos)
  namespace func_implS
  {
    struct approx {};
    struct exact {};
  }

  namespace shapeS
  {
    struct box {};
    struct triangle {};
    struct gauss {};
  }

  // Use squared distances, to avoid to sqrt then ^2
  namespace squaredS
  {
    struct yes {};
    struct no {};
  }
  
  // Kernel integrates to 1 or kernel max == 1
  namespace value_scaleS
  {
    struct normalized {};
    struct max1 {};
  }

  // Use intrinsic width (cf. Watson/VMF), or a dist-proportional width
  namespace h_scaleS
  {
    struct intrinsic {};
    struct dist {};
  }
  
  // ****************************************** //
  // Metric Definition
  // ****************************************** //

  template
  <
  class Group,
  class FunctionImpl = func_implS::approx,
  class Squared = squaredS::no
  >
  struct dist {};

  template<class FunctionImpl>
  struct dist<groupS::r3, FunctionImpl, squaredS::no>
  {
    static coord_t d(const Vector3 &v1, const Vector3 &v2)
    {
      return (v1-v2).Length();
    }
  };

  template<class FunctionImpl>
  struct dist<groupS::r3, FunctionImpl, squaredS::yes>
  {
    static coord_t d(const Vector3 &v1, const Vector3 &v2)
    {
      return (v1-v2).SquaredLength();
    }
  };

  template<>
  struct dist<groupS::so3, func_implS::exact, squaredS::no>
  {
    static coord_t d(const Quaternion &q1, const Quaternion &q2)
    {
      coord_t productAbs = std::fabs(q1.Dot(q2));
      return 2*ACos( productAbs );
    }
  };

  template<>
  struct dist<groupS::so3, func_implS::approx, squaredS::no>
  {
    static coord_t d(const Quaternion &q1, const Quaternion &q2)
    {
      coord_t productAbs = std::fabs(q1.Dot(q2));
      return 2*FastACos(productAbs);
    }
  };

  template<>
  struct dist<groupS::s2, func_implS::exact, squaredS::no>
  {
    static coord_t d(const Vector3 &v1, const Vector3 &v2)
    {
      coord_t product = v1.Dot(v2);
      return ACos(product);
    }
  };

  template<>
  struct dist<groupS::s2, func_implS::approx, squaredS::no>
  {
    static coord_t d(const Vector3 &v1, const Vector3 &v2)
    {
      coord_t product = v1.Dot(v2);
      return FastACos(product);
    }
  };

  template<>
  struct dist<groupS::s2p, func_implS::exact, squaredS::no>
  {
    static coord_t d(const Vector3 &v1, const Vector3 &v2)
    {
      coord_t product = std::fabs(v1.Dot(v2));
      return ACos(product);
    }
  };

  template<>
  struct dist<groupS::s2p, func_implS::approx, squaredS::no>
  {
    static coord_t d(const Vector3 &v1, const Vector3 &v2)
    {
      coord_t product = std::fabs(v1.Dot(v2));
      return FastACos(product);
    }
  };

  // ****************************************** //
  // Random Elements
  // ****************************************** //

  template<class Group>
  struct random_element {};

  template<>
  struct random_element<groupS::so3>
  {
    static Quaternion r()
    {
      return Random::uniformQuaternion();
    }
  };

  template<>
  struct random_element<groupS::s2>
  {
    static Vector3 r()
    {
      return Random::uniformDirection3d();
    }
  };

  template<>
  struct random_element<groupS::s2p>
  {
    static Vector3 r()
    {
      return Random::uniformDirection3d();
    }
  };

  // ****************************************** //
  // Kernel Shapes
  // ****************************************** //

  template<class Shape, class FunctionImpl, class Squared = squaredS::no>
  struct shape_function {};

  template<class FunctionImpl>
  struct shape_function<shapeS::box, FunctionImpl, squaredS::no>
  {
    static coord_t s(const coord_t h, const coord_t d)
    {
      if (std::fabs(d) > h) return 0;
      else return 1;
    }
    static coord_t cut_point(const coord_t h)
    {
      return std::fabs(h);
    }
  };

  template<class FunctionImpl>
  struct shape_function<shapeS::triangle, FunctionImpl, squaredS::no>
  {
    static coord_t s(const coord_t h, const coord_t d)
    {
      coord_t abs_d = std::fabs(d);
      if (abs_d > gauss_equiv_spread*h) return 0;
      else return 1 - abs_d/(gauss_equiv_spread*h);
    }
    static coord_t cut_point(const coord_t h)
    {
      return gauss_equiv_spread*std::fabs(h);
    }
    
    static const coord_t gauss_equiv_spread;
  };

  template<class FunctionImpl>
  const coord_t shape_function
  <
    shapeS::triangle,
    FunctionImpl,
    squaredS::no
  >::gauss_equiv_spread = 2;

  template<class FunctionImpl>
  struct shape_function<shapeS::triangle, FunctionImpl, squaredS::yes>
  {
    static coord_t s(const coord_t h, const coord_t d)
    {
      coord_t abs_d = std::fabs(d);
      if (abs_d > gauss_equiv_spread*h*gauss_equiv_spread*h) return 0;
      else return 1 - std::sqrt(abs_d)/(gauss_equiv_spread*h);
    }
    static coord_t cut_point(const coord_t h)
    {
      return gauss_equiv_spread*std::fabs(h);
    }

    static const coord_t gauss_equiv_spread;
  };

  template<class FunctionImpl>
  const coord_t shape_function
  <
    shapeS::triangle,
    FunctionImpl,
    squaredS::yes
  >::gauss_equiv_spread = 2;

  template<>
  struct shape_function<shapeS::gauss, func_implS::exact, squaredS::no>
  {
    static coord_t s(const coord_t h, const coord_t d)
    {
      return std::exp( - d*d / (2*h*h) );
    }
    static coord_t cut_point(const coord_t h)
    {
      return std::numeric_limits<coord_t>::infinity();
    }
  };

  template<>
  struct shape_function<shapeS::gauss, func_implS::approx, squaredS::no>
  {
    static coord_t s(const coord_t h, const coord_t d)
    {
      return FastNegExp( d*d / (2*h*h) );
    }
    static coord_t cut_point(const coord_t h)
    {
      return std::numeric_limits<coord_t>::infinity();
    }
  };
  

  // ****************************************** //
  // Kernels
  // ****************************************** //

  template
  <
  class Group,
  class Shape = shapeS::triangle,
  class FunctionImpl = func_implS::approx
  >
  struct unnormalized_shape_dist_kernel
  {
    typedef Group group_t;
    typedef FunctionImpl function_impl_t;
    
    typedef shape_function<Shape, FunctionImpl> shape_function_t;
    typedef dist<Group, FunctionImpl> dist_t;
    
    template<class T>
    static coord_t eval(const T &mean, const coord_t h, const T &p)
    {
      return shape_function_t::s(h, dist_t::d(mean, p));
    }
    
    static coord_t cut_point(const coord_t h)
    {
      return shape_function_t::cut_point(h);
    }
  };
  
  template<class FunctionImpl>
  struct unnormalized_shape_dist_kernel
  <groupS::r3, shapeS::triangle, FunctionImpl>
  {
    typedef groupS::r3 group_t;
    typedef FunctionImpl function_impl_t;
    
    typedef
    shape_function<shapeS::triangle, FunctionImpl, squaredS::yes>
    shape_function_t;
    
    typedef
    dist<groupS::r3, FunctionImpl, squaredS::yes>
    dist_t;
    
    static coord_t eval(const Vector3 &mean, const coord_t h, const Vector3 &p)
    {
      return shape_function_t::s(h, dist_t::d(mean, p));
    }

    static coord_t cut_point(const coord_t h)
    {
      return shape_function_t::cut_point(h);
    }
  };
  
  template
  <
  class ValueScale = value_scaleS::max1,
  class FunctionImpl = func_implS::approx
  >
  struct watson_kernel
  {
    typedef groupS::so3 group_t;
    typedef FunctionImpl function_impl_t;

    static coord_t eval(const Quaternion &mean, const coord_t h,
                        const Quaternion &p)
    {
      return eval(mean, h, p, ValueScale(), FunctionImpl());
    }

    static coord_t eval(const Quaternion &mean, const coord_t h,
                        const Quaternion &p,
                        value_scaleS::max1, func_implS::exact)
    {
      coord_t arg = std::pow(mean.Dot(p), 2);
      return std::exp( h*(arg-1) );
    }

    static coord_t eval(const Quaternion &mean, const coord_t h,
                        const Quaternion &p,
                        value_scaleS::max1, func_implS::approx)
    {
      coord_t arg = std::pow(mean.Dot(p), 2);
      return FastNegExp( h*(1-arg) );
    }
    
    // Overflows when h >~ 1000
    template<class FI>
    static coord_t eval(const Quaternion &mean, const coord_t h,
                        const Quaternion &p,
                        value_scaleS::normalized, FI)
    {
      coord_t arg = std::pow(mean.Dot(p), 2);
      return std::exp( h*arg ) / confluentHypergeometric1F1(.5, 2, h);
    }
  };

  template
  <
  class Group,
  class ValueScale = value_scaleS::max1,
  class FunctionImpl = func_implS::approx,
  class HScale = h_scaleS::dist
  >
  struct von_mises_fisher_kernel
  {
    typedef Group group_t;
    typedef typename Group::element_t element_t;
    typedef FunctionImpl function_impl_t;
    
    // Generic evaluation
    static coord_t eval(const element_t &mean, const coord_t h,
                        const element_t &p)
    {
      return eval(mean, h, p, Group(), ValueScale(), FunctionImpl(), HScale());
    }

    // Heuristic for computing h from angle_h
    static coord_t h_from_angle_h(coord_t angle_h)
    {
      return h_from_angle_h(angle_h, Group());
    }
    
    // Inverse heuristic
    static coord_t angle_h_from_h(coord_t h)
    {
      return angle_h_from_h(h, Group());
    }
    
  private:
        
    // Heuristic for computing h from angle_h (SO3)
    static coord_t h_from_angle_h(coord_t angle_h, groupS::so3)
    {
      NUKLEI_FAST_ASSERT(0 <= angle_h && angle_h <= M_PI);
      return 1./( 1-std::cos(angle_h/2) );
    }
    
    // Inverse heuristic (SO3)
    static coord_t angle_h_from_h(coord_t h, groupS::so3)
    {
      NUKLEI_FAST_ASSERT(1 <= h);
      return 2*FastACos(1-1./h);
    }

    // Heuristic for computing h from angle_h (S^2_+)
    static coord_t h_from_angle_h(coord_t angle_h, groupS::s2p)
    {
      NUKLEI_FAST_ASSERT(0 <= angle_h && angle_h <= M_PI);
      return 1./( 1-std::cos(angle_h) );
    }
    
    // Inverse heuristic (S^2_+)
    static coord_t angle_h_from_h(coord_t h, groupS::s2p)
    {
      NUKLEI_FAST_ASSERT(1 <= h);
      return FastACos(1-1./h);
    }

    // Heuristic for computing h from angle_h (S^2)
    static coord_t h_from_angle_h(coord_t angle_h, groupS::s2)
    {
      NUKLEI_FAST_ASSERT(0 <= angle_h && angle_h <= M_PI);
      return 1./( 1-std::cos(angle_h) );
    }
    
    // Inverse heuristic (S^2)
    static coord_t angle_h_from_h(coord_t h, groupS::s2)
    {
      NUKLEI_FAST_ASSERT(1 <= h);
      return FastACos(1-1./h);
    }
    
    // Evaluation, dist-proportional width
    template<class GP, class VS, class FI>
    static coord_t eval(const element_t &mean, const coord_t h,
                        const element_t &p,
                        GP, VS, FI, h_scaleS::dist)
    {
      return eval(mean, h_from_angle_h(h), p,
                  GP(), VS(), FI(), h_scaleS::intrinsic());
    }
    
    static coord_t dist_exponent(const element_t &mean,
                                 const element_t &p,
                                 groupS::so3)
    {
      return std::fabs(mean.Dot(p));
    }
    
    static coord_t dist_exponent(const element_t &mean,
                                 const element_t &p,
                                 groupS::s2p)
    {
      return std::fabs(mean.Dot(p));
    }
    
    static coord_t dist_exponent(const element_t &mean,
                                 const element_t &p,
                                 groupS::s2)
    {
      return mean.Dot(p);
    }
    
    // Normalized VMF SO3 evaluation
    template<class FI>
    static coord_t eval(const element_t &mean, const coord_t h,
                        const element_t &p,
                        groupS::so3, value_scaleS::normalized,
                        FI, h_scaleS::intrinsic)
    {
      coord_t arg = dist_exponent(mean, p, groupS::so3());

      // This function should return
      // std::exp( h*arg ) * h / (4 * M_PI * M_PI * besselI1(h));
      // However, besselI1 quickly overflows, and it is slow.
      // The below approximation works very well.

      // See "Clustering Documents with an Exponential-Family Approximation of
      // the Dirichlet Compound Multinomial Distribution"
      // http://cseweb.ucsd.edu/~elkan/edcm.pdf
      
      const coord_t alpha = 1+ h*h;
      const coord_t sqrt_alpha = std::sqrt(alpha);
      const coord_t eta = sqrt_alpha + std::log(h) - std::log(1+sqrt_alpha);
      const coord_t minus_log_sqrt_2_pi = -0.918938533204673;
      const coord_t log_bessel_I1_h = (minus_log_sqrt_2_pi + eta -
                                       .25*std::log(alpha));
      return std::exp( h*arg - log_bessel_I1_h ) * h / (4 * M_PI * M_PI);
    }
    
    // Normalized VMF S2 evaluation
    template<class FI>
    static coord_t eval(const element_t &mean, const coord_t h,
                        const element_t &p,
                        groupS::s2, value_scaleS::normalized,
                        FI, h_scaleS::intrinsic)
    {
      coord_t arg = dist_exponent(mean, p, groupS::s2());
      return (h / (2 * M_PI * (1-std::exp(-2*h)))) * std::exp( h*(arg-1) );
    }

    template<class GP>
    static coord_t eval(const element_t &mean, const coord_t h,
                        const element_t &p,
                        GP, value_scaleS::max1, func_implS::exact,
                        h_scaleS::intrinsic)
    {
      coord_t arg = dist_exponent(mean, p, GP());
      return std::exp( h*(arg-1) );
    }

    template<class GP>
    static coord_t eval(const element_t &mean, const coord_t h,
                        const element_t &p,
                        GP, value_scaleS::max1, func_implS::approx,
                        h_scaleS::intrinsic)
    {
      coord_t arg = dist_exponent(mean, p, GP());
      return FastNegExp( h*(1-arg) );
    }
  
  };
  
  // ****************************************** //
  // Kernel Sampling
  // ****************************************** //

  template<class Kernel>
  typename Kernel::group_t::element_t
  importance_sampling_uniform_proposal(typename Kernel::group_t::element_t mean,
                                       coord_t h)
  {
    typename Kernel::group_t::element_t q;
    coord_t kernelMaxPoint = Kernel::eval(mean, h, mean);
    for (;;)
    {
      q = random_element<typename Kernel::group_t>::r();
      coord_t e = Kernel::eval(mean, h, q);
      // if e == 0, avoid drawing a random number
      if (e == 0) continue;
      if (kernelMaxPoint*Random::uniform() < e)
        break;
    }
    return q;
  }

  template<class Kernel>
  typename Kernel::group_t::element_t
  importance_sampling_similar_proposal(typename Kernel::group_t::element_t mean,
                                       coord_t h_target, coord_t h_proposal)
  {
    typename Kernel::group_t::element_t q;
    coord_t targetMaxPoint = Kernel::eval(mean, h_target, mean);
    coord_t proposalMaxPoint = Kernel::eval(mean, h_proposal, mean);
    for (;;)
    {
      q = importance_sampling_uniform_proposal<Kernel>(mean, h_proposal);
      coord_t e_target = Kernel::eval(mean, h_target, q)/targetMaxPoint;
      coord_t e_proposal = Kernel::eval(mean, h_proposal, q)/proposalMaxPoint;
      NUKLEI_ASSERT(e_target <= e_proposal);
      // if e_target == 0, avoid drawing a random number
      if (e_target == 0) continue;
      if (e_proposal*Random::uniform() < e_target)
        break;
    }
    return q;
  }
  

  template<class Kernel> struct sampler {};
  
  
  // R^3

  template<class FunctionImpl>
  struct sampler
  <
  unnormalized_shape_dist_kernel<groupS::r3, shapeS::gauss, FunctionImpl>
  >
    {
      typedef unnormalized_shape_dist_kernel
      <groupS::r3, shapeS::gauss, FunctionImpl> kernel_t;
      typedef typename kernel_t::group_t group_t;
      typedef typename kernel_t::function_impl_t function_impl_t;

      static Vector3 s(const Vector3 &mean, const coord_t h)
      {
        Vector3 s(mean);
        for (int i = 0; i < 3; ++i)
          s[i] += Random::gaussian(h);
        return s;
      }
    };
  
  template<class Shape, class FunctionImpl>
  struct sampler
  <
  unnormalized_shape_dist_kernel<groupS::r3, Shape, FunctionImpl>
  >
  {
    typedef unnormalized_shape_dist_kernel
    <groupS::r3, Shape, FunctionImpl> kernel_t;
    typedef typename kernel_t::group_t group_t;
    typedef typename kernel_t::function_impl_t function_impl_t;
        
    static Vector3 s(const Vector3 &mean, const coord_t h)
    {
      Vector3 zero(Vector3::ZERO);
      coord_t kernelMaxPoint = kernel_t::eval(zero, h, zero);
      coord_t cutPoint = kernel_t::cut_point(h);
      Vector3 r;
      for (;;)
      {
        r.X() = Random::uniform(-cutPoint, cutPoint);
        r.Y() = Random::uniform(-cutPoint, cutPoint);
        r.Z() = Random::uniform(-cutPoint, cutPoint);
        coord_t e = kernel_t::eval(zero, h, r);
        // if e == 0, avoid drawing a random number
        if (e == 0) continue;
        if (kernelMaxPoint*Random::uniform() < e)
          break;
      }
      for (int i = 0; i < 3; ++i)
        r[i] += mean[i];
      return r;
    }
  };
  
  // SO(3) ( S^3_+ )

  template<class Shape, class FunctionImpl>
  struct sampler
  <
  unnormalized_shape_dist_kernel<groupS::so3, Shape, FunctionImpl>
  >
    {
      typedef unnormalized_shape_dist_kernel
      <groupS::so3, Shape, FunctionImpl> kernel_t;
      typedef typename kernel_t::group_t group_t;
      typedef typename kernel_t::function_impl_t function_impl_t;

      static Quaternion s(const Quaternion &mean, const coord_t h)
      {
        if (h < .1) NUKLEI_WARN("S^2 IS with h=" + stringify(h) + " is slow.");
        // The max angle between two quaternions is pi/2.
        // Actually no, this is taken care of in the metric function
        // h /= 2.0;
        return importance_sampling_uniform_proposal<kernel_t>(mean, h);
      }
    };

  template
  <
  class ValueScale,
  class FunctionImpl
  >
  struct sampler
  <
  watson_kernel<ValueScale, FunctionImpl>
  >
  {
    typedef watson_kernel<FunctionImpl> kernel_t;
    typedef typename kernel_t::group_t group_t;
    typedef typename kernel_t::function_impl_t function_impl_t;
    
    static Quaternion s(const Quaternion &mean, const coord_t h)
    {
      return importance_sampling_uniform_proposal<kernel_t>(mean, h);
    }
  };
  
  template
  <
  class Group,
  class ValueScale,
  class FunctionImpl,
  class HScale
  >
  struct sampler
  <
  von_mises_fisher_kernel<Group, ValueScale, FunctionImpl, HScale>
  >
  {
    typedef
    von_mises_fisher_kernel<Group, ValueScale, FunctionImpl, HScale>
    kernel_t;
    typedef typename kernel_t::group_t group_t;
    typedef typename group_t::element_t element_t;
    typedef typename kernel_t::function_impl_t function_impl_t;

    static element_t s(const element_t &mean, const coord_t h)
    {
      return s(mean, h, Group(), HScale());
    }
    
  private:
    
    template<class GP>
    static element_t s(const element_t &mean, const coord_t h,
                       GP, h_scaleS::dist)
    {
      return s(mean, kernel_t::h_from_angle_h(h), GP(), h_scaleS::intrinsic());
    }
    
    static Quaternion s(const Quaternion &mean, const coord_t h,
                        groupS::so3, h_scaleS::intrinsic)
    {
      //return importance_sampling_uniform_proposal<kernel_t>(mean, h);
      const int d = 4;
      const coord_t t1 = std::sqrt(4*h*h + (d-1)*(d-1));
      const coord_t b = (-2*h+t1)/(d-1);
      const coord_t x0 = (1-b)/(1+b);
      
      const coord_t m = (d-1.0)/2;
      const coord_t c = h*x0 + (d-1) * std::log(1-x0*x0);
      coord_t w = -1;
      for (;;)
      {
        coord_t z = Random::beta(m,m);
        w = (1-(1+b)*z)/(1-(1-b)*z);
        coord_t t = h*w + (d-1)*std::log(1-x0*w)-c;
        if ( ! (t < std::log( Random::uniform() )) )
          break;
      }
      Vector3 v = std::sqrt(1 - w*w) * Random::uniformDirection3d();
      Quaternion q(v.X(), v.Y(), v.Z(), w);
      NUKLEI_FAST_DEBUG_ASSERT(std::fabs(1-q.Length()) < 1e-6);
      Quaternion sample = (mean * Quaternion(0, 0, 0, 1).Conjugate()) * q;
      sample.Normalize();
      // Technically, in SO3, sample is equivalent to -sample.
      // We randomize the sign of sample to make this equivalence explicit.
      if (Random::uniformInt(2))
        return sample;
      else
        return -sample;
    }

    static Vector3 s(const Vector3 &mean, const coord_t h,
                     groupS::s2, h_scaleS::intrinsic)
    {
      const int d = 3;
      const coord_t t1 = std::sqrt(4*h*h + (d-1)*(d-1));
      const coord_t b = (-2*h+t1)/(d-1);
      const coord_t x0 = (1-b)/(1+b);
      
      const coord_t m = (d-1.)/2;
      const coord_t c = h*x0 + (d-1) * std::log(1-x0*x0);
      coord_t w = -1;
      for (;;)
      {
        coord_t z = Random::beta(m,m);
        w = (1-(1+b)*z)/(1-(1-b)*z);
        coord_t t = h*w + (d-1)*std::log(1-x0*w)-c;
        if ( ! (t < std::log( Random::uniform() )) )
          break;
      }
      Vector2 v = std::sqrt(1 - w*w) * Random::uniformDirection2d();
      Vector3 q(v.X(), v.Y(), w);
      NUKLEI_FAST_DEBUG_ASSERT(std::fabs(1-q.Length()) < 1e-6);
      Vector3 cross = Vector3::UNIT_Z.Cross(mean);
      Quaternion rotation(1+Vector3::UNIT_Z.Dot(mean),
                          cross.X(),
                          cross.Y(),
                          cross.Z());
      rotation.Normalize();
      return la::normalized(rotation.Rotate(q));
    }
    
    static Vector3 s(const Vector3 &mean, const coord_t h,
                     groupS::s2p, h_scaleS::intrinsic)
    {
      Vector3 sample = s(mean, h, groupS::s2(), h_scaleS::intrinsic());
      // Technically, in s2p, sample is equivalent to -sample.
      // We randomize the sign of sample to make this equivalence explicit.
      if (Random::uniformInt(2))
        return sample;
      else
        return -sample;
    }      
  };
  
  // S^2

  template<class Shape, class FunctionImpl>
  struct sampler
  <
  unnormalized_shape_dist_kernel<groupS::s2, Shape, FunctionImpl>
  >
    {
      typedef unnormalized_shape_dist_kernel
      <groupS::s2, Shape, FunctionImpl> kernel_t;
      typedef typename kernel_t::group_t group_t;
      typedef typename kernel_t::function_impl_t function_impl_t;

      static Vector3 s(const Vector3 &mean, const coord_t h)
      {
        if (h < .1) NUKLEI_WARN("S^2 IS with h=" + stringify(h) + " is slow.");
        return importance_sampling_uniform_proposal<kernel_t>(mean, h);
      }
    };


  // S^2_+

  template<class Shape, class FunctionImpl>
  struct sampler
  <
  unnormalized_shape_dist_kernel<groupS::s2p, Shape, FunctionImpl>
  >
    {
      typedef unnormalized_shape_dist_kernel
      <groupS::s2p, Shape, FunctionImpl> kernel_t;
      typedef typename kernel_t::group_t group_t;
      typedef typename kernel_t::function_impl_t function_impl_t;

      static Vector3 s(const Vector3 &mean, const coord_t h)
      {
        if (h < .1) NUKLEI_WARN("S^2 IS with h=" + stringify(h) + " is slow.");
        return importance_sampling_uniform_proposal<kernel_t>(mean, h);
      }
    };

}

#endif
