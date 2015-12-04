// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */

#ifndef NUKLEI_SPECIALIZED_KERNEL_H
#define NUKLEI_SPECIALIZED_KERNEL_H

#include <iostream>
#include <boost/cast.hpp>
#include <boost/ptr_container/ptr_array.hpp>

#include <nuklei/GenericKernel.h>
#include <nuklei/Descriptor.h>
#include <nuklei/Indenter.h>

namespace nuklei {
  
  /** @brief Namespace containing kernel definitions. */
  namespace kernel
  {
    class se3;
    class base;
    
    //const bitfield_t EXACT = (1 << 0);
    
    /**
     * @ingroup kernels
     * @brief Polymorphic kernel class.
     *
     * This class serves as an abstract base for kernels defined on
     * various subspaces of @f$ SE(3) @f$. See @ref kernels for a
     * discussion of what a kernel is. See @ref programming_paradigm
     * for an overview and motivation of the kernel class hierarchy.
     *
     * See @ref programming_paradigm for an explanation of why some of
     * the methods begin with @p poly.
     *
     * In Nuklei, the kernel classes (kernel::base and its descendants) play the
     * double role of representing kernels and points. For instance, there is no
     * class specifically designed for holding an @f$ SE(3) @f$ point, the class
     * kernel::se3 is used for that purpose.
     */
    class base
    {
    public:
      
      /** @brief std::auto_ptr for kernel::base. */
      typedef std::auto_ptr< kernel::base > ptr;
      
      /** @brief Explicit query of a kernel's type. See @ref type for more info. */
      typedef enum { R3 = 0, R3XS2, R3XS2P, SE3, UNKNOWN } Type;
      /** @brief Explicit query of a kernel's type. See @ref type for more info. */
      static const Type defaultType = SE3;
      /** @brief Explicit query of a kernel's type. See @ref type for more info. */
      static const std::string TypeNames[];
      /** @} */
      
    protected:
      base() : w_(1.), flag1_(0), bitfield_(0) {}
      
      base(const base& k) : w_(k.w_), flag1_(k.flag1_), bitfield_(k.bitfield_)
      {
        if (k.hasDescriptor()) desc_ = k.desc_->clone();
      }
      
      base& operator=(const base& k)
      {
        w_ = k.w_;
        flag1_ = k.flag1_;
        bitfield_ = k.bitfield_;
        // no problem with auto-assign here.
        if (k.hasDescriptor()) desc_ = k.desc_->clone();
        else desc_.reset();
        return *this;
      }
      
    public:
      virtual ~base() {}
      
      virtual void assertConsistency() const = 0;
      
      /** @brief Calls polyPrint */
      friend std::ostream& operator<<(std::ostream &out, const kernel::base &k);
      /**
       * @brief Prints the kernel parameters to the provided stream.
       *
       * The purpose of this method is debugging. It shouldn't be used
       * for serialization.
       */
      virtual std::ostream& polyPrint(std::ostream &out) const = 0;
      
      /** @brief Weight-based kernel comparator. */
      bool operator<(const kernel::base& k) const { return w_ < k.w_; }
      /** @brief Weight-based kernel comparator. */
      bool operator>(const kernel::base& k) const { return w_ > k.w_; }
      
      /** @brief Clone the kernel. */
      virtual base::ptr clone() const = 0;
      /** @brief Create a new kernel of the same type. */
      virtual base::ptr create() const = 0;
      
      /** @brief Get the location component of the kernel. */
      virtual Vector3 getLoc() const = 0;
      /** @brief Set the location component of the kernel. */
      virtual void setLoc(const Vector3 &v) = 0;
      
      /** @brief Get the location bandwidth. */
      virtual coord_t getLocH() const = 0;
      /** @brief Set the location bandwidth. */
      virtual void setLocH(const coord_t h) = 0;
      /**
       * @brief Get the orientation bandwidth.
       *
       * This method is implemented in child classes. In classes that do not
       * have an orientation (or a direction), this method returns 0.
       */
      virtual coord_t getOriH() const = 0;
      /**
       * @brief Set the orientation bandwidth.
       *
       * This method is implemented in child classes. In classes that do not
       * have an orientation (or a direction), this method does nothing.
       */
      virtual void setOriH(const coord_t h) = 0;
      
      /** @brief Get the "kernel type", i.e., its domain of definition. */
      virtual Type polyType() const = 0;
      
      /** @brief Evaluate the kernel at the location/orientation of @p k. */
      virtual coord_t polyEval(const base &k) const = 0;
      /** @brief Distance (from the origin) at which the value of the kernel becomes zero. */
      virtual coord_t polyCutPoint() const = 0;
      /** @brief Get a sample from the kernel. */
      virtual base::ptr polySample() const = 0;
      /**
       * @brief Get an @f$ SE(3) @f$ sample from the kernel.
       *
       * Take a sample from the kernel. If the kernel is defined on a
       * subspace of @f$ SE(3) @f$, make the sample @f$ SE(3) @f$ by
       * selecting random values from a uniform distribution for the
       * extra DOFs.
       */
      virtual std::auto_ptr<kernel::se3> polySe3Sample() const = 0;
      /**
       * @brief Get an @f$ SE(3) @f$ version of the kernel.
       *
       * Take a sample from the kernel. If the kernel is defined on a
       * subspace of @f$ SE(3) @f$, make it @f$ SE(3) @f$ by
       * selecting random values from a uniform distribution for the
       * extra DOFs.
       */
      virtual std::auto_ptr<kernel::se3> polySe3Proj() const = 0;
      
      /**
       * @addtogroup matrix_transfo
       * @{
       */
      /**
       * @brief Transforms @p *this with @p k and returns the
       * result. (See @ref matrix_transfo.)
       */
      virtual base::ptr polyTransformedWith(const kernel::se3& k) const = 0;
      /**
       * @brief Transforms @p *this with @p k and sets @p *this to the
       * result. (See @ref matrix_transfo.)
       */
      virtual void polyMakeTransformWith(const kernel::se3& k) = 0;
      /**
       * @brief Projects @p *this onto @p k and returns the
       * result. (See @ref matrix_transfo.)
       */
      virtual base::ptr polyProjectedOn(const kernel::se3& k) const = 0;
      /** @} */
      /**
       * @brief Interpolates between @p *this and @p k.
       *
       * For @p x = 0, this function returns @p *this. If @p x = 1, it
       * returns @p k. For orientation, interpolation is done in
       * Euclidean space. Slerp should be used instead. To be fixed
       * soon.
       */
      virtual base::ptr polyLinearInterpolation(const kernel::base& k,
                                                const coord_t x = .5) const = 0;
      /** @brief Used internally. */
      virtual void polyUpdateWidth(const kernel::base& k,
                                   const coord_t x = .5) = 0;
      /**
       * @brief Returns a std::pair containing the distance in
       * position and orientation between @p *this and @p k.
       */
      virtual coord_pair polyDistanceTo(const kernel::base& k) const = 0;
      
      /**
       * @brief Weight accessor for kernel::base. The accessor is used
       * internally. Normal Nuklei users will generally not need it.
       *
       * Example:
       * @code
       * WeightAccessor wa;
       * kernel::base& k = ...
       * wa(k) == k.getWeight() // true
       * @endcode
       */
      struct WeightAccessor
      {
        weight_t operator() (const base& k) const
        { return k.w_; }
        weight_t operator() (const base* k) const
        { return k->w_; }
      };
      
      /** @brief Returns this kernel's weight. */
      weight_t getWeight() const { return w_; }
      /** @brief Sets this kernel's weight. */
      void setWeight(const weight_t w) { w_ = w; }
      
      bool hasDescriptor() const { return desc_.get() != NULL; }
      Descriptor& getDescriptor()
      { NUKLEI_ASSERT(hasDescriptor()); return *desc_; }
      const Descriptor& getDescriptor() const
      { NUKLEI_ASSERT(hasDescriptor()); return *desc_; }
      void setDescriptor(const Descriptor& desc)
      { desc_ = desc.clone(); }
      void clearDescriptor() { desc_.reset(); }
      
      void setFlag(bitfield_t flag)
      { bitfield_ |= flag; }
      void resetFlag(bitfield_t flag)
      { bitfield_ &= ~flag; }
      bool getFlag(bitfield_t flag) const
      { return (bitfield_ & flag) != 0; }

    protected:

      // The byte-size of this structure *must* be a multiple of 8.
      // 32-bit Count:
      //    vtable ptr: 4;
      //    w_:         8;
      //    flag1_:     4;
      //    bitfield_:  4;
      //    desc_:      4;
      //    TOTAL:      24 ok.
      // 64-bit Count:
      //    vtable ptr: 8;
      //    w_:         8;
      //    flag1_:     4;
      //    bitfield_:  4;
      //    desc_:      8;
      //    TOTAL:      32 ok.
      // When not aligned, the kernel class behaves 2 times slower.
      
      weight_t w_;
      
      int flag1_;
      bitfield_t bitfield_;
      
      std::auto_ptr<Descriptor> desc_;
      
      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & BOOST_SERIALIZATION_NVP(w_);
        ar & BOOST_SERIALIZATION_NVP(desc_);
      }
      
    };
    
    inline kernel::base* new_clone(const kernel::base& k)
    {
      base::ptr c = k.clone();
      return c.release();
    }
    
    template<class T>
    struct implementation_prototype : public base
    {
      base::ptr clone() const
      {
        NUKLEI_TRACE_BEGIN();
        return base::ptr(new T(*static_cast<const T*>(this)));
        NUKLEI_TRACE_END();
      }
      
      base::ptr create() const
      {
        NUKLEI_TRACE_BEGIN();
        return base::ptr(new T);
        NUKLEI_TRACE_END();
      }
      
      Vector3 getLoc() const
      {
        NUKLEI_TRACE_BEGIN();
        return static_cast<const T*>(this)->loc_;
        NUKLEI_TRACE_END();
      }
      
      void setLoc(const Vector3 &v)
      {
        NUKLEI_TRACE_BEGIN();
        static_cast<T*>(this)->loc_ = v;
        NUKLEI_TRACE_END();
      }
      
      std::ostream& polyPrint(std::ostream &out) const
      {
        NUKLEI_TRACE_BEGIN();
        return static_cast<const T*>(this)->print(out);
        NUKLEI_TRACE_END();
      }
      
      Type polyType() const
      {
        NUKLEI_TRACE_BEGIN();
        return static_cast<const T*>(this)->type();
        NUKLEI_TRACE_END();
      }
      
      coord_t polyEval(const base &k) const
      {
        NUKLEI_TRACE_BEGIN();
        const T& down = dynamic_cast<const T&>(k);
        return static_cast<const T*>(this)->eval(down);
        NUKLEI_TRACE_END();
      }
      
      coord_t polyCutPoint() const
      {
        NUKLEI_TRACE_BEGIN();
        return static_cast<const T*>(this)->cutPoint();
        NUKLEI_TRACE_END();
      }
      
      base::ptr polySample() const
      {
        NUKLEI_TRACE_BEGIN();
        base::ptr s(  new T(static_cast<const T*>(this)->sample())  );
        return s;
        NUKLEI_TRACE_END();
      }
      
      std::auto_ptr<kernel::se3> polySe3Sample() const;
      
      std::auto_ptr<kernel::se3> polySe3Proj() const;
      
      base::ptr polyProjectedOn(const kernel::se3& k) const
      {
        NUKLEI_TRACE_BEGIN();
        base::ptr s(  new T(static_cast<const T*>(this)->projectedOn(k))  );
        return s;
        NUKLEI_TRACE_END();
      }
      
      base::ptr polyTransformedWith(const kernel::se3& k) const
      {
        NUKLEI_TRACE_BEGIN();
        base::ptr s(  new T(static_cast<const T*>(this)->transformedWith(k))  );
        return s;
        NUKLEI_TRACE_END();
      }
      
      void polyMakeTransformWith(const kernel::se3& k)
      {
        NUKLEI_TRACE_BEGIN();
        static_cast<T*>(this)->makeTransformWith(k);
        NUKLEI_TRACE_END();
      }
      
      base::ptr polyLinearInterpolation(const kernel::base& k,
                                        const coord_t x) const
      {
        NUKLEI_TRACE_BEGIN();
        const T& down = dynamic_cast<const T&>(k);
        base::ptr s(  new T(static_cast<const T*>(this)->linearInterpolation(down, x))  );
        return s;
        NUKLEI_TRACE_END();
      }
      
      void polyUpdateWidth(const kernel::base& k,
                           const coord_t x = .5)
      {
        NUKLEI_TRACE_BEGIN();
        const T& down = dynamic_cast<const T&>(k);
        static_cast<T*>(this)->updateWidth(down, x);
        NUKLEI_TRACE_END();
      }
      
      coord_pair polyDistanceTo(const kernel::base& k) const
      {
        NUKLEI_TRACE_BEGIN();
        const T& down = dynamic_cast<const T&>(k);
        return static_cast<const T*>(this)->distanceTo(down);
        NUKLEI_TRACE_END();
      }
      
    protected:
      implementation_prototype() {};
    };
    
    /** @ingroup kernels */
    class se3 : public implementation_prototype<se3>
    {
    public:
      typedef std::auto_ptr<kernel::se3> ptr;
      
      typedef nuklei::unnormalized_shape_dist_kernel
      <groupS::r3, shapeS::triangle> PositionKernel;
      typedef von_mises_fisher_kernel<groupS::so3> OrientationKernel;
      
      se3() :
      loc_(Vector3::ZERO), ori_(Quaternion::IDENTITY),
      loc_h_(0), ori_h_(0)
      {}
      
      // Is this wise?
      explicit se3(const kernel::base& k)
      { *this = dynamic_cast<const kernel::se3&>(k); }
      
      void assertConsistency() const;
      
      std::ostream& print(std::ostream &out) const;
      
      coord_t getLocH() const { return loc_h_; }
      void setLocH(const coord_t h) { loc_h_ = h; }
      coord_t getOriH() const { return ori_h_; }
      void setOriH(const coord_t h) { ori_h_ = h; }
      
      Type type() const;
      
      /**
       * @brief Evaluates this kernel at pose @p k.
       *
       * This function returns
       * @f[
       * \mathcal K_{SE(3)}\left(\lambda, \theta ; \mu_t, \mu_r, \sigma_t, \sigma_r\right),
       * @f]
       * where the arguments of @f$ \mathcal K_{SE(3)} @f$ are respectively equal to @p
       * k.loc_, @p k.ori_, @p this->loc_, @p this->ori_, @p this->loc_h_, @p
       * this->ori_h_.
       * See @ref kernels for the definition of @f$ \mathcal K_{SE(3)} @f$.
       *
       * <b>Note:</b> @p this->ori_h_ should be expressed in radians.
       */
      inline coord_t eval(const kernel::se3& k) const;
      inline coord_t cutPoint() const;
      /**
       * @brief Returns a sample taken from this kernel.
       *
       * This function returns a sample @f$ (\lambda, \theta) @f$ drawn from
       * @f[
       * \mathcal K'_{SE(3)}\left(\lambda, \theta ; \mu_t, \mu_r, \sigma_t, \sigma_r\right),
       * @f]
       * where the four last arguments of @f$ \mathcal K'_{SE(3)} @f$ are respectively
       * equal to @p this->loc_, @p this->ori_, @p this->loc_h_, @p
       * this->ori_h_.
       * See @ref kernels for the definition of @f$ \mathcal K'_{SE(3)} @f$.
       *
       * <b>Note:</b> @p this->ori_h_ should be expressed in radians.
       */
      kernel::se3 sample() const;
      kernel::se3 se3Sample() const;
      kernel::se3 se3Proj() const;
      
      kernel::se3 projectedOn(const kernel::se3& k) const;
      kernel::se3 transformedWith(const kernel::se3& k) const;
      void makeTransformWith(const kernel::se3& k);
      kernel::se3 transformationFrom(const kernel::se3& k) const;
      kernel::se3 inverseTransformation() const;
      
      kernel::se3 linearInterpolation(const kernel::se3& k,
                                      const coord_t x) const;
      void updateWidth(const kernel::se3& k,
                       const coord_t x = .5);
      coord_pair distanceTo(const kernel::se3& k) const;
      
      /** @brief Kernel location. */
      Vector3 loc_;
      /** @brief Kernel orientation. */
      Quaternion ori_;
      /** @brief Location bandwidth. */
      coord_t loc_h_;
      /** @brief Orientation bandwidth, in radians. */
      coord_t ori_h_;
      
    private:
      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & boost::serialization::make_nvp
        ( "base",  
         boost::serialization::base_object<base>( *this ) );
        ar & BOOST_SERIALIZATION_NVP(loc_)
        & BOOST_SERIALIZATION_NVP(ori_)
        & BOOST_SERIALIZATION_NVP(loc_h_)
        & BOOST_SERIALIZATION_NVP(ori_h_);
      }
    };
    
    /** @ingroup kernels */
    template<class OriGrp>
    class r3xs2_base : public implementation_prototype< r3xs2_base<OriGrp> >
    {
    public:
      typedef std::auto_ptr< kernel::r3xs2_base<OriGrp> > ptr;
      
      typedef implementation_prototype< r3xs2_base<OriGrp> > Super;
      using Super::getWeight;
      using Super::setWeight;
      using Super::hasDescriptor;
      using Super::getDescriptor;
      using Super::setDescriptor;
      using Super::getFlag;
      
      typedef nuklei::unnormalized_shape_dist_kernel
      <groupS::r3, shapeS::triangle> PositionKernel;
      typedef von_mises_fisher_kernel<OriGrp> OrientationKernel;
      
      r3xs2_base() :
      loc_(Vector3::ZERO), dir_(Vector3::UNIT_X),
      loc_h_(0), dir_h_(0)
      {}
      
      // Is this wise?
      explicit r3xs2_base(const kernel::base& k)
      { *this = dynamic_cast<const kernel::r3xs2_base<OriGrp>&>(k); }
      
      void assertConsistency() const;
      
      std::ostream& print(std::ostream &out) const;
      
      coord_t getLocH() const { return loc_h_; }
      void setLocH(const coord_t h) { loc_h_ = h; }
      coord_t getOriH() const { return dir_h_; }
      void setOriH(const coord_t h) { dir_h_ = h; }
      
      base::Type type() const;
      
      /**
       * @brief Evaluates this kernel at pose @p k.
       *
       * <b>Note:</b> This doc describes the behavior of this method when @p
       * OriGrp is groupS::s2p, i.e., when the object on which the method is
       * called is of type kernel::r3xs2p.
       *
       * This function returns
       * @f[
       * \mathcal K_{RSA}\left(\lambda, \theta ; \mu_t, \mu_r, \sigma_t, \sigma_r\right),
       * @f]
       * where the arguments of @f$ \mathcal K_{RSA} @f$ are respectively equal to @p
       * k.loc_, @p k.dir_, @p this->loc_, @p this->dir_, @p this->loc_h_, @p
       * this->dir_h_.
       * See @ref kernels for the definition of @f$ \mathcal K_{RSA} @f$.
       *
       * <b>Note:</b> @p this->dir_h_ should be expressed in radians.
       */
      inline coord_t eval(const kernel::r3xs2_base<OriGrp>& k) const;
      inline coord_t cutPoint() const;
      /**
       * @brief Returns a sample taken from this kernel.
       *
       * <b>Note:</b> This doc describes the behavior of this method when @p
       * OriGrp is groupS::s2p, i.e., when the object on which the method is
       * called is of type kernel::r3xs2p.
       *
       * This function returns a sample @f$ (\lambda, \theta) @f$ drawn from
       * @f[
       * \mathcal K'_{RSA}\left(\lambda, \theta ; \mu_t, \mu_r, \sigma_t, \sigma_r\right),
       * @f]
       * where the four last arguments of @f$ \mathcal K'_{RSA} @f$ are respectively
       * equal to @p this->loc_, @p this->dir_, @p this->loc_h_, @p
       * this->dir_h_.
       * See @ref kernels for the definition of @f$ \mathcal K'_{RSA} @f$.
       *
       * <b>Note:</b> @p this->dir_h_ should be expressed in radians.
       */
      kernel::r3xs2_base<OriGrp> sample() const;
      kernel::se3 se3Sample() const;
      kernel::se3 se3Proj() const;
      
      kernel::r3xs2_base<OriGrp> projectedOn(const kernel::se3& k) const;
      kernel::r3xs2_base<OriGrp> transformedWith(const kernel::se3& k) const;
      void makeTransformWith(const kernel::se3& k);
      
      kernel::r3xs2_base<OriGrp> linearInterpolation(const kernel::r3xs2_base<OriGrp>& k,
                                                     const coord_t x) const;
      void updateWidth(const kernel::r3xs2_base<OriGrp>& k,
                       const coord_t x = .5);
      coord_pair distanceTo(const kernel::r3xs2_base<OriGrp>& k) const;
      
      Vector3 loc_;
      Vector3 dir_;
      coord_t loc_h_;
      coord_t dir_h_;
      
    private:
      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & boost::serialization::make_nvp
        ( "base",  
         boost::serialization::base_object<base>( *this ) );
        ar & BOOST_SERIALIZATION_NVP(loc_)
        & BOOST_SERIALIZATION_NVP(dir_)
        & BOOST_SERIALIZATION_NVP(loc_h_)
        & BOOST_SERIALIZATION_NVP(dir_h_);
      }
    };
    
    /**
     * @ingroup kernels
     * @brief @f$ \mathbb R^3 \times S^2 @f$
     */
    typedef r3xs2_base<groupS::s2> r3xs2;
    /**
     * @ingroup kernels
     * @brief @f$ \mathbb R^3 \times S^2_p @f$
     */
    typedef r3xs2_base<groupS::s2p> r3xs2p;
    
    /** @ingroup kernels */
    class r3 : public implementation_prototype<r3>
    {
    public:
      typedef std::auto_ptr<kernel::r3> ptr;
      
      typedef nuklei::unnormalized_shape_dist_kernel
      <groupS::r3, shapeS::triangle> PositionKernel;

      r3() :
      loc_(Vector3::ZERO),
      loc_h_(0)
      {}
      
      // Is this wise?
      explicit r3(const kernel::base& k)
      { *this = dynamic_cast<const kernel::r3&>(k); }
      
      void assertConsistency() const;
      
      std::ostream& print(std::ostream &out) const;
      
      coord_t getLocH() const { return loc_h_; }
      void setLocH(const coord_t h) { loc_h_ = h; }
      coord_t getOriH() const { return 0.; }
      void setOriH(const coord_t h) { }
      
      Type type() const;
      
      /**
       * @brief Evaluates this kernel at pose @p k.
       *
       * This function returns
       * @f[
       * \mathcal K_{\mathbb R^3}\left(\lambda ; \mu_t, \sigma_t\right),
       * @f]
       * where the arguments of @f$ \mathcal K_{\mathbb R^3} @f$ are respectively equal to @p
       * k.loc_, @p this->loc_, @p this->loc_h_.
       * See @ref kernels for the definition of @f$ \mathcal K_{\mathbb R^3} @f$.
       */
      inline coord_t eval(const kernel::r3& k) const;
      inline coord_t cutPoint() const;
      /**
       * @brief Returns a sample taken from this kernel.
       *
       * This function returns a sample @f$ \lambda @f$ drawn from
       * @f[
       * \mathcal K_{\mathbb R^3}\left(\lambda ; \mu_t, \sigma_t\right),
       * @f]
       * where the two last arguments of @f$ \mathcal K_{\mathbb R^3} @f$ are
       * respectively equal to @p this->loc_, @p this->loc_h_.
       * See @ref kernels for the definition of @f$ \mathcal K_{\mathbb R^3} @f$.
       */
      kernel::r3 sample() const;
      kernel::se3 se3Sample() const;
      kernel::se3 se3Proj() const;
      
      kernel::r3 projectedOn(const kernel::se3& k) const;
      kernel::r3 transformedWith(const kernel::se3& k) const;
      void makeTransformWith(const kernel::se3& k);
      
      kernel::r3 linearInterpolation(const kernel::r3& k,
                                     const coord_t x) const;
      void updateWidth(const kernel::r3& k,
                       const coord_t x = .5);
      coord_pair distanceTo(const kernel::r3& k) const;
      
      Vector3 loc_;
      coord_t loc_h_;
      
    private:
      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & boost::serialization::make_nvp
        ( "base",  
         boost::serialization::base_object<base>( *this ) );
        ar & BOOST_SERIALIZATION_NVP(loc_)
        & BOOST_SERIALIZATION_NVP(loc_h_);
      }
    };
    
    inline
    coord_t se3::eval(const kernel::se3& k) const
    {      
      coord_t r3e = PositionKernel::eval(loc_, loc_h_, k.loc_);
      if (r3e < FLOATTOL) return 0;
      else return r3e * OrientationKernel::eval(ori_, ori_h_, k.ori_);
    }
    
    template<class OriGrp>
    coord_t r3xs2_base<OriGrp>::eval(const kernel::r3xs2_base<OriGrp>& k) const
    {      
      coord_t r3e = PositionKernel::eval(loc_, loc_h_, k.loc_);
      if (r3e < FLOATTOL) return 0;
      else return r3e * OrientationKernel::eval(dir_, dir_h_, k.dir_);
    }
    
    inline
    coord_t r3::eval(const kernel::r3& k) const
    {      
      return PositionKernel::eval(loc_, loc_h_, k.loc_);
    }
    
    
    inline
    coord_t se3::cutPoint() const
    {
      return PositionKernel::cut_point(loc_h_);
    }
    
    template<class OriGrp>
    coord_t r3xs2_base<OriGrp>::cutPoint() const
    {
      return PositionKernel::cut_point(loc_h_);
    }
    
    inline
    coord_t r3::cutPoint() const
    {
      return PositionKernel::cut_point(loc_h_);
    }
    
    
    // S2P defs. The lib will also compile if these are in the .cpp file.
    // However, although execution will be fine under linux, it will fail
    // on the mac (dynamic_cast throws STL bad cast exception when casting
    // from base to r3xs2).
    
    template<class OriGrp>
    void r3xs2_base<OriGrp>::assertConsistency() const
    {
      NUKLEI_TRACE_BEGIN();
      NUKLEI_ASSERT(getWeight() >= 0);
      NUKLEI_ASSERT(loc_h_ >= 0);
      NUKLEI_ASSERT(dir_h_ >= 0);
      NUKLEI_TRACE_END();
    }
    
    template<> inline
    std::ostream&
    kernel::r3xs2_base<groupS::s2>::print(std::ostream &out) const
    {
      Indenter idt(out);
      idt << "Kernel R^3 x S^2: [ weight = " << w_ << " ]" << std::endl;
      {
        Indenter idt2(out);
        idt2 << "Location: " << loc_ << std::endl;
        idt2 << "LocWidth: " << loc_h_ << std::endl;
        idt2 << "Direction: " << dir_ << std::endl;
        idt2 << "DirWidth: " << dir_h_ << std::endl;
      }
      return out;
    }
    
    template<> inline
    std::ostream&
    kernel::r3xs2_base<groupS::s2p>::print(std::ostream &out) const
    {
      Indenter idt(out);
      idt << "Kernel R^3 x S^2_+: [ weight = " << w_ << " ]" << std::endl;
      {
        Indenter idt2(out);
        idt2 << "Location: " << loc_ << std::endl;
        idt2 << "LocWidth: " << loc_h_ << std::endl;
        idt2 << "Direction: " << dir_ << std::endl;
        idt2 << "DirWidth: " << dir_h_ << std::endl;
      }
      return out;
    }
    
    template<> inline
    kernel::base::Type kernel::r3xs2_base<groupS::s2>::type() const
    {
      return R3XS2;
    }
    
    template<> inline
    kernel::base::Type kernel::r3xs2_base<groupS::s2p>::type() const
    {
      return R3XS2P;
    }
    
    //template<class OriGrp>
    //coord_t r3xs2_base<OriGrp>::eval(const kernel::r3xs2_base<OriGrp>& k) const
    //cf. header.
    
    template<class OriGrp>
    kernel::r3xs2_base<OriGrp> r3xs2_base<OriGrp>::sample() const
    {
      kernel::r3xs2_base<OriGrp> s;

      typedef nuklei::sampler<PositionKernel> PositionSampler;
      typedef nuklei::sampler<OrientationKernel> OrientationSampler;
      
      s.loc_ = PositionSampler::s(loc_, loc_h_);
      s.dir_ = OrientationSampler::s(dir_, dir_h_);
      
      if (hasDescriptor()) s.setDescriptor(getDescriptor());
      return s;
    }
    
    template<class OriGrp>
    kernel::se3 r3xs2_base<OriGrp>::se3Sample() const
    {
      kernel::r3xs2_base<OriGrp> r3xs2pk = sample();
      kernel::se3 se3k;
      se3k.loc_ = r3xs2pk.loc_;
      se3k.ori_ = la::so3FromS2(r3xs2pk.dir_);
      if (hasDescriptor()) se3k.setDescriptor(getDescriptor());
      return se3k;
    }
    
    template<> inline
    kernel::se3
    kernel::r3xs2_base<groupS::s2>::se3Proj() const
    {
      kernel::r3xs2_base<groupS::s2> r3xs2pk = *this;
      kernel::se3 se3k;
      se3k.loc_ = r3xs2pk.loc_;
      se3k.ori_ = la::so3FromS2(r3xs2pk.dir_);
      if (hasDescriptor()) se3k.setDescriptor(getDescriptor());
      return se3k;
    }
    
    template<> inline
    kernel::se3
    kernel::r3xs2_base<groupS::s2p>::se3Proj() const
    {
      kernel::r3xs2_base<groupS::s2p> r3xs2pk = *this;
      kernel::se3 se3k;
      se3k.loc_ = r3xs2pk.loc_;
      if (Random::uniformInt(2) == 0)
        se3k.ori_ = la::so3FromS2(r3xs2pk.dir_);
      else
        se3k.ori_ = la::so3FromS2(-r3xs2pk.dir_);
      if (hasDescriptor()) se3k.setDescriptor(getDescriptor());
      return se3k;
    }
    
    template<class OriGrp>
    kernel::r3xs2_base<OriGrp> r3xs2_base<OriGrp>::projectedOn(const kernel::se3& k) const
    {
      kernel::r3xs2_base<OriGrp> p;
      la::project(p.loc_, p.dir_, k.loc_, k.ori_, loc_, dir_);
      if (hasDescriptor()) p.setDescriptor(getDescriptor());
      return p;
    }
    
    template<class OriGrp>
    kernel::r3xs2_base<OriGrp> r3xs2_base<OriGrp>::transformedWith(const kernel::se3& k) const
    {
      kernel::r3xs2_base<OriGrp> p;
      la::transform(p.loc_, p.dir_, k.loc_, k.ori_, loc_, dir_);
      if (hasDescriptor()) p.setDescriptor(getDescriptor());
      return p;
    }
    
    template<class OriGrp>
    void r3xs2_base<OriGrp>::makeTransformWith(const kernel::se3& k)
    {
      la::transform(loc_, dir_, k.loc_, k.ori_, loc_, dir_);
    }
    
    template<> inline
    kernel::r3xs2_base<groupS::s2>
    kernel::r3xs2_base<groupS::s2>::linearInterpolation
    (const kernel::r3xs2_base<groupS::s2>& k,
     const coord_t x) const
    {
      NUKLEI_ASSERT(0 <= x && x <= 1);
      kernel::r3xs2_base<groupS::s2> i;
      i.loc_ = (1-x) * loc_ + x * k.loc_;
      i.dir_ = la::normalized( (1-x) * dir_ + x * i.dir_ );
      return i;
    }
    
    template<> inline
    kernel::r3xs2_base<groupS::s2p>
    kernel::r3xs2_base<groupS::s2p>::linearInterpolation
    (const kernel::r3xs2_base<groupS::s2p>& k,
     const coord_t x) const
    {
      NUKLEI_ASSERT(0 <= x && x <= 1);
      kernel::r3xs2_base<groupS::s2p> i;
      i.loc_ = (1-x) * loc_ + x * k.loc_;
      Vector3 d = k.dir_;
      if (dir_.Dot(d) < 0) d = - d;
      i.dir_ = la::normalized( (1-x) * dir_ + x * d );
      return i;
    }
    
    template<class OriGrp>
    void
    kernel::r3xs2_base<OriGrp>::updateWidth
    (const kernel::r3xs2_base<OriGrp>& k,
     const coord_t x)
    {
      loc_h_ = std::sqrt( (1-x) * loc_h_*loc_h_ +
                         x * std::pow(dist<groupS::r3>::d(loc_, k.loc_), 2) );
      dir_h_ = std::sqrt( (1-x) * dir_h_*dir_h_ +
                         x * std::pow(dist<OriGrp>::d(dir_, k.dir_), 2) );
    }
    
    template<class OriGrp>
    coord_pair
    kernel::r3xs2_base<OriGrp>::distanceTo(const kernel::r3xs2_base<OriGrp>& k) const
    {
      return std::make_pair(dist<groupS::r3>::d(loc_, k.loc_),
                            dist<OriGrp>::d(dir_, k.dir_));
    }
    
    
    template<class T>
    std::auto_ptr<kernel::se3>
    kernel::implementation_prototype<T>::polySe3Sample() const
    {
      NUKLEI_TRACE_BEGIN();
      std::auto_ptr<se3> p(  new se3(static_cast<const T*>(this)->se3Sample())  );
      return p;
      NUKLEI_TRACE_END();
    }
    
    template<class T>
    std::auto_ptr<kernel::se3>
    kernel::implementation_prototype<T>::polySe3Proj() const
    {
      NUKLEI_TRACE_BEGIN();
      std::auto_ptr<se3> p(  new se3(static_cast<const T*>(this)->se3Proj())  );
      return p;
      NUKLEI_TRACE_END();
    }
    
    
  }
}

#if BOOST_VERSION < 104100

#else

BOOST_CLASS_EXPORT_KEY2(nuklei::kernel::se3, "mdfh_kernel_se3")
BOOST_CLASS_EXPORT_KEY2(nuklei::kernel::r3xs2, "mdfh_kernel_r3xs2")
BOOST_CLASS_EXPORT_KEY2(nuklei::kernel::r3xs2p, "mdfh_kernel_r3xs2p")
BOOST_CLASS_EXPORT_KEY2(nuklei::kernel::r3, "mdfh_kernel_r3")

#endif // BOOST_VERSION

#endif
