// (C) Copyright Renaud Detry   2007-2015.
// Distributed under the GNU General Public License and under the
// BSD 3-Clause License (See accompanying file LICENSE.txt).

/** @file */


#ifndef NUKLEI_KERNELCOLLECTION_H
#define NUKLEI_KERNELCOLLECTION_H

#include <vector>
#include <string>
#include <iostream>
#include <list>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/none.hpp>
#include <boost/tuple/tuple.hpp>

#include <nuklei/decoration.h>
#include <nuklei/BoostSerialization.h>
#include <nuklei/Definitions.h>
#include <nuklei/Kernel.h>
#include <nuklei/nullable.h>
#include <nuklei/RegionOfInterest.h>
#include <nuklei/trsl/common.hpp>
#include <nuklei/trsl/ppfilter_iterator.hpp>
#include <nuklei/trsl/is_picked_systematic.hpp>
#include <nuklei/trsl/sort_iterator.hpp>


namespace nuklei {

  /**
   * @ingroup kernels
   * @brief This class acts as a vector-like container for kernels. It also
   * provides methods related to kernel density estimation.
   *
   * The KDE-related functions of this class are discussed in @ref kernels_kde.
   *
   * Note: In Nuklei, the kernel classes (kernel::base and its
   * descendants) play the double role of representing kernels and
   * points. For instance, there is no class specifically designed for
   * holding an @f$ SE(3) @f$ point, the class kernel::se3 is used for
   * that purpose. A KernelCollection is thus often used to contain a
   * set of points that are entirely unrelated to a density function.
   *
   * @section intermediary Intermediary Results
   *
   * Some of the methods of this class can benefit from caching intermediary
   * results. For instance, the #evaluationAt() method requires a @f$k@f$d-tree
   * of all kernel positions. @f$k@f$d-trees are expensive to construct, it is
   * important to avoid reconstructing them in each call of #evaluationAt().
   *
   * KernelCollection provides methods for precomputing intermediary results,
   * such as @f$k@f$d-trees. These structures are stored internally. For
   * instance,
   * @code
   * using namespace nuklei;
   * KernelCollection kc;
   * readObservations("file.txt", kc);
   *
   * kc.computeKernelStatistics(); // kernel statistics stored internally
   * kc.buildKdTree(); // position kd-tree stored internally
   *
   * kernel::se3 k;
   * ... // choose a value for k
   *
   * double e = kc.evaluationAt(k); // evaluationAt makes use of intermediary
   *                                // results
   * @endcode
   *
   * The functions responsible for computing intermediary results are:
   * - #computeKernelStatistics() 
   * - #buildKdTree()
   * - #buildNeighborSearchTree()
   * - #buildConvexHull()
   *
   * When a KernelCollection is modified, intermediary results become
   * invalid. To avoid inconsistencies, each call to a KernelCollection method
   * which can potentially allow one to modify the contained kernels (for
   * instance, #add()) automatically destroys all intermediary results. In order
   * to preserve intermediary results, one has to be careful to avoid calling
   * these methods. In particular, several methods, such as #front() and
   * #front()const, or #begin() and #begin()const, have a @p const and a @p
   * non-const version. The @p const methods will always preserve intermediary
   * results, while the @p non-const methods are likely to destroy them. One can
   * force a call to the @p const version with as_const():
   * @code
   * using namespace nuklei;
   * KernelCollection kc;
   * readObservations("file.txt", kc);
   *
   * kc.computeKernelStatistics();
   * kc.buildKdTree();
   *
   * kernel::se3 k;
   * ... // choose a value for k
   *
   * double e = kc.evaluationAt(k); // ok!
   *
   * for (KernelCollection::const_iterator i = as_const(kc).begin();
   *      i != as_const(kc).end(); ++i)
   * {
   *   i->setLocH(10);
   * }
   *
   * double e = kc.evaluationAt(k); // ok!
   *
   * // In the following line, even though i is a const_iterator, kc.begin()
   * // calls the non-const begin() method, which destroys intermediary results.
   * for (KernelCollection::const_iterator i = kc.begin();
   *      i != kc.end(); ++i)
   * {
   *   i->setLocH(10);
   * }
   *
   * double e = kc.evaluationAt(k); // throws exception: no kernel statistics,
   *                                // no kd-tree.
   * @endcode
   *
   * If the intermediary results that a method requires have not been computed,
   * the method throws an exception.
   *
   * @section iterators Sample Iterators, Sort Iterators
   *
   * KernelCollection provides iterators over a random permutation of its
   * elements (#sampleBegin()), and over a sorted permutation of its elements
   * (#sortBegin()). One will note that KernelCollection does not provide
   * sampleEnd() or sortEnd() methods. Ends are provided by the iterators
   * themselves, and should be accessed as follows:
   * @code
   * nuklei::KernelCollection kc = ...;
   * for (nuklei::KernelCollection::const_sample_iterator i = as_const(kc).sampleBegin(5);
   *      i != i.end(); ++i) // note i.end() instead of kc.end()
   * {
   *   // *i returns a reference to a datapoint/kernel of kc
   *   // i.index() returns the index (in kc) of that element.
   * }
   * for (nuklei::KernelCollection::const_sort_iterator i = as_const(kc).sortBegin(5);
   *      i != i.end(); ++i) // note i.end() instead of kc.end()
   * {
   *   // *i returns a reference to a datapoint/kernel of kc
   *   // i.index() returns the index (in kc) of that element.
   * }
   * @endcode
   *
   * These iterators are implemented with <a
   * href="http://trsl.sourceforge.net">the TRSL library</a>. Refer to the doc
   * of TRSL for more information.
   */
  class KernelCollection
    {
    public:
      void assertConsistency() const;

      // Forwarded container symbols

      /** @brief Kernel container type. */
      typedef boost::ptr_vector<kernel::base> Container;
      /** @brief KernelCollection iterator. */
      typedef Container::iterator iterator;
      /** @brief KernelCollection iterator. */
      typedef Container::const_iterator const_iterator;
      /** @brief KernelCollection iterator. */
      typedef Container::reverse_iterator reverse_iterator;
      /** @brief KernelCollection iterator. */
      typedef Container::const_reverse_iterator const_reverse_iterator;

      /** @brief Returns the kernel at index @p n. */
      Container::reference at(Container::size_type n)
        { invalidateHelperStructures(); return kernels_.at(n); }
      /** @brief Returns the kernel at index @p n. */
      Container::const_reference at(Container::size_type n) const
        { return kernels_.at(n); }

      /** @brief Returns the kernel at index @p 0. */
      Container::reference front()
        { invalidateHelperStructures(); return kernels_.front(); }
      /** @brief Returns the kernel at index @p 0. */
      Container::const_reference front() const
        { return kernels_.front(); }

      /** @brief Returns the kernel at index #size()-1. */
      Container::reference back()
        { invalidateHelperStructures(); return kernels_.back(); }
      /** @brief Returns the kernel at index #size()-1. */
      Container::const_reference back() const
        { return kernels_.back(); }

      /** @brief Returns the number of kernels. */
      Container::size_type size() const
        { return kernels_.size(); }

      /** @brief Returns true if empty. */
      bool empty() const
        { return kernels_.empty(); }

      /** @brief Returns an iterator pointing to the first kernel. */
      Container::iterator begin()
        { invalidateHelperStructures(); return kernels_.begin(); }
      /** @brief Returns an iterator pointing to the first kernel. */
      Container::const_iterator begin() const
        { return kernels_.begin(); }
      /** @brief Returns an iterator pointing to the last kernel. */
      Container::iterator end()
        { invalidateHelperStructures(); return kernels_.end(); }
      /** @brief Returns an iterator pointing to the last kernel. */
      Container::const_iterator end() const
        { return kernels_.end(); }

      /** @brief Returns an reverse iterator pointing to the last kernel. */
      Container::reverse_iterator rbegin()
        { invalidateHelperStructures(); return kernels_.rbegin(); }
      /** @brief Returns an reverse iterator pointing to the last kernel. */
      Container::const_reverse_iterator rbegin() const
        { return kernels_.rbegin(); }
      /** @brief Returns an reverse iterator pointing to the first kernel. */
      Container::reverse_iterator rend()
        { invalidateHelperStructures(); return kernels_.rend(); }
      /** @brief Returns an reverse iterator pointing to the first kernel. */
      Container::const_reverse_iterator rend() const
        { return kernels_.rend(); }

      // Container-related methods

      /** @brief Resets the class to its initial state. */
      void clear();
      /** @brief Adds a copy of @p f. */
      void add(const kernel::base &f);
      /** @brief Adds a copy of the kernels contained in @p kv. */
      void add(const KernelCollection &kv);
      /** @brief Replaces the @p idx'th kernel with a copy of @p k. */
      void replace(const size_t idx, const kernel::base &k);
      kernel::base::Type kernelType() const;

      // Iterators

      /** @brief Used internally. */
      typedef nuklei_trsl::is_picked_systematic<
        kernel::base, weight_t, kernel::base::WeightAccessor> is_picked;
      /**
       * @brief Sample Iterator type.
       *
       * See #sampleBegin().
       */
      typedef nuklei_trsl::ppfilter_iterator<
        is_picked, iterator> sample_iterator;
      /**
       * @brief Sample Iterator type.
       *
       * See #sampleBegin().
       */
      typedef nuklei_trsl::ppfilter_iterator<
        is_picked, const_iterator> const_sample_iterator;
      
      /**
       * @brief Returns an iterator that iterates through @p sampleSize kernels
       * selected randomly.
       *
       * Iterates through @p sampleSize kernels. The probability that a kernel
       * is selected is proportional to its weight: The probability that the @f$
       * i^{th} @f$ kernel returned by the iterator is the @f$ l^{th} @f$ kernel
       * of the KernelCollection is proportional to the weight of the @f$ l^{th}
       * @f$ kernel, as @f[ P(i = \ell) \propto w_{\ell}. @f]
       *
       * <b>This iterator does not return samples of the density! It returns
       * a random subset of the kernels. To get samples from the density, one
       * needs to get exactly one sample from each kernel returned by the
       * iterator.</b>
       *
       * See @ref iterators for more details.
       *
       * This method runs in @f$ O(n+\textrm{sampleSize}) @f$ time, where
       * @f$ n @f$ is the number of kernels in the collection.
       */
      sample_iterator sampleBegin(size_t sampleSize);
      /**
       * @brief Returns an iterator that iterates through @p sampleSize kernels.
       *
       * This is the @c const version of #sampleBegin().
       */
      const_sample_iterator sampleBegin(size_t sampleSize) const;

      /**
       * @brief Sort Iterator type.
       *
       * See #sortBegin().
       */
      typedef nuklei_trsl::reorder_iterator<iterator> sort_iterator;
      /**
       * @brief Sort Iterator type.
       *
       * See #sortBegin().
       */
      typedef nuklei_trsl::reorder_iterator<const_iterator> const_sort_iterator;
      
      /**
       * @brief Returns an iterator that iterates through the @p sortSize
       * kernels of highest weight, in order of decreasing weight.
       *
       * See @ref iterators for more details.
       */
      sort_iterator sortBegin(size_t sortSize);
      /**
       * @brief Returns an iterator that iterates through the @p sortSize
       * kernels of highest weight, in order of decreasing weight.
       *
       * This is the @c const version of #sortBegin().
       */
      const_sort_iterator sortBegin(size_t sortSize) const;
      
      // Particle-related methods

      /**
       * @brief Computes the sum of all kernel weights (total weight), and the
       * maximum kernel cut point.
       *
       * See @ref kernels_kde for an explanation of "cut point".
       */
      void computeKernelStatistics();
      /**
       * @brief Returns the sum of kernel weights.
       *
       * Precede by a call to #computeKernelStatistics(). See @ref intermediary.
       */
      weight_t totalWeight() const;
      coord_t maxLocCutPoint() const;
      /** @brief Divides all weights by the total weight. */
      void normalizeWeights();
      /**
       * @brief Sets all weights to @f$ 1 / t @f$, where @f$ t @f$ is
       * the total weight of the collection.
       */
      void uniformizeWeights();

      // Statistical moments
      
      /**
       * @brief Returns a kernel holding the mean position and orientation of
       * the data.
       */
      kernel::base::ptr mean() const;
      /**
       * @brief Returns a kernel holding the mean and standard deviation in
       * position and orientation of the data.
       *
       * Standard deviations for position and orientation are stored in the
       * kernel bandwidths.
       */
      kernel::base::ptr moments() const;
      
      // Geometrical properties
      
      /** @brief Transforms the data with @p t. */
      void transformWith(const kernel::se3& t);
      /** @brief Transforms the data with the provided translation and rotation. */
      void transformWith(const Vector3 &translation,
                         const Quaternion &rotation);

      /**
       * @brief Computes the local differential properties of the nearest
       * neighbors of @p k.
       *
       * This function requires a neighbor search tree. Its call must thus be
       * preceded by a call to #buildNeighborSearchTree(). See @ref
       * intermediary.
       *
       * This function uses the CGAL <a
       * href="http://www.cgal.org/Manual/3.3/doc_html/cgal_manual/Jet_fitting_3/Chapter_main.html">Monge
       * fit</a> functions.
       */
      boost::tuple<Matrix3, Vector3, coord_t>
      localLocationDifferential(const Vector3& k) const;
      
      /**
       * @brief Fits a plane to the positions of the kernels contained in @p
       * *this.
       *
       * The location of the returned kernel is a point of the plane. The
       * orientation of the returned kernel is such that its @f$ z @f$ axis is
       * normal to the plane.
       */
      kernel::se3 linearLeastSquarePlaneFit() const;
      /**
       * @brief Fits a plane to the positions of the kernels contained in @p
       * *this.
       *
       * The location of the returned kernel is a point of the plane. The
       * orientation of the returned kernel is such that its @f$ z @f$ axis is
       * normal to the plane.
       */
      kernel::se3
      ransacPlaneFit(coord_t inlinerThreshold, unsigned nSeeds = 100) const;

      /**
       * @brief Returns the locations of the contained kernels in an
       * std::vector.
       */
      std::vector<Vector3> get3DPointCloud() const;

      /**
       * @brief Computes surface normals at all points. After running this
       * method, all kernels are nuklei::kernel::r3xs2p.
       *
       * The orientations/directions that may be associated to the kernels
       * prior to calling this method are ignored and replaced with the normals
       * computed from local neighbors.
       *
       * This function requires a neighbor search tree. Its call must thus be
       * preceded by a call to #buildNeighborSearchTree(). See @ref
       * intermediary.
       */
      void computeSurfaceNormals();

      /**
       * @brief Builds a kd-tree of the kernel positions and stores the tree
       * internally. See @ref intermediary.
       */
      void buildKdTree();
      /**
       * @brief Builds a neighbor search tree of the kernel positions and stores
       * the tree internally. See @ref intermediary.
       */
      void buildNeighborSearchTree();
      /**
       * @brief Builds the convex hull of kernel positions and stores the hull
       * internally. See @ref intermediary.
       */
      void buildConvexHull(unsigned n);
      /**
       * @brief Check if @p k is within the convex hull of contained kernel
       * positions.
       *
       * Precede by a call to #buildConvexHull(). See @ref intermediary.
       */
      bool isWithinConvexHull(const kernel::base& k) const;
      /**
       * @brief Builds a mesh from kernel positions. See @ref intermediary.
       */
      void buildMesh();
      void writeMeshToOffFile(const std::string& filename) const;
      void readMeshFromOffFile(const std::string& filename);
      void writeMeshToPlyFile(const std::string& filename) const;
      void readMeshFromPlyFile(const std::string& filename);
      /**
       * @brief Builds set of partial views of the object. See @ref intermediary.
       */
      void buildPartialViewCache(const double meshTol, const bool useRayToSurfacenormalAngle = false);
      /**
       * @brief Assuming that the points in this collection form the surface of
       * an object, this function computes whether a point @p p is visible from
       * @p viewpoint, or if it is occluded by the object.
       *
       * This function requires prior computation of a surface mesh from the
       * points of the collection. See buildMesh().
       *
       * This function computes whether a segment linking @p viewpoint to @p p
       * intersects with the mesh.
       *
       * If @p tolerance is greater than 0, the function computes whether a
       * segment linking @p viewpoint to \f[ viewpoint + (p - viewpoint)
       * \frac{|p-viewpoint|-tolerance}{|p-viewpoint|} \f] intersects with the mesh.
       */
      bool isVisibleFrom(const Vector3& p, const Vector3& viewpoint,
                         const coord_t& tolerance = FLOATTOL) const;
      /**
       * @brief Same as isVisibleFrom(), but additionally checks that the 
       * ray-to-surfacenormal angle is small enough.
       */
      bool isVisibleFrom(const kernel::r3xs2p& p, const Vector3& viewpoint,
                         const coord_t& tolerance = FLOATTOL) const;
      
      /**
       * @brief Assuming that the points in this collection form the surface of
       * an object, this function returns the indices of points visible from
       * @p viewpoint.
       *
       * See isVisibleFrom() for more details.
       */
      std::vector<int> partialView(const Vector3& viewpoint,
                                   const coord_t& tolerance = FLOATTOL,
                                   const bool useViewcache = false,
                                   const bool useRayToSurfacenormalAngle = false) const;
      /**
       * @brief Partial View Iterator type.
       *
       * See #partialViewBegin().
       */
      typedef nuklei_trsl::reorder_iterator<const_iterator> const_partialview_iterator;

      /**
       * @brief Assuming that the points in this collection form the surface of
       * an object, this function returns an iterator that iterates through the
       * kernels that are visible from @p viewpoint.
       *
       * See isVisibleFrom() for more details.
       *
       * See @ref iterators for more details.
       */
      const_partialview_iterator
      partialViewBegin(const Vector3& viewpoint,
                       const coord_t& tolerance = FLOATTOL,
                       const bool useViewcache = false,
                       const bool useRayToSurfacenormalAngle = false) const;
      
      // Density-related methods
            
      /**
       * @brief Returns @p sampleSize samples from the density modeled by *this.
       *
       * See @ref kernels_kde for a description of this method.
       *
       * Precede by a call to #computeKernelStatistics(). See
       * @ref intermediary.
       */
      KernelCollection sample(int sampleSize) const;
      /**
       * @brief Deprecated. Use #sample() instead.
       */
      void resetWithSampleOf(const KernelCollection &kc,
                             int sampleSize);
            
      /**
       * @brief Returns a kernel from the collection.
       *
       * The probability of returning the @f$ i^{\rm th} @f$ kernel is
       * proportional to the weight of that kernel. This methods is @f$ O(n)
       * @f$, where @f$ n @f$ is the number of kernels contained in the
       * collection. To efficiently select multiple kernels randomly, use
       * #sampleBegin().
       */
      const kernel::base& randomKernel() const;
      
      /** @brief Sets the location bandwidth of all kernels. */
      void setKernelLocH(coord_t h);
      /**
       * @brief Sets the orientation bandwidth of all kernels.
       *
       * This method calls kernel::base::setOriH on all kernels. If the kernels
       * do not have an orientation, this method does nothing.
       */
      void setKernelOriH(coord_t h);
      
      typedef enum { SUM_EVAL, MAX_EVAL, WEIGHTED_SUM_EVAL } EvaluationStrategy;
      /**
       * @brief Evaluates the density represented by @p *this at @p f.
       *
       * See @ref kernels_kde for a description of this method.
       *
       * Precede by a call to #computeKernelStatistics() and #buildKdTree(). See
       * @ref intermediary.
       */
      weight_t evaluationAt(const kernel::base &f,
                            const EvaluationStrategy strategy = WEIGHTED_SUM_EVAL) const;
      
      
      // Misc
      
      void clearDescriptors();

      void setFlag(const bitfield_t flag);

    private:
      Container kernels_;
      
      boost::optional<weight_t> totalWeight_;
      boost::optional<coord_t> maxLocCutPoint_;
      boost::optional<kernel::base::Type> kernelType_;
      
      decoration<int> deco_;
      const static int HULL_KEY;
      const static int KDTREE_KEY;
      const static int NSTREE_KEY;
      const static int MESH_KEY;
      const static int AABBTREE_KEY;
      const static int VIEWCACHE_KEY;

      void invalidateHelperStructures();

      template<class KernelType>
      weight_t staticEvaluationAt(const kernel::base &k,
                                  const EvaluationStrategy strategy) const;
      
      kernel::base::ptr deviation(const kernel::base &center) const;
      template<typename C>
      C partialView(const Vector3& viewpoint,
                    const coord_t& tolerance,
                    const bool useViewcache,
                    const bool useRayToSurfacenormalAngle) const;
      
      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
          ar  & BOOST_SERIALIZATION_NVP(kernels_);
          ar  & BOOST_SERIALIZATION_NVP(kernelType_);
        }

    };

}

#endif
