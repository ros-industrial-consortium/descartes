/*
 * common.h
 *
 *  Created on: Aug 30, 2019
 *      Author: Jorge Nicho
 */

#ifndef INCLUDE_DESCARTES_PLANNER_COMMON_H_
#define INCLUDE_DESCARTES_PLANNER_COMMON_H_

#include <descartes_core/trajectory_id.h>
#include <memory>

namespace descartes_planner
{
  template <typename FloatT = float>
  struct PointSampleGroup
  {
    virtual ~PointSampleGroup(){}

    std::size_t num_samples;      /** the number for samples */
    std::size_t num_dofs;         /** the number of dofs per sample */
    std::vector<FloatT> values;   /** the samples data where each sample contains `num_dofs` elements*/

    int point_id;
    typedef typename std::shared_ptr<PointSampleGroup> Ptr;
    typedef typename std::shared_ptr<const PointSampleGroup> ConstPtr;
  };

  /**
   * @class descartes_planner::PointSampler
   * @brief the base class for trajectory point samples,  actual implementations should
   * know the details of the robot such as ik solvers, joint limits, dofs, etcV
   */
  template <typename FloatT = float>
  class PointSampler
  {
  public:
    PointSampler(){}
    virtual ~PointSampler(){ }

    virtual std::size_t getNumSamples() = 0;
    virtual std::size_t getDofs() = 0;

    /**
     * @brief Computes the samples
     * @param g A pointer to the PointSampleGroup that is to be used for storing the sample data.  The values member
     *  can be preallocated before hand in order to improve the memory efficiency.  If any downcasting is necessary
     *  then the "std::dynamic_pointer_cast<DerivedType>(g)" can be used.
     * @return True on success, false otherwise
     */
    virtual bool getSamples(typename PointSampleGroup<FloatT>::Ptr g) = 0;

    /**
     * @brief Computes the samples
     * @return a pointer to a PointSampleGroup object
     */
    virtual typename PointSampleGroup<FloatT>::Ptr getSamples() = 0;

    /**
     * @brief computes a single sample at the requested index.  This is useful when only one sample is desired
     * @param idx The index for the sample that is to be computed
     * @return  A pointer to a PointSampleGroup with just one sample.
     */
    virtual typename PointSampleGroup<FloatT>::Ptr getSample(std::size_t idx) = 0;

    typedef typename std::shared_ptr<PointSampler<FloatT> > Ptr;
    typedef typename std::shared_ptr<const PointSampler<FloatT> > ConstPtr;
  };

  struct VertexProperties
  {
    virtual ~VertexProperties(){}
    int point_id;
    std::size_t sample_index;
  };

  template <typename FloatT = float>
  struct EdgeProperties
  {
    VertexProperties src_vtx;
    VertexProperties dst_vtx;
    bool valid;
    FloatT weight;
  };

  template <typename FloatT = float>
  class EdgeEvaluator
  {
  public:

    virtual ~EdgeEvaluator(){}

    /**
     * @brief evaluates all the edges between the samples of s1 and s2.
     * @param s1  A point sample group with n1 samples
     * @param s2  A point sample group with n2 samples
     * @return  Returns a n1 x n2 vector of EdgeProperties objects
     */
    virtual std::vector< EdgeProperties<FloatT> > evaluate(typename PointSampleGroup<FloatT>::ConstPtr s1,
                                                            typename PointSampleGroup<FloatT>::ConstPtr s2) const = 0;

    typedef typename std::shared_ptr<EdgeEvaluator> Ptr;
    typedef typename std::shared_ptr<const EdgeEvaluator> ConstPtr;
  };

}



#endif /* INCLUDE_DESCARTES_PLANNER_COMMON_H_ */
