#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "RrtConConBase.h"
#include "YourSampler.h"
#include <rl/plan/KdtreeNearestNeighbors.h>

using namespace ::rl::plan;

/**
 *	The implementation of your planner.
 *	modify any of the existing methods to improve planning performance.
 */
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  virtual void seed(const ::std::mt19937::result_type &value);

  /**::rl::math::Real getProbability() const;

  void setProbability(const ::rl::math::Real& probability);**/

  ::rl::math::Real probability;

  bool solve();

  YourSampler *sampler;

  rl::plan::KdtreeNearestNeighbors *nearestNeighbors0;
  rl::plan::KdtreeNearestNeighbors *nearestNeighbors1;

protected:
  void choose(::rl::math::Vector &chosen, int index);

private:
  Neighbor nearest(const Tree &tree, const rl::math::Vector &chosen, int index);

  Vertex connect(Tree &tree, const Neighbor &nearest, const rl::math::Vector &chosen, int index);

  Vertex addVertex(Tree &tree, const VectorPtr &q, int index);

  Vertex addVertex(Tree &tree, const VectorPtr &q);
};

#endif // _YOUR_PLANNER_H_
