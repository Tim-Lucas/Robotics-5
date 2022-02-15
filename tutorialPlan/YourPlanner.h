#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"
#include <random>

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
  
  virtual void seed(const ::std::mt19937::result_type& value);
  
  /**::rl::math::Real getProbability() const;
  
  void setProbability(const ::rl::math::Real& probability);**/
  
  ::rl::math::Real probability;

  bool solve();

protected:
  void choose(::rl::math::Vector& chosen, ::rl::math::Vector& goalBias);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
  
  ::std::uniform_real_distribution<::rl::math::Real>::result_type rand();
  
  ::std::uniform_real_distribution<::rl::math::Real> randDistribution;
  
  ::std::mt19937 randEngine;

private:

};

#endif // _YOUR_PLANNER_H_
