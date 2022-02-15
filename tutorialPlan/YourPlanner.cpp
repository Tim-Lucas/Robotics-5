#include "YourPlanner.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <iostream>

YourPlanner::YourPlanner() :
  RrtConConBase(),
  probability(static_cast<::rl::math::Real>(0.05)),
  randDistribution(0, 1),
  randEngine(::std::random_device()())
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen, ::rl::math::Vector& goalBias)
{
  this->model->getDof();
  //your modifications here
  
  // RRT Goal Bias
  //if (this->rand() > this->probability) {
  
    // Sample a random configuration with probability p
    RrtConConBase::choose(chosen);

  /**} else {
    chosen = goalBias;
  }**/
  
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::connect(tree, nearest, chosen);
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
  //your modifications here
  //return RrtConConBase::solve();
  
  this->time = ::std::chrono::steady_clock::now();
  
  // Define the roots of both trees
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

  Tree* a = &this->tree[0];
  Tree* b = &this->tree[1];

  ::rl::math::Vector chosen(this->model->getDof());
  ::rl::math::Vector goalBias(this->model->getDof());


  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    //First grow tree a and then try to connect b.
    //then swap roles: first grow tree b and connect to a.
    for (::std::size_t j = 0; j < 2; ++j)
    {
      // get goal for goalBias
      goalBias = &this->tree[0] == a ? *this->goal : *this->start;

      Neighbor aNearest;
      do
      {
	this->choose(chosen, goalBias);
	aNearest = this->nearest(*a, chosen);
      }
      while (aNearest.second > (*a)[aNearest.first].R);

      //Do a CONNECT step from the nearest neighbour to the sample
      Vertex aConnected = this->connect(*a, aNearest, chosen);

      //If a new node was inserted tree a
      if (NULL != aConnected)
      {
      	// if expansion was successful, increase radius for nearest neighbor
        if ((*a)[aNearest.first].R < ::std::numeric_limits<::rl::math::Real>::max())
	{
	  (*a)[aNearest.first].R *= 1.05;
	}
	
        // Try a CONNECT step form the other tree to the sample
        Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
        Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

        if (NULL != bConnected)
        {
          //Test if we could connect both trees with each other
          if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
          {
            this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
            this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
            return true;
          }
        }
      } else {
      	// if expansion failed, decrease radius of nearest neighbor node
      	if ((*a)[aNearest.first].R < ::std::numeric_limits<::rl::math::Real>::max())
	{
	  (*a)[aNearest.first].R *= (1 - 0.05);
	  (*a)[aNearest.first].R = ::std::max(2.0, (*a)[aNearest.first].R);
	}
	else
	{
	  (*a)[aNearest.first].R = 20.0;
	}
      }

      //Swap the roles of a and b
      using ::std::swap;
      swap(a, b);
    }

  }

  return false;
}

/**::rl::math::Real
YourPlanner::getProbability() const
{
    return this->probability;
}

void
YourPlanner::setProbability(const ::rl::math::Real& probability)
{
    this->probability = probability;
}**/

::std::uniform_real_distribution<::rl::math::Real>::result_type
YourPlanner::rand()
{
	return this->randDistribution(this->randEngine);
}

void
YourPlanner::seed(const ::std::mt19937::result_type& value)
{
	this->randEngine.seed(value);
}

