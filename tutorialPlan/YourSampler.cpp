#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"
#include <iostream>

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            Sampler(),
            randDistribution(0, 1),
            randEngine(::std::random_device()())
        {

        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            //return generateBridge();
            // return generateGaussian();
            ::rl::math::Vector rand(this->model->getDof());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                rand(i) = this->rand();
            }

            return this->model->generatePositionUniform(rand);
        }

        ::rl::math::Vector
        YourSampler::generate(int index)
        {
            ::rl::math::Vector rand(this->model->getDof());

            ::rl::math::Real r = this->rand();

            // Return goal configuration for the current tree with probability this->goal_prob
            if (r <= this->goal_prob)
            {
                return *this->getGoal(index);
            }

            // Otherwise return a randomly generated configuration
            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                rand(i) = this->rand();
            }

            return this->model->generatePositionUniform(rand);
        }

        ::rl::math::Vector
        YourSampler::generateGaussian(int index) {
          ::rl::math::Vector rand(this->model->getDof());

          ::rl::math::Real r = this->rand();

          // Return goal configuration for the current tree with probability this->goal_prob
          if (r <= this->goal_prob)
          {
              return *this->getGoal(index);
          }

          ::rl::math::Vector sigma(this->model->getDof());

          for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
            sigma(i) = 5.;
          }

          while (true) {
            // generate first sample
            ::rl::math::Vector rand(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
              rand(i) = this->rand();
            }
            ::rl::math::Vector q1 = this->model->generatePositionUniform(rand);

            // generate second sample (gaussian around the first one)
            for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
              rand(i) = this->rand();
            }
            ::rl::math::Vector q2 = this->model->generatePositionGaussian(rand, q1, sigma);

            this->model->setPosition(q1);
            this->model->updateFrames();

            if(!this->model->isColliding()){
              this->model->setPosition(q2);
              this->model->updateFrames();
              if(this->model->isColliding()) {
                return q1;
              }
            } else{
              this->model->setPosition(q2);
              this->model->updateFrames();
              if(!this->model->isColliding()) {
                return q2;
              }
            }
          }

        }
        
        void YourSampler::setGoalBias(rl::math::Vector *goal_a, rl::math::Vector *goal_b, rl::math::Real prob)
        {
            // Setting the goal configuration of the two trees a and b, as well as the probability of drawing it
            this->goal_a = goal_a;
            this->goal_b = goal_b;
            this->goal_prob = prob;
        }


        rl::math::Vector *
        YourSampler::getGoal(int index)
        {
            // Return the goal configuration for a given tree
            if (index == 0)
                return this->goal_a;
            return this->goal_b;
        }


        ::std::uniform_real_distribution< ::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->randEngine.seed(value);
        }
    }
}