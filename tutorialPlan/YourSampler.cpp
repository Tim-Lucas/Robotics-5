#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() : Sampler(),
                                     randDistribution(0, 1),
                                     randEngine(::std::random_device()()),
                                     gaussDistribution(0, 1),
			                               gaussEngine(::std::random_device()()),
                                     ratio(static_cast<::rl::math::Real>(5) / static_cast<::rl::math::Real>(6))
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generate(int index)
        {
            ::rl::math::Real r = this->rand();

            // Return goal configuration for the current tree with probability this->goal_prob
            if (r <= this->goal_prob)
            {
                return *this->getGoal(index);
            }

            //Otherwise return a gaussian generated configuration
          
            return this->generateGaussian(); // Gaussian sampling
            // return this->generateBridge(); // Bridge sampling
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            ::rl::math::Vector rand(this->model->getDof());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                rand(i) = this->rand();
            }

            return this->model->generatePositionUniform(rand);
        }

        ::std::uniform_real_distribution<::rl::math::Real>::result_type
        YourSampler::rand()
        {
            return this->randDistribution(this->randEngine);
        }

        ::std::normal_distribution<::rl::math::Real>::result_type
        YourSampler::gauss()
        {
          return this->gaussDistribution(this->gaussEngine);
        }

        void
        YourSampler::seed(const ::std::mt19937::result_type &value)
        {
            this->randEngine.seed(value);
            this->gaussEngine.seed(value);
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

        // Gaussian sampling
        ::rl::math::Vector
        YourSampler::generateGaussian() {

          ::rl::math::Vector sigma(this->model->getDof());
          for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
            sigma(i) = 1.;
          }

          while (true) {
            // First uniform generated sample
            ::rl::math::Vector q1 = this->generate();

            // second sample gaussian generated around the first sample
            ::rl::math::Vector gauss(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
              gauss(i) = this->gauss();
            }
            ::rl::math::Vector q2 = this->model->generatePositionGaussian(gauss, q1, sigma);

            this->model->setPosition(q1);
            this->model->updateFrames();
            // Check if first sample is not colliding
            if(!this->model->isColliding()){
              this->model->setPosition(q2);
              this->model->updateFrames();
              // If second sample colliding -> first sample is near an obstacle
              if(this->model->isColliding()) {
                return q1;
              }
            } else{ // If first sample is colliding
              this->model->setPosition(q2);
              this->model->updateFrames();
              // If seecond sample is not colliding -> second sample is near an obstacle
              if(!this->model->isColliding()) {
                return q2;
              }
            }
          }
        }

        // Bridge sampling
        ::rl::math::Vector
        YourSampler::generateBridge()
        {
          // With rand. probability sample uniform and return
          if (this->rand() > this->ratio)
          {
            return this->generate();
          }
          ::rl::math::Vector q(this->model->getDof());
          ::rl::math::Vector gauss(this->model->getDof());
          
          ::rl::math::Vector sigma(this->model->getDof());
          for (::std::size_t i = 0; i < this->model->getDof(); ++i) {
            sigma(i) = 0.6;
          }

          while (true)
          {
            // Generate first sample uniform
            ::rl::math::Vector q2 = this->generate();
            
            this->model->setPosition(q2);
            this->model->updateFrames();
            // Check if the sample is colliding
            if (this->model->isColliding())
            {
              for (::std::size_t i = 0; i < this->model->getDof(); ++i)
              {
                gauss(i) = this->gauss();
              }
              // Generate seecond sample gaussian around first sample
              ::rl::math::Vector q3 = this->model->generatePositionGaussian(gauss, q2, sigma);
              
              this->model->setPosition(q3);
              this->model->updateFrames();
              // Check if second sample is also colliding
              if (this->model->isColliding())
              {
                // If colliding -> build a sample between the first and the second sample
                this->model->interpolate(q2, q3, static_cast<::rl::math::Real>(0.5), q);
                
                this->model->setPosition(q);
                this->model->updateFrames();
                // If the new sample is not coliding (is between two obstacles), return  it
                if (!this->model->isColliding())
                {
                  return q;
                }
              }
            }
          }
          
        }
    }
}
