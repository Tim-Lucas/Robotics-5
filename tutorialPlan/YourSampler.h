#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_

#include <rl/plan/Sampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Uniform random sampling strategy.
         */
        class YourSampler : public Sampler
        {
        public:
            YourSampler();

            virtual ~YourSampler();

            ::rl::math::Vector generate();
            
            ::rl::math::Vector generate(int index);
            
            ::rl::math::Vector generateGaussian();
            
            ::rl::math::Vector generateBridge();

            virtual void seed(const ::std::mt19937::result_type &value);
            
            void setGoalBias(rl::math::Vector *goal_a, rl::math::Vector *goal_b, rl::math::Real prob);
            
            rl::math::Vector *getGoal(int index);

            ::rl::math::Real ratio;  // Probability of choosing bridge sample.

        protected:
            ::std::uniform_real_distribution<::rl::math::Real>::result_type rand();

            ::std::uniform_real_distribution<::rl::math::Real> randDistribution;

            ::std::mt19937 randEngine;

            ::std::uniform_real_distribution<::rl::math::Real>::result_type gauss();

            ::std::uniform_real_distribution<::rl::math::Real> gaussDistribution;

            ::std::mt19937 gaussEngine;

        private:
            rl::math::Vector *goal_a; // goal configuration for tree a
            
            rl::math::Vector *goal_b; // goal configuration for tree b
            
            rl::math::Real goal_prob; // probability of drawing goal configuration
        };
    }
}

#endif // _YOURSAMPLER_H_
