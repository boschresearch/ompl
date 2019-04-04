/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, Robert Bosch GmbH
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Leonard Bruns */

#ifndef OMPL_BASE_DETERMINISTIC_STATE_SAMPLER_
#define OMPL_BASE_DETERMINISTIC_STATE_SAMPLER_

#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {
        /** \brief An abstract class for the concept of using deterministic sampling sequences
        to decrease the dispersion of the samples. */
        class DeterministicStateSampler : public StateSampler
        {
        public:
            enum DeterministicSamplerType { HALTON, HAMMERSLEY};

            /** \brief Constructor */
            DeterministicStateSampler(const StateSpace *space, DeterministicSamplerType type=DeterministicSamplerType::HALTON) : StateSampler(space)
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;
        };

        /** \brief Deterministic state space sampler for SO(2) */
        class SO2DeterministicStateSampler : public DeterministicStateSampler
        {
        public:
            /** \brief Constructor */
            SO2DeterministicStateSampler(const StateSpace *space, DeterministicSamplerType type=DeterministicSamplerType::HALTON) : DeterministicStateSampler(space)
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;
        };

        /** \brief Deterministic state sampler for the R<sup>n</sup> state space */
        class RealVectorDeterministicStateSampler : public DeterministicStateSampler
        {
        public:
            /** \brief Constructor */
            RealVectorDeterministicStateSampler(const StateSpace *space, DeterministicSamplerType type=DeterministicSamplerType::HALTON) : DeterministicStateSampler(space)
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;
        };
    }
}

#endif
