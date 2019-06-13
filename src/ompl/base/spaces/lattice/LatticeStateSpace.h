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
*   * Neither the name of the Robert Bosch GmbH nor the names of its
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


#ifndef OMPL_BASE_SPACES_LATTICE_LATTICE_STATE_SPACE
#define OMPL_BASE_SPACES_LATTICE_LATTICE_STATE_SPACE

#include "ompl/base/spaces/WrapperStateSpace.h"
#include "ompl/geometric/PathGeometric.h"

#include <vector>

namespace ompl {
    namespace base {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::LatticeStateSpace */
        OMPL_CLASS_FORWARD(LatticeStateSpace);
        /// @endcond

        /** \class ompl::base::LatticeStateSpacePtr
            \brief A shared pointer wrapper for ompl::base::LatticeStateSpace */

        /** \brief A lattice state space that adds motion primitives to another 
         * state space. Planners that don't support planning with motion primitives 
         * will do normal sampling based planning on the underlying state space.
         */
        class LatticeStateSpace : public WrapperStateSpace 
        {
        public:
            using MotionPrimitive = geometric::PathGeometric;

            /** \brief Lattice state type. Contains a reference to the underlying state. And an id
             * of the motion primitive it belongs to. An id of -1 indicates that the state does not
             * belong to any motion primitive.
            */
            class StateType : public State
            {
            public:
                /** \brief Constructor. Takes a reference \a state to the underlying state. */
                StateType(State *state, int primitiveId) : state_(state), primitiveId_(primitiveId)
                {
                }

                /** \brief Get a const pointer to the underlying state. */
                const State *getState() const
                {
                    return state_;
                }

                /** \brief Get a pointer to the underlying state. */
                State *getState()
                {
                    return state_;
                }

            protected:
                /** \brief Underlying state. */
                State *state_;

                /** \brief Id of the motion primitive this state is part of. */
                int primitiveId_;
            };

            /** \brief Constructor. The underlying state space has to be defined. */
            LatticeStateSpace(const StateSpacePtr &space) : WrapperStateSpace(space)
            {
            }

            /** \brief Add a motion primitive to the state space. */
            void addMotionPrimitive(MotionPrimitive motionPrimitive);

            /** \brief Returns the outgoing motion primitive ids of a given state. */
            std::vector<size_t> getOutPrimitives(State *state) const;

            /** \brief Transforms a motion primitive to start at a given state. */
            MotionPrimitive transformPrimitive(State * startState, const MotionPrimitive& motionPrimitive) const;

            /** \brief Returns the end state given a motion primitive and a given start state. */
            State* getEndState(State* startState, const MotionPrimitive& motionPrimitive) const;

            /** \brief Sets the transform calculator function */
            void setTransformCalculator(const std::function<std::function<State*(State*)>(State*, const MotionPrimitive&)>& transformCalculator);

            /** \brief Sets the primitive validator function */
            void setPrimitiveValidator(const std::function<bool(State*, const MotionPrimitive&)>& primitiveValidator);

            /** \brief Sets a primitive validator that always returns true. For this to work the transformCalculator must be
             * able to transform all primitives to all possible start states.
             */
            void setDefaultPrimitiveValidator();

            void interpolate(const State* from, const State* to, double t, State* state) const override;
        protected:
            /** \brief Motion primitives this space includes for lattice planning. */
            std::vector<MotionPrimitive> motionPrimitives_;

            /** \brief Given a starting state and motion primitive, this function returns a function that transforms
             * a state on the primitive such that the starting state aligns with the starting state of the motion primitive.
             * Note that State* is a pointer to the underlying state type not to LatticeStateSpace type. */
            std::function<std::function<State*(State*)>(State*, const MotionPrimitive&)> transformCalculator_;

            /** \brief Given a starting state and a motion primitive, this function returns whether the motion primitive 
             * can be transformed to align with it. */
            std::function<bool(State*, const MotionPrimitive&)> primitiveValidator_;
        };
    }
}

#endif