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

#include "ompl/base/spaces/lattice/LatticeStateSpace.h"

namespace ompl 
{
    namespace base 
    {
        LatticeStateSpace::LatticeStateSpace(const StateSpacePtr &space) : WrapperStateSpace(space)
        {
        }

        State *LatticeStateSpace::allocState() const 
        {
            return new StateType(space_->allocState(), -1);
        }

        void LatticeStateSpace::addMotionPrimitive(const MotionPrimitive& motionPrimitive)
        {
            motionPrimitives_.push_back(motionPrimitive);
        }

        void LatticeStateSpace::clearMotionPrimitives()
        {
            motionPrimitives_.clear();
        }

        std::vector<size_t> LatticeStateSpace::getOutPrimitives(const State *state) const {
            std::vector<size_t> out_ids;
            
            size_t current_id = 0;
            for(const auto& motionPrimitive : motionPrimitives_)
            {
                if(primitiveValidator_(state, motionPrimitive))
                {
                    out_ids.push_back(current_id);
                }
                ++current_id;
            }

            return out_ids;
        }

        LatticeStateSpace::MotionPrimitive LatticeStateSpace::transformPrimitive(const State * startState, const MotionPrimitive& motionPrimitive) const
        {
            MotionPrimitive transformedPrimitive(motionPrimitive);

            auto transform = transformCalculator_(startState, motionPrimitive);

            for(auto& state : transformedPrimitive.getStates())
            {
                transform(state);
            }

            return motionPrimitive;
        }

        State* LatticeStateSpace::getEndState(const State* startState, size_t primitiveId) const
        {
            auto transform = transformCalculator_(startState, motionPrimitives_[primitiveId]);

            // for the motion primitives and the states passed around internally, 
            // we simply use the underlying state type
            State * endState = allocState();

            transform(endState);

            return endState;
        }

        void LatticeStateSpace::setTransformCalculator(const std::function<std::function<void(State*)>(const State*, const MotionPrimitive&)>& transformCalculator) 
        {
            transformCalculator_ = transformCalculator;
        }

        void LatticeStateSpace::setPrimitiveValidator(const std::function<bool(const State*, const MotionPrimitive&)>& primitiveValidator)
        {
            primitiveValidator_ = primitiveValidator;
        }

        void LatticeStateSpace::setDefaultPrimitiveValidator()
        {
            setPrimitiveValidator(
                [] (const State *, const MotionPrimitive&) -> bool
                {
                    return true;
                }
            ); 
        }

        void LatticeStateSpace::setDefaultPrimitiveInterpolator()
        {
            setPrimitiveInterpolator(
                [] (const State* from, const State* to, double t, const MotionPrimitive& primitive, State* state)
                {
                    primitive.getSpaceInformation()->getStateSpace()->interpolate(from, to, t, state);
                }
            ); 
        }

        void LatticeStateSpace::setPrimitiveInterpolator(const std::function<void(const State*, const State*, double, const MotionPrimitive&, State*)>& primitiveInterpolator)
        {
            primitiveInterpolator_ = primitiveInterpolator;
        }

        bool LatticeStateSpace::hasSymmetricInterpolate() const 
        {
            // since the interpolate depends on the motion primtiive this is generally not true
            return false;
        }

        void LatticeStateSpace::interpolate(const State* from, const State* to, double t, State* state) const
        {
            const auto *rfrom = static_cast<const StateType *>(from);
            const auto *rto = static_cast<const StateType *>(to);
            StateType *rstate = static_cast<StateType *>(state);
            int motionPrimitiveIdFrom = rfrom->getPrimitiveId();
            if(motionPrimitiveIdFrom == -1) // no motion primitive -> interpolate using the underlying space
            {
                space_->interpolate(rfrom->getState(), rto->getState(), t, rstate->getState());
            }
            else // interpolate on a motion primitive
            {
                primitiveInterpolator_(rfrom->getState(), rto->getState(), t, motionPrimitives_[motionPrimitiveIdFrom], rstate->getState());
            }
        }

        void LatticeStateSpace::setInitialState(State *state)
        {
            freeState(state);
            initialState_ = state;
        }

        const State *LatticeStateSpace::getInitialState() const
        {
            return initialState_;
        }


        void LatticeStateSpace::setDefaultCostHeuristic()
        {
            setCostHeuristic(
                [this] (State *from, State *to)->base::Cost {
                    return base::Cost(distance(from,to));
                }
            );
        }

        void LatticeStateSpace::setCostHeuristic(const std::function<base::Cost(State *, State *)>& costHeuristic)
        {
            costHeuristic_ = costHeuristic;
        }


        base::Cost LatticeStateSpace::getMotionPrimitiveCost(size_t primitive) const
        {
            return base::Cost(motionPrimitives_[primitive].length());
        }

        base::Cost LatticeStateSpace::costHeuristic(State *from, State *to) const
        {
            return costHeuristic_(from, to);
        }
    }
}