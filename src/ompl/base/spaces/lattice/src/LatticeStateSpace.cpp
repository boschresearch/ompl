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
            // OMPL_INFORM("allocState: Allocating a state...");
            return new StateType(space_->allocState(), -1);
        }

        void LatticeStateSpace::copyState(State *destination, const State *source) const
        {
            // OMPL_INFORM("copyState: Attempting to copy state...");
            // if(space_ == nullptr) {
            //     OMPL_INFORM("space_ is nullptr");
            //     return;
            // }
            // if(destination->as<StateType>()->getState() == nullptr) {
            //     OMPL_INFORM("destination is nullptr");
            //     return;
            // }
            // if(source->as<StateType>()->getState() == nullptr) {
            //     OMPL_INFORM("source is nullptr");
            //     return;
            // }

            space_->copyState(destination->as<StateType>()->getState(), source->as<StateType>()->getState());
            destination->as<StateType>()->setPrimitiveId(source->as<StateType>()->getPrimitiveId());
            // OMPL_INFORM("copyState: Copy succesful");
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

            return transformedPrimitive;
        }

        State* LatticeStateSpace::getEndState(const State* startState, size_t primitiveId) const
        {
            auto transform = transformCalculator_(startState, motionPrimitives_[primitiveId]);

            size_t lastStateIndex = motionPrimitives_[primitiveId].getStateCount() - 1;
            State * endState = cloneState(motionPrimitives_[primitiveId].getState(lastStateIndex));

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
            // since the interpolate depends on the motion primitive this is generally not true
            return false;
        }

        void LatticeStateSpace::interpolate(const State* from, const State* to, double t, State* state) const
        {
            const auto *rfrom = static_cast<const StateType *>(from);
            const auto *rto = static_cast<const StateType *>(to);
            StateType *rstate = static_cast<StateType *>(state);
            int motionPrimitiveIdFrom = rfrom->getPrimitiveId();
            // TODO, remove the true, update distance function, distance->interpolate->distance->interpolate->distance should always yield (approx.) same distance, right now not the case!
            if(true || motionPrimitiveIdFrom == -1) // no motion primitive -> interpolate using the underlying space
            {
                space_->interpolate(rfrom->getState(), rto->getState(), t, rstate->getState());
            }
            else // interpolate on a motion primitive
            {
                primitiveInterpolator_(rfrom, rto, t, motionPrimitives_[motionPrimitiveIdFrom], rstate);
            }
        }

        void LatticeStateSpace::printState(const State *state, std::ostream &out) const
        {
            out << "Lattice state [" << std::endl;
            space_->printState(state->as<StateType>()->getState(), out);
            out << "Primitive id [" << state->as<StateType>()->getPrimitiveId() << "]" << std::endl;
            out << "Primitive pos [" << state->as<StateType>()->getPrimitivePos() << "]" << std::endl;
            out << "]" << std::endl;
        }

        void LatticeStateSpace::setInitialState(State *state)
        {
            OMPL_INFORM("Set initial state...");
            if(initialState_ != nullptr) 
                freeState(initialState_);
            initialState_ = state;
        }

        const State *LatticeStateSpace::getInitialState() const
        {
            if(initialState_ == nullptr) {
                OMPL_WARN("No initial state specified, returning default state...");
                return allocState();
            }
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

        void LatticeStateSpace::setStateHasher(StateHasherPtr stateHasher) 
        {
            stateHasher_ = stateHasher;
        }

        std::string LatticeStateSpace::adjustAndHash(State *state)
        {
            return stateHasher_->adjustAndHash(state);
        }

        LatticeStateSpace::StateHasher::StateHasher() 
        {
        }

        LatticeStateSpace::StateHasher::~StateHasher() 
        {
        }

        std::string LatticeStateSpace::StateHasher::adjustAndHash(State *state) 
        {
            adjustState(state);
            return hashState(state);
        }

        bool LatticeMotionValidator::checkMotion(const State *start, const LatticeStateSpace::MotionPrimitive& primitive) const 
        {
            LatticeStateSpace::MotionPrimitive tp = ss_.transformPrimitive(start, primitive);

            auto states = tp.getStates();
            auto stateValidityChecker = si_->getStateValidityChecker();

            for(const auto& state : states) {
                if(!stateValidityChecker->isValid(state)) {
                    return false;
                }
            }

            return true;
        }

    } // namespace base

} // namespace ompl