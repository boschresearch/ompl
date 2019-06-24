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

/* Authors: Leonard Bruns */

#include "ompl/geometric/planners/state_lattice/StateLattice.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Console.h"

// BGL
#include "boost/graph/copy.hpp"

// STL
#include <memory>

namespace ompl {
    namespace geometric {
        StateLattice::StateLattice(const base::SpaceInformationPtr &si) :
            base::Planner(si, "StateLattice"),
            vertexIndexProperty_(boost::get(boost::vertex_index_t(), g_)),
            vertexStateProperty_(boost::get(vertex_state_t(), g_)),
            vertexValidityProperty_(boost::get(vertex_flags_t(), g_)),
            edgeWeightProperty_(boost::get(boost::edge_weight_t(), g_)),
            edgeValidityProperty_(boost::get(edge_flags_t(), g_)),
            edgePrimitiveProperty_(boost::get(edge_primitive_t(), g_)) 
        { 

            // check if the provided space is a LatticeStateSpace
            lssPtr_ = std::dynamic_pointer_cast<base::LatticeStateSpace>(si->getStateSpace());

            // provided state space is no LatticeStateSpace
            if(!lssPtr_) 
            {
                OMPL_ERROR("No LatticeStateSpace provided to StateLattice planner.");

                // TODO: add support to automatically replace the statespace with a newly created LatticeStateSpace
            }

            // set the plannr specs
            specs_.directed = true; // directed, 

        }

        StateLattice::~StateLattice() = default;

        void StateLattice::setup() 
        {
            Planner::setup();

            // Setup nearest neighbor data structure
            if (!nn_)
            {
                nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
                nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                        {
                                            return si_->distance(vertexStateProperty_[a], vertexStateProperty_[b]);
                                        });
            }

            // Setup optimization objective
            if (pdef_)
            {
                if (pdef_->hasOptimizationObjective())
                    opt_ = pdef_->getOptimizationObjective();
                else
                {
                    opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
                }
            }
            else
            {
                OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
                setup_ = false;
            }

            // TODO: do I need a sampler? (maybe if regions are specified)
            // Setup state sampler
            // sampler_ = si_->allocStateSampler();
        }

        void StateLattice::setProblemDefinition(const base::ProblemDefinitionPtr &pdef) 
        {
            Planner::setProblemDefinition(pdef);
            clearQuery();
        }

        void StateLattice::getPlannerData(base::PlannerData &data) const
        {
            Planner::getPlannerData(data);

            // Explicitly add start and goal states. Tag all states known to be valid as 1.
            // Unchecked states are tagged as 0.
            for (auto i : startM_)
                data.addStartVertex(base::PlannerDataVertex(vertexStateProperty_[i], 1));

            for (auto i : goalM_)
                data.addGoalVertex(base::PlannerDataVertex(vertexStateProperty_[i], 1));

            // TODO: add edges, note there might be a problem, since PlannerData documentation states that "only a single directed edge conntects two vertices"
        }

        void StateLattice::clear()
        {
            Planner::clear();
            freeMemory();
            if(nn_)
                nn_->clear();
            clearQuery();
        }

        void StateLattice::clearQuery()
        {
            startM_.clear();
            goalM_.clear();
            pis_.clear();

            // TODO: remove start / end state + connections from graph??
        }

        base::PlannerStatus StateLattice::solve(const base::PlannerTerminationCondition &ptc)
        {
            checkValidity();

            // create the state lattice if it does not exist yet
            // TODO: add ptc to buld lattice function / combine solving and building lattice with both using ptc?
            if(!lattice_built_)
            {
                buildLattice();
                OMPL_INFORM("%s: Created lattice with %u states", getName().c_str(), boost::num_vertices(g_));
            }

            auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

            if (goal == nullptr)
            {
                OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
                return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
            }

            // Add the valid start states as milestones
            while (const base::State *st = pis_.nextStart())
                startM_.push_back(addNonLatticeVertex(si_->cloneState(st)));

            if (startM_.empty())
            {
                OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
                return base::PlannerStatus::INVALID_START;
            }

            if (!goal->couldSample())
            {
                OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
                return base::PlannerStatus::INVALID_GOAL;
            }

            // Ensure there is at least one valid goal state
            if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
            {
                const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
                if (st != nullptr)
                    goalM_.push_back(addNonLatticeVertex(si_->cloneState(st)));

                if (goalM_.empty())
                {
                    OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
                    return base::PlannerStatus::INVALID_GOAL;
                }
            }

            size_t startIndex = 0;
            size_t goalIndex = 0;
            bool someSolutionFound = false;
            bool fullyOptimized = false;
            base::Cost bestCost = opt_->infiniteCost();

            std::map<std::pair<size_t, size_t>, bool> firstSolutionIndices;
            base::PathPtr bestSolution = nullptr;
            while (!ptc)
            {
                base::PathPtr solution = constructSolution(ptc, startM_[startIndex], goalM_[goalIndex]);

                if (solution) 
                {
                    someSolutionFound = true;
                    base::Cost c = solution->cost(opt_);
                    if (opt_->isSatisfied(c))
                    {
                        fullyOptimized = true;
                        bestSolution = solution;
                        bestCost = c;
                        break;
                    }
                    if (opt_->isCostBetterThan(c, bestCost))
                    {
                        bestSolution = solution;
                        bestCost = c;
                    }
                }

                ++goalIndex;
                if (goalIndex > goalM_.size()) 
                {
                    goalIndex = 0;
                    ++startIndex;
                    // tried all start and goal states and no solution was found
                    if (startIndex > startM_.size() && !someSolutionFound)
                    {
                        return base::PlannerStatus::TIMEOUT;
                    }
                }
            }


            if (bestSolution)
            {
                base::PlannerSolution psol(bestSolution);
                psol.setPlannerName(getName());
                // if the solution was optimized, we mark it as such
                psol.setOptimized(opt_, bestCost, fullyOptimized);
                pdef_->addSolutionPath(psol);
            }

            // OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

            return base::PlannerStatus::EXACT_SOLUTION;
        }

        void StateLattice::buildLattice()
        {
            OMPL_INFORM("%s: Creating lazy lattice");
            lattice_built_ = true;

            // store the full lattice in case we want to plan in the same statespace, but with a different map
            boost::copy_graph(g_, g_full_lattice_);

            // TODO: add store lattice to file support
        }

        void StateLattice::addNonLatticeVertex(base::State *state)
        {

        }

        void StateLattice::freeMemory()
        {
            for (auto& vertex : boost::make_iterator_range(boost::vertices(g_))) 
            {
                si_->freeState(vertexStateProperty_[vertex]);
            }
            g_.clear();
        }

        StateLattice::Vertex StateLattice::expandVertex(base::State *state)
        {

        }

        ompl::base::PathPtr StateLattice::constructSolution(const base::PlannerTerminationCondition &ptc, const Vertex &start, const Vertex &goal)
        {

        }
    }
}