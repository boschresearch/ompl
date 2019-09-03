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
#include "ompl/geometric/planners/prm/src/GoalVisitor.hpp"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/State.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Console.h"

// BGL
#include "boost/graph/copy.hpp"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

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

        void StateLattice::setupNearestNeighbors() 
        {
            // Setup nearest neighbor data structure
            if (!nn_)
            {
                nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
                nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                        {
                                            return si_->distance(vertexStateProperty_[a], vertexStateProperty_[b]);
                                        });

                // OMPL_INFORM("Nearest neightbor datastructure initialized.");
            }
        }

        void StateLattice::setup() 
        {
            // OMPL_INFORM("Enter setup function...");

            Planner::setup();

            // OMPL_INFORM("Planner::setup completed.");

            // Setup nearest neighbor data structure
            setupNearestNeighbors();

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

            // OMPL_INFORM("Setup completed.");

            // TODO: do I need a sampler? (maybe if regions are specified)
            // Setup state sampler
            // sampler_ = si_->allocStateSampler();
        }

        void StateLattice::setProblemDefinition(const base::ProblemDefinitionPtr &pdef) 
        {
            // OMPL_INFORM("Set problem definition");
            Planner::setProblemDefinition(pdef);
            clearQuery();
        }

        void StateLattice::getPlannerData(base::PlannerData &data) const
        {
            // OMPL_INFORM("Get planner data");
            Planner::getPlannerData(data);

            // Explicitly add start and goal states. Tag all states known to be valid as 1.
            // Unchecked states are tagged as 0.
            for (auto i : startM_)
                data.addStartVertex(base::PlannerDataVertex(vertexStateProperty_[i], 1));

            for (auto i : goalM_)
                data.addGoalVertex(base::PlannerDataVertex(vertexStateProperty_[i], 1));

            // Adding edges and all other vertices simultaneously
            boost::graph_traits<Graph>::edge_iterator ei, eend;
            for(boost::tie(ei, eend) = boost::edges(g_); ei != eend; ++ei)
            {
                const Vertex v1 = boost::source(*ei, g_);
                const Vertex v2 = boost::target(*ei, g_);
                data.addEdge(base::PlannerDataVertex(vertexStateProperty_[v1]), base::PlannerDataVertex(vertexStateProperty_[v2]));
            }
        }

        void StateLattice::clear()
        {
            Planner::clear();
            freeMemory();
            if(nn_)
                nn_->clear();
            clearQuery();
        }

        void StateLattice::resetLattice()
        {
            // setup nearest neighbors structure, if not done so far
            setupNearestNeighbors();

            // build lattice if no lattice has been built so far
            if(!lattice_built_) {
                buildLattice();
                OMPL_INFORM("%s: Created lattice with %u states", getName().c_str(), boost::num_vertices(g_));
            }
            else {
                // OMPL_INFORM("%s: Reset lattice, graph has %u vertices", getName().c_str(), boost::num_vertices(g_));

                g_ = g_full_lattice_;

                // need to create new nearest neighbor structure, since vertex addresses have changed
                // OMPL_INFORM("%s: Attempting to recreate nearest neighbor structure...", getName().c_str());
                nn_->clear();

                boost::graph_traits<Graph>::vertex_iterator vi, vend;
                for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi)
                    nn_->add(*vi);

                // OMPL_INFORM("%s: Nearest neighbor structure successfully created", getName().c_str());
                // boost::copy_graph(g_full_lattice_, g_);
                OMPL_INFORM("%s: Reset lattice, graph has %u vertices", getName().c_str(), boost::num_vertices(g_));
            }
        }

        void StateLattice::checkVertices()
        {
            std::vector<Vertex> verticesToRemove;
            boost::graph_traits<Graph>::vertex_iterator vi, vend;
            for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi) {
                const base::State *st = vertexStateProperty_[*vi];
                unsigned int &vd = vertexValidityProperty_[*vi];
                if ((vd & VALIDITY_TRUE) == 0)
                    if (si_->isValid(st))
                        vd |= VALIDITY_TRUE;
                if ((vd & VALIDITY_TRUE) == 0)
                    verticesToRemove.push_back(*vi);
            }

            // OMPL_INFORM("constructSolution: removing %u vertices", verticesToRemove.size());

            // We remove *all* invalid vertices.
            if (!verticesToRemove.empty())
            {
                // Remember the current neighbors.
                for (auto it = verticesToRemove.begin(); it != verticesToRemove.end(); ++it)
                {
                    // Remove vertex from nearest neighbors data structure.
                    // nn_->remove(*it);
                    // Remove all edges.
                    boost::clear_vertex(*it, g_);
                    // Remove the vertex.
                    boost::remove_vertex(*it, g_);
                }
            }
        }

        void StateLattice::clearQuery()
        {
            startM_.clear();
            goalM_.clear();
            pis_.restart();

            // TODO: remove start / end state + connections from graph??
        }

        base::PlannerStatus StateLattice::solve(const base::PlannerTerminationCondition &ptc)
        {

            // check if lattice motion validator is chosen
            latticeMotionValidatorPtr_ = std::dynamic_pointer_cast<ompl::base::LatticeMotionValidator>(si_->getMotionValidator());

            // OMPL_INFORM("Enter solve function...");
            checkValidity();

            // OMPL_INFORM("Validity checked succesfully.");

            // create the state lattice if it does not exist yet
            // TODO: add ptc to buld lattice function / combine solving and building lattice with both using ptc?
            if(!lattice_built_)
            {
                buildLattice();
                OMPL_INFORM("%s: Created lattice with %u states", getName().c_str(), boost::num_vertices(g_));
            }


            // OMPL_INFORM("Lattice built succesfully.");

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
                    goalM_.push_back(addNonLatticeVertex(si_->cloneState(st), true)); // true indicates end direction

                if (goalM_.empty())
                {
                    OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
                    return base::PlannerStatus::INVALID_GOAL;
                }
            }

            // OMPL_INFORM("Start and goal added succesfully.");


            // either check vertices now, or only for A* solutions
            // it depends on how costly the collision check, which way performs better
            if(checkVerticesBefore_)
                checkVertices();

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
                    // OMPL_INFORM("Some solution was found.");
                    base::Cost c = solution->cost(opt_);
                    if (opt_->isSatisfied(c))
                    {
                        // OMPL_INFORM("solve: opt is satisfied...");
                        fullyOptimized = true;
                        bestSolution = solution;
                        bestCost = c;
                        break;
                    }
                    if (opt_->isCostBetterThan(c, bestCost))
                    {
                        // OMPL_INFORM("solve: new best cost...");
                        bestSolution = solution;
                        bestCost = c;
                    }
                }

                ++goalIndex;
                if (goalIndex >= goalM_.size()) 
                {
                    // OMPL_INFORM("solve: all goals tested for startindex %u...", startIndex);
                    goalIndex = 0;
                    ++startIndex;
                    if (startIndex >= startM_.size()) // all start / goal combinations have been tried
                    {
                        if(!someSolutionFound) // no solution found -> there is no solution with this lattice
                        {
                            OMPL_INFORM("Tried all start/goal combinations without finding a solution.");
                            return base::PlannerStatus::TIMEOUT;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }


            if (bestSolution)
            {
                // OMPL_INFORM("Add best solution as solution path");
                base::PlannerSolution psol(bestSolution);
                psol.setPlannerName(getName());
                // if the solution was optimized, we mark it as such
                psol.setOptimized(opt_, bestCost, fullyOptimized);
                pdef_->addSolutionPath(psol);
                // OMPL_INFORM("Solution path added");
            }

            // OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

            // OMPL_INFORM("Solve completed.");
            return base::PlannerStatus::EXACT_SOLUTION;
        }

        void StateLattice::freeMemory()
        {
            for (auto& vertex : boost::make_iterator_range(boost::vertices(g_))) 
            {
                si_->freeState(vertexStateProperty_[vertex]);
            }
            g_.clear();
        }

        void StateLattice::buildLattice()
        {
            // OMPL_INFORM("buildLattice: Creating lazy lattice");
            lattice_built_ = true;

            std::map<std::string, Vertex> stateVertexMap;

            Vertex v = boost::add_vertex(g_);
            // OMPL_INFORM("buildLattice: Added first vertex to graph");
            base::State* state = lssPtr_->cloneState(lssPtr_->getInitialState());
            stateVertexMap[lssPtr_->adjustAndHash(state)] = v;
            vertexStateProperty_[v] = state; // TODO: better initial state handling?
            // OMPL_INFORM("buildLattice: Assign initial state to first vertex");
            vertexValidityProperty_[v] = VALIDITY_UNKNOWN;
            // OMPL_INFORM("buildLattice: Assign initial state to first vertex");
            nn_->add(v);
            // OMPL_INFORM("buildLattice: Add initial vertex to nearest neighbor structure");

            std::vector<Vertex> toBeExpanded;
            toBeExpanded.push_back(v);

            // OMPL_INFORM("buildLattic: Init state");
            // lssPtr_->printState(vertexStateProperty_[v]);

            // OMPL_INFORM("buildLattice: init build lattice algorithm");

            

            while ( boost::num_vertices(g_) < maxVertices_ && toBeExpanded.size() != 0 ) 
            {
                Vertex curVertex = toBeExpanded.back();
                base::State* curState = vertexStateProperty_[curVertex];

                // OMPL_INFORM("buildLattice: state expanded, toBeExpanded size: %u, total size: %u", toBeExpanded.size(), boost::num_vertices(g_));
                // lssPtr_->printState(vertexStateProperty_[curVertex]);

                toBeExpanded.pop_back();

                std::vector<size_t> outPrimitives = lssPtr_->getOutPrimitives(vertexStateProperty_[curVertex]);

                for( const size_t primitive : outPrimitives)
                {
                    auto endState = lssPtr_->getEndState(curState, primitive);

                    if(lssPtr_->satisfiesBounds(endState)) 
                    {
                        std::string stateHash = lssPtr_->adjustAndHash(endState);
                        auto stateVertexPairIt = stateVertexMap.find(stateHash);
                        Vertex endVertex;
                        if(stateVertexPairIt == stateVertexMap.end()) { // vertex does not exist yet
                            endVertex = boost::add_vertex(g_);
                            stateVertexMap[stateHash] = endVertex;
                            vertexStateProperty_[endVertex] = endState; 
                            vertexValidityProperty_[endVertex] = VALIDITY_UNKNOWN;
                            nn_->add(endVertex);

                            // OMPL_INFORM("buildLattice: vertex added, total %u", boost::num_vertices(g_));
                            // lssPtr_->printState(vertexStateProperty_[endVertex]);
                            toBeExpanded.push_back(endVertex);
                        } 
                        else {
                            endVertex = stateVertexPairIt->second;
                        }

                        auto edge = boost::add_edge(curVertex, endVertex, g_);
                        edgeValidityProperty_[edge.first] = VALIDITY_UNKNOWN;
                        // TODO: different options to handle motion primitive cost, precomputed, based on opt_, based on motion primitive length, ...?
                        edgeWeightProperty_[edge.first] = lssPtr_->getMotionPrimitiveCost(primitive);
                        edgePrimitiveProperty_[edge.first] = primitive;

                    }

                }
            }

            // OMPL_INFORM("buildLattice: lattice built successfully");

            g_full_lattice_.clear();
            g_full_lattice_ = g_;

            
            // TODO: add store lattice to file support
        }

        StateLattice::Vertex StateLattice::addNonLatticeVertex(base::State *state, bool end)
        {
            // first create the vertex and add all properties
            Vertex v = boost::add_vertex(g_);
            vertexStateProperty_[v] = state;
            vertexValidityProperty_[v] = VALIDITY_UNKNOWN;
            std::vector<Vertex> neighbors;
            nn_->nearestK(v, nearestK_, neighbors);

            // OMPL_INFORM("Adding non lattice states ");
            for(const auto& neighbor : neighbors)
            {
                if(!end) 
                {
                    auto edge = boost::add_edge(v, neighbor, g_);
                    edgeValidityProperty_[edge.first] = VALIDITY_UNKNOWN;
                    edgeWeightProperty_[edge.first] = opt_->motionCost(vertexStateProperty_[v], vertexStateProperty_[neighbor]);
                    edgePrimitiveProperty_[edge.first] = -1;

                    // OMPL_INFORM("Connected to: ");
                    // si_->printState(vertexStateProperty_[neighbor]);
                }
                else 
                {
                    auto edge = boost::add_edge(neighbor, v, g_);
                    edgeValidityProperty_[edge.first] = VALIDITY_UNKNOWN;
                    edgeWeightProperty_[edge.first] = opt_->motionCost(vertexStateProperty_[neighbor], vertexStateProperty_[v]);
                    edgePrimitiveProperty_[edge.first] = -1;

                    // OMPL_INFORM("Connected to: ");
                    // si_->printState(vertexStateProperty_[neighbor]);
                }
            }

            // nn_->add(v);

            return v;
        }

        ompl::base::PathPtr StateLattice::constructSolution(const base::PlannerTerminationCondition &ptc, const Vertex &start, const Vertex &goal)
        {
            // OMPL_INFORM("constructSolution: Starting construct solution...");
            size_t counter=1;

            while(true)
            {
                // OMPL_INFORM("constructSolution: init try %u", counter);
                ++counter;
                // Need to update the index map here, becuse nodes may have been removed and
                // the numbering will not be 0 .. N-1 otherwise.
                unsigned long int index = 0;
                boost::graph_traits<Graph>::vertex_iterator vi, vend;
                for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi, ++index)
                    vertexIndexProperty_[*vi] = index;

                boost::property_map<Graph, boost::vertex_predecessor_t>::type prev;
                try
                {
                    // Consider using a persistent distance_map if it's slow
                    boost::astar_search(g_, start,
                                        [this, goal](Vertex v)
                                        {
                                            return lssPtr_->costHeuristic(vertexStateProperty_[v], vertexStateProperty_[goal]);
                                        },
                                        boost::predecessor_map(prev)
                                            .distance_compare([this](base::Cost c1, base::Cost c2)
                                                            {
                                                                return opt_->isCostBetterThan(c1, c2);
                                                            })
                                            .distance_combine([this](base::Cost c1, base::Cost c2)
                                                            {
                                                                return opt_->combineCosts(c1, c2);
                                                            })
                                            .distance_inf(opt_->infiniteCost())
                                            .distance_zero(opt_->identityCost())
                                            .visitor(AStarGoalVisitor<Vertex>(goal)));
                }
                catch (AStarFoundGoal &)
                {
                    // OMPL_INFORM("Astar found goal");
                }
                if (prev[goal] == goal) { // no solution found
                    // OMPL_INFORM("constructSolution: no solution found");
                    return base::PathPtr(); // shared_ptr -> default is nullptr
                }
                // OMPL_INFORM("constructSolution: solution found, check for collisions now");

                // First, get the solution states without copying them, and check them for validity.
                // We do all the node validity checks for the vertices, as this may remove a larger
                // part of the graph (compared to removing an edge).
                std::vector<const base::State *> states(1, vertexStateProperty_[goal]); // all valid states
                std::set<Vertex> verticesToRemove;
                for (Vertex pos = prev[goal]; prev[pos] != pos; pos = prev[pos])
                {
                    const base::State *st = vertexStateProperty_[pos];
                    unsigned int &vd = vertexValidityProperty_[pos];
                    if ((vd & VALIDITY_TRUE) == 0)
                        if (si_->isValid(st))
                            vd |= VALIDITY_TRUE;
                    if ((vd & VALIDITY_TRUE) == 0)
                        verticesToRemove.insert(pos);
                    if (verticesToRemove.empty())
                        states.push_back(st);
                }

                // OMPL_INFORM("constructSolution: have to remove %u vertices", verticesToRemove.size());

                // We remove *all* invalid vertices.
                if (!verticesToRemove.empty())
                {
                    for (auto it = verticesToRemove.begin(); it != verticesToRemove.end(); ++it)
                    {
                        // Remove vertex from nearest neighbors data structure.
                        // nn_->remove(*it); // not actually required, but more safe to do it anyways
                        // Remove all edges.
                        boost::clear_vertex(*it, g_);
                        // Remove the vertex.
                        boost::remove_vertex(*it, g_);
                    }
                    continue; // try again with new A* query
                }

                // OMPL_INFORM("constructSolution: no more vertices to remove", verticesToRemove.empty());

                // start is checked for validity already
                states.push_back(vertexStateProperty_[start]);

                // Check the edges too, if the vertices were valid. Remove the first invalid edge only.
                std::vector<const base::State *>::const_iterator prevState = states.begin(), state = prevState + 1;
                std::vector<int> primitive_ids;
                primitive_ids.push_back(-1);
                // OMPL_INFORM("constructSolution: checking edges now");
                Vertex prevVertex = goal, pos = prev[goal];
                bool all_valid = true;
                do
                {
                    // TODO: how does this handle parallel edges??
                    // check https://stackoverflow.com/questions/21214091/find-multiple-edges-given-2-vertices-in-boost-graph for 
                    // some hints

                    Edge e = boost::lookup_edge(pos, prevVertex, g_).first;
                    unsigned int &evd = edgeValidityProperty_[e];
                    int primitive_id = edgePrimitiveProperty_[e];
                    primitive_ids.push_back(primitive_id);
                    if ((evd & VALIDITY_TRUE) == 0)
                    {
                        if(latticeMotionValidatorPtr_ != nullptr && primitive_id != -1) 
                        {
                            if(latticeMotionValidatorPtr_->checkMotion(*state, lssPtr_->getMotionPrimitive(primitive_id)))
                                evd |= VALIDITY_TRUE;
                        }
                        else if (si_->checkMotion(*prevState, *state))
                            evd |= VALIDITY_TRUE;
                    }
                    if ((evd & VALIDITY_TRUE) == 0)
                    {
                        // OMPL_INFORM("constructSolution: Removing edge.");
                        boost::remove_edge(e, g_);
                        all_valid = false;
                        break; // try again with new A* query
                    }
                    prevState = state;
                    ++state;
                    prevVertex = pos;
                    pos = prev[pos];
                } while (prevVertex != pos);

                if(!all_valid)
                    continue;

                // OMPL_INFORM("constructSolution: path is valid, construct solution...", verticesToRemove.empty());

                auto p(std::make_shared<PathGeometric>(si_));

                // states vector is goal -> start, reverse for correct order
                for(int i = states.size() - 1; i >= 0; --i) {
                    p->append(states[i]);
                    p->getStates().back()->as<ompl::base::LatticeStateSpace::StateType>()->setPrimitiveId(primitive_ids[i]);
                }
                return p;
            }
        }
    }
}