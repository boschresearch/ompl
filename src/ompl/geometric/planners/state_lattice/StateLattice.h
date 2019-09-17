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

#ifndef OMPL_GEOMETRIC_PLANNERS_STATE_LATTICE_STATE_LATTICE
#define OMPL_GEOMETRIC_PLANNERS_STATE_LATTICE_STATE_LATTICE

// OMPL
#include <ompl/geometric/planners/PlannerIncludes.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/spaces/lattice/LatticeStateSpace.h"

// Boost
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

// STL

namespace ompl
{
    namespace geometric
    {
        class MotionPrimitive;
        typedef std::shared_ptr<MotionPrimitive> MotionPrimitivePtr;

        /**
           @anchor gStateLattice
           @par Short description
           \ref gStateLattice StateLattice is a planner that constructs a roadmap based
           on a set of motion primitives.
           TODO: add references ?
        */

        class StateLattice : public ompl::base::Planner
        {
        public:
        /** @brief Vertex property that contains the OMPL state of a vertex. */
            struct vertex_state_t
            {
                using kind = boost::vertex_property_tag;
            };
            /** @brief Vertex property used to flag whether collision checks for an edge have been performed. */
            struct vertex_flags_t
            {
                using kind = boost::vertex_property_tag;
            };

            /** @brief Edge property that holds the id of the motion primitive associated with the edge */
            struct edge_primitive_t
            {
                using kind = boost::edge_property_tag;
            };
            /** @brief Edge property used to flag whether collision checks for an edge have been performed. */
            struct edge_flags_t
            {
                using kind = boost::edge_property_tag;
            };

            // cant use using Vertex = boost::graph_traits<Graph>::vertex_descriptor; for predecessor due to circular dependency
            // exterior property for astar would be better maybe??
            /** @brief The type for a vertex in the roadmap. */
            using Vertex = boost::adjacency_list_traits<boost::listS, boost::listS, boost::bidirectionalS>::vertex_descriptor;

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix. We use listS for the vertex list
             because vertex descriptors are invalidated by remove operations if using vecS.

             @par Obviously, a ompl::base::State* vertex property is required.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges are directed to allow unsymmetric systems.
             */
            using Graph = boost::adjacency_list<
                boost::vecS, boost::listS, boost::directedS,
                boost::property<vertex_state_t, base::State *,
                                boost::property<boost::vertex_index_t, unsigned long int,
                                                boost::property<vertex_flags_t, unsigned int,
                                                                boost::property<boost::vertex_predecessor_t, Vertex>>>>,
                boost::property<boost::edge_weight_t, base::Cost, 
                                boost::property<edge_flags_t, unsigned int,
                                                boost::property<edge_primitive_t, size_t>>>>;

            // using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

            /** @brief The type for an edge in the roadmap. */
            using Edge = boost::graph_traits<Graph>::edge_descriptor;
            

            /** \brief Constructor */
            StateLattice(const base::SpaceInformationPtr &si);

            ~StateLattice() override;

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /** \brief Return the number of milestones currently in the graph */
            unsigned long int milestoneCount() const
            {
                return boost::num_vertices(g_);
            }

            /** \brief Return the number of edges currently in the graph */
            unsigned long int edgeCount() const
            {
                return boost::num_edges(g_);
            }

            void getPlannerData(base::PlannerData &data) const override;

            void setup() override;

            void clear() override;

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for the StateLattice planner. */
            void clearQuery();

            void resetLattice();
            

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        protected:
            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_UNKNOWN = 0;

            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_TRUE = 1;

            /** \brief Check and remove all invalid vertices from the lattice */
            void checkVertices();

            // VALIDITY_FALSE not required, since such edges / vertices will be removed

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Given two vertices construct a path connecting them */
            ompl::base::PathPtr constructSolution(const base::PlannerTerminationCondition &ptc, const Vertex &start, const Vertex &goal);

            /** \brief Given an underlying state, create a vertex connected to the state lattice. */
            Vertex addNonLatticeVertex(base::State *state, bool end=false);

            /** \brief Build a state lattice based on the motion primitives and store it in g_ */
            void buildLattice();

            void setupNearestNeighbors();

            /** \brief Add start and goal vertices to the lattice graph */
            bool addStartAndGoalVertices();

            /** @brief A nearest neighbors data structure for vertices. */
            std::shared_ptr<NearestNeighbors<Vertex> > nn_;

            /** \brief Connectivity graph */
            Graph g_;

            /** \brief Graph of the full lattice */
            Graph g_full_lattice_;

            /** \brief Array of start vertices */
            std::vector<Vertex> startM_;

            /** \brief Array of goal vertices */
            std::vector<Vertex> goalM_;

            /** \brief Access to the internal index at vertex */
            boost::property_map<Graph, boost::vertex_index_t>::type vertexIndexProperty_;

            /** \brief Access to the internal base::state at vertex */
            boost::property_map<Graph, vertex_state_t>::type vertexStateProperty_;

            /** \brief Access the validity state of a vertex */
            boost::property_map<Graph, vertex_flags_t>::type vertexValidityProperty_;

            /** \brief Access to the weights of an edge */
            boost::property_map<Graph, boost::edge_weight_t>::type edgeWeightProperty_;

            /** \brief Access the validity state of an edge */
            boost::property_map<Graph, edge_flags_t>::type edgeValidityProperty_;

            /** \brief Access the motion primitive of an edge */
            boost::property_map<Graph, edge_primitive_t>::type edgePrimitiveProperty_;

            /** \brief Objective cost function for graph edges */
            base::OptimizationObjectivePtr opt_;

            /** \brief Number of nearest neighbors used for connecting start and end
             * vertices to the lattice.
             */
            std::size_t nearest_k_{10}; // TODO add support for nearest_r_ and setting nearest_k_ to different values

            /** \brief Flag whether to check whether a lattice has already been built */
            bool lattice_built_{false};

            /** \brief Pointer to lattice state space */
            base::LatticeStateSpacePtr lssPtr_{nullptr};

            /** \brief Maximum number of vertices to create */
            size_t maxVertices_{100000};

            /** \brief K for using k-nearest neighbor when connecting start and goal positions */
            size_t nearestK_{10};

            /** \brief Flag whether to check all vertices before doing graph search */
            bool checkVerticesBefore_{true};

            /** \brief Pointer to the motion validator function */
            std::shared_ptr<ompl::base::LatticeMotionValidator> latticeMotionValidatorPtr_ {nullptr};
        };
    }
}

#endif