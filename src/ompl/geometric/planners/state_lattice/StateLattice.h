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

#include <ompl/geometric/planners/PlannerIncludes.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

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
                                                boost::property<vertex_flags_t, unsigned int>>>,
                boost::property<boost::edge_weight_t, base::Cost, 
                                boost::property<edge_flags_t, unsigned int,
                                                boost::property<edge_primitive_t, size_t>>>>;

            // TODO: lazy PRM does:
            // using Vertex = boost::adjacency_list_traits<boost::vecS, boost::listS, boost::undirectedS>::vertex_descriptor;
            // why is that?

            /** @brief The type for a vertex in the roadmap. */
            using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

            /** @brief The type for an edge in the roadmap. */
            using Edge = boost::graph_traits<Graph>::edge_descriptor;

            /** @brief A nearest neighbors data structure for roadmap vertices. */
            using RoadmapNeighbors = std::shared_ptr<NearestNeighbors<Vertex> >;

            /** @brief A function returning the milestones of the lattice the start and end 
             * vertices should be attempted to connect to. */
            using ConnectionStrategy = std::function<const std::vector<Vertex> &(const Vertex)>;
            

            /** \brief Constructor */
            StateLattice(const base::SpaceInformationPtr &si);

            ~StateLattice() override;

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously constructed state lattice,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for StateLattice. */
            void clearQuery();

            /** \brief Set the connection strategy function that specifies the
             milestones that connection attempts will be made to for the start and end vertex.

             \par Since this is only used for two vertices a very liberal connection strategy,
             i.e. one that tries to connect to many vertices seems reasonable.

             \param pdef A function that takes a milestone as an argument and
             returns a collection of other milestones to which a connection
             attempt must be made. The default connection strategy is to connect
             a milestone's 10 closest neighbors.
             */
            void setConnectionStrategy(const ConnectionStrategy &connectionStrategy)
            {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }
            /** Set default strategy for connecting start and end */
            void setDefaultConnectionStrategy();


            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() == 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Vertex>>();
                if (!userSetConnectionStrategy_)
                    setDefaultConnectionStrategy();
                if (isSetup())
                    setup();
            }

            /** \brief Convenience function that sets the connection strategy to the
             default one with k nearest neighbors.
             */
            void setMaxNearestNeighbors(unsigned int k);


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

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        protected:
            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_UNKNOWN = 0;

            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_TRUE = 1;

            // VALIDITY_FALSE not required, since such edges / vertices will be removed

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Expands a milestone by following all fitting motion primitives from that state
             * and connect it to the roadmap accordingly to the motion primitive.
            */
            Vertex expandMilestone(base::State *state);

            /** \brief Given two milestones construct a path connecting them and set it as the solution */
            ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy connectionStrategy_;

            /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are
             * assumed) */
            bool userSetConnectionStrategy_{false};

            /** \brief Nearest neighbors data structure */
            RoadmapNeighbors nn_;

            /** \brief Connectivity graph */
            Graph g_;

            /** \brief Array of start milestones */
            std::vector<Vertex> startM_;

            /** \brief Array of goal milestones */
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
        };
    }
}

#endif