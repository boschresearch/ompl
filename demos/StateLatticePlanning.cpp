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

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/lattice/LatticeStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/state_lattice/StateLattice.h>
#include <ompl/util/PPM.h>

#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <memory>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class GridStateHasher : public ompl::base::LatticeStateSpace::StateHasher 
{
public:
    GridStateHasher() : StateHasher() {}
protected:
    void adjustState(ompl::base::State * state) override 
    {
        (void)state;
        // not necessary if motion primitives already lead to grid cells
    }
    std::string hashState(ompl::base::State* state) override
    {
        // produce a unique string for each grid state
        // if primitives are accurate, not as important

        std::stringstream ss;
        ss.setf(std::ios::fixed,std::ios::floatfield);
        ss.precision(2);
        auto se2state = state->as<ob::LatticeStateSpace::StateType>()->getState()->as<ob::SE2StateSpace::StateType>();

        // + 0.01 to prevent +0 and -0
        // > 0.99*M_PI => -2*M_PI
        // to prevent -pi and +pi
        if(se2state->getYaw() > 0.99*M_PI)
            ss << se2state->getX() << ";" << se2state->getY() << ";" << se2state->getYaw() + 0.01 - 2*M_PI; 
        else 
            ss << se2state->getX() << ";" << se2state->getY() << ";" << se2state->getYaw() + 0.01;

        return ss.str();
    }
};

class Plane2DEnvironment
{
public:

    Plane2DEnvironment(const char *ppm_file, boost::filesystem::path motion_primitive_path)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            // this is the space used to connect start and goal, bound checks, statetype, ... 
            auto underlying_space(std::make_shared<ob::ReedsSheppStateSpace>(40.0));

            // the lattice space wraps around the space and adds motion primitives to it
            // it also provides ways of defining how the primitives are connected, expanded and transformed
            // augments the statetype with primitive information
            auto lattice_space{std::make_shared<ompl::base::LatticeStateSpace>(underlying_space)};

            ob::RealVectorBounds bounds(2);
            bounds.setLow(0);
            bounds.setHigh(0, ppm_.getWidth());
            bounds.setHigh(1, ppm_.getHeight());
            underlying_space->setBounds(bounds);
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;

            ss_ = std::make_shared<og::SimpleSetup>(lattice_space);

            // set state validity checking for this space
            ss_->setStateValidityChecker([this, lattice_space](const ob::State *state) { return isStateValid(state, lattice_space); });
            lattice_space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1 / lattice_space->getMaximumExtent());

            auto state_lattice_planner = std::make_shared<ompl::geometric::StateLattice>(ss_->getSpaceInformation());
            // do both vertices and edges prior to A*, best setting depends on speed of collision checker and size of graph
            // here: very fast collision checker, large graph -> all checks before planning
            state_lattice_planner->setCollisionCheckStrategy(true, true);
            ss_->setPlanner(state_lattice_planner);

            // primitive validator (i.e. check if a given motion primitive can be applied in a given state)
            lattice_space->setPrimitiveValidator( std::bind(&Plane2DEnvironment::primitiveValidator, this, std::placeholders::_1, std::placeholders::_2));

            // primitive interpolator
            lattice_space->as<ompl::base::LatticeStateSpace>()->setPrimitiveInterpolator( std::bind(&Plane2DEnvironment::primitiveInterpolator, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

            // transform calculator
            lattice_space->setTransformCalculator( [this](const ompl::base::State* s, const ompl::base::LatticeStateSpace::MotionPrimitive& p) {
                return transformCalculator(s,p);
            });

            // cost heuristic (default uses distance from underlying space, this could be used for precomputed look up table for example)
            lattice_space->setDefaultCostHeuristic();

            // state hasher (this is used to obtain a mapping from state to grid, this could be used to realize approximate state lattice planning)
            lattice_space->setStateHasher(std::make_shared<GridStateHasher>());

            addMotionPrimitives(motion_primitive_path, ss_->getSpaceInformation());
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        if (!ss_)
            return false;
        ob::ScopedState<ob::LatticeStateSpace> start(ss_->getStateSpace());
        start.get()->getState()->as<ob::SE2StateSpace::StateType>()->setXY(start_row, start_col);
        ob::ScopedState<ob::LatticeStateSpace> goal(ss_->getStateSpace());
        goal.get()->getState()->as<ob::SE2StateSpace::StateType>()->setXY(goal_row, goal_col);
        ss_->setStartAndGoalStates(start, goal);

        ss_->solve(ob::PlannerTerminationCondition(ob::plannerNonTerminatingCondition));

        if (ss_->haveSolutionPath())
        {
            return true;
        }

        return false;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate(10000);
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
        {
            const int w = std::min(maxWidth_, (int)getUnderlying(p.getState(i))->getX());
            const int h = std::min(maxHeight_, (int)getUnderlying(p.getState(i))->getY());
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }

    void save(const char *filename)
    {
        if (!ss_)
            return;
        ppm_.saveFile(filename);
    }

private:
    double abs_angle_difference(double x, double y)  
    {
        return M_PI - std::abs(std::fmod(std::abs(x-y), 2*M_PI) - M_PI);
    }

    ob::SE2StateSpace::StateType* getUnderlying(ob::State* lattice_state) const
    {
        return lattice_state->as<ob::LatticeStateSpace::StateType>()->getState()->as<ob::SE2StateSpace::StateType>();
    }

    const ob::SE2StateSpace::StateType* getUnderlying(const ob::State* lattice_state) const
    {
        return lattice_state->as<ob::LatticeStateSpace::StateType>()->getState()->as<ob::SE2StateSpace::StateType>();
    }

    bool isStateValid(const ob::State *state, const ob::StateSpacePtr statespace) const
    {
        if(!statespace->satisfiesBounds(state))
            return false;

        const int w = std::min((int)(getUnderlying(state)->getX()), maxWidth_);
        const int h = std::min((int)(getUnderlying(state)->getY()), maxHeight_);


        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }


    bool primitiveValidator(const ompl::base::State* state, const ompl::base::LatticeStateSpace::MotionPrimitive& primitive)
    {
        // only primitives starting with same start heading angle can originate from it

        double yaw_state = state->as<ompl::base::LatticeStateSpace::StateType>()->getState()->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
        double yaw_primitive = primitive.getState(0)->as<ompl::base::LatticeStateSpace::StateType>()->getState()->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

        if(abs_angle_difference(yaw_state, yaw_primitive) < 0.02) 
            return true;
        else
            return false;
        
    }

    std::function<void (ompl::base::State*)> transformCalculator(const ompl::base::State* state1, const ompl::base::State* state2) 
    {
        // return a function that transforms the primitive to a new starting state

        double x_state = getUnderlying(state1)->getX();
        double y_state = getUnderlying(state1)->getY();
        double yaw_state = getUnderlying(state1)->getYaw();
        double x_state_2 = getUnderlying(state2)->getX();
        double y_state_2 = getUnderlying(state2)->getY();
        double yaw_state_2 = getUnderlying(state2)->getYaw();

        if(abs_angle_difference(yaw_state, yaw_state_2) > 0.05) 
        {
            OMPL_WARN("transformCalculator: state and goal state starting yaw differs more than 0.05rad, check the code");
        } 

        double delta_x = x_state - x_state_2;
        double delta_y = y_state - y_state_2;

        return [delta_x, delta_y] (ompl::base::State* state1) {
            (*(state1->as<ob::LatticeStateSpace::StateType>()->getState()->as<ob::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0] += delta_x;
            (*(state1->as<ob::LatticeStateSpace::StateType>()->getState()->as<ob::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1] += delta_y;
        };
    }

    std::function<void (ompl::base::State*)> transformCalculator(const ompl::base::State* state, const ompl::base::LatticeStateSpace::MotionPrimitive& motion_primitive) 
    {
        return transformCalculator(state, motion_primitive.getState(0));
    }

    void primitiveInterpolator(const ompl::base::State* s1, const ompl::base::State* s2, double t, const ompl::base::LatticeStateSpace::MotionPrimitive& motion_primitive, ompl::base::State* state) 
    {
        // linearly interpolate on the primitive
        // maybe this could be provided as a default? (not sure if possible to write it StateType agnostic)

        // this function uses the following conditions (which are ensured by the LatticeStateSpace):
        // - s1 and s2 are connected by primitive
        // - s1 comes before s2
        // - thus: s1 and s2 are both on the same primitive s1.t < s2.t
        // - s2 might be the starting point, but nothing else, of another motion primitive (in that case s2.t==0)
        double t1 = s1->as<ompl::base::LatticeStateSpace::StateType>()->getPrimitivePos();
        double t2 = s2->as<ompl::base::LatticeStateSpace::StateType>()->getPrimitivePos();

        // t_primitive is the relative position on the primitive
        double t_primitive;
        if(t2 == 0) // t2 is starting point of next primitive 
            t_primitive = t1 + t * (1-t1);
        else // both on same primitive
            t_primitive = t1 + t * (t2 - t1);


        // find the two states left and right of t_primitive on the primitive
        size_t state_count = motion_primitive.getStateCount();
        
        size_t idx_1 = std::floor(t1 * (state_count-1)); // corresponding primitive idx of s1

        size_t idx_left = std::floor(t_primitive * (state_count-1));

        double t_left = (1.0 / state_count) * idx_left;

        size_t idx_right = idx_left + 1;

        auto transform = transformCalculator(s1, motion_primitive.getState(idx_1));

        if(idx_right >= state_count) // last state
        {
            double x_primitive = getUnderlying(motion_primitive.getState(idx_left))->getX();
            double y_primitive = getUnderlying(motion_primitive.getState(idx_left))->getY();
            double yaw_primitive = getUnderlying(motion_primitive.getState(idx_left))->getYaw();
            getUnderlying(state)->setX(x_primitive);
            getUnderlying(state)->setY(y_primitive);
            getUnderlying(state)->setYaw(yaw_primitive);
            state->as<ompl::base::LatticeStateSpace::StateType>()->setPrimitiveId(s1->as<ompl::base::LatticeStateSpace::StateType>()->getPrimitiveId());
            state->as<ompl::base::LatticeStateSpace::StateType>()->setPrimitivePos(t_primitive);
            transform(state);
        }
        else 
        {
            // find the t between the two states
            double t_between = (t_primitive - t_left) * state_count;

            double x_primitive_1 = getUnderlying(motion_primitive.getState(idx_left))->getX();
            double y_primitive_1 = getUnderlying(motion_primitive.getState(idx_left))->getY();
            double yaw_primitive_1 = getUnderlying(motion_primitive.getState(idx_left))->getYaw();

            double x_primitive_2 = getUnderlying(motion_primitive.getState(idx_right))->getX();
            double y_primitive_2 = getUnderlying(motion_primitive.getState(idx_right))->getY();
            double yaw_primitive_2 = getUnderlying(motion_primitive.getState(idx_right))->getYaw();

            getUnderlying(state)->setX(x_primitive_1 + t_between * (x_primitive_2 - x_primitive_1));
            getUnderlying(state)->setY(y_primitive_1 + t_between * (y_primitive_2 - y_primitive_1));
            getUnderlying(state)->setYaw(yaw_primitive_1 + t_between * (yaw_primitive_2 - yaw_primitive_1));

            state->as<ompl::base::LatticeStateSpace::StateType>()->setPrimitiveId(s1->as<ompl::base::LatticeStateSpace::StateType>()->getPrimitiveId());
            state->as<ompl::base::LatticeStateSpace::StateType>()->setPrimitivePos(t_primitive);
            transform(state);
        }
    }


    bool addMotionPrimitives(boost::filesystem::path filepath, const ompl::base::SpaceInformationPtr& si)
    {
        std::ifstream ifs(filepath.string());
        if(ifs.is_open()) 
        {
            std::string line;
            ompl::base::LatticeStateSpace::MotionPrimitive motionPrimitive(si);
            while(std::getline(ifs, line)) {
                if(line == "") {// empty line => next motion primitive
                    si->getStateSpace()->as<ompl::base::LatticeStateSpace>()->addMotionPrimitive(motionPrimitive);
                    motionPrimitive = ompl::base::LatticeStateSpace::MotionPrimitive(si);
                    continue;
                }
                
                std::istringstream iss(line);
                double x,y,yaw;
                iss >> x >> y >> yaw;

                auto next_state = si->allocState();
                auto next_se2_state = next_state->as<ompl::base::LatticeStateSpace::StateType>()->getState()->as<ompl::base::SE2StateSpace::StateType>();
                next_se2_state->setX(x);
                next_se2_state->setY(y);
                next_se2_state->setYaw(yaw);
                motionPrimitive.getStates().push_back(next_state);
            }
            if(motionPrimitive.getStateCount() != 0) {
                si->getStateSpace()->as<ompl::base::LatticeStateSpace>()->addMotionPrimitive(motionPrimitive);
                motionPrimitive = ompl::base::LatticeStateSpace::MotionPrimitive(si);
            }

            OMPL_INFORM("Motion primitives added successfully");
            return true;
        }
        else 
        {
            OMPL_ERROR("Could not open motion primitive file, check the path.");
            return false;
        }
    }

    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;

};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    boost::filesystem::path path(TEST_RESOURCES_DIR);
    Plane2DEnvironment env((path / "ppm/floor.ppm").string().c_str(), path / "motion_primitives.txt");

    if (env.plan(200, 1270, 1700, 700))
    {
        env.recordSolution();
        env.save("result_demo.ppm");
    }

    return 0;
}
