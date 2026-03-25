#include <iostream>
#include <cmath>
#include <ctime>
#include "path_optimizer_2/path_optimizer.hpp"
#include "config/planning_flags.hpp"
#include "tools/time_recorder.h"
#include "reference_path_smoother/reference_path_smoother.hpp"
#include "tools/tools.hpp"
#include "data_struct/reference_path.hpp"
#include "data_struct/data_struct.hpp"
#include "data_struct/vehicle_state_frenet.hpp"
#include "tools/collosion_checker.hpp"
#include "tools/Map.hpp"
#include "tools/spline.h"
#include "solver/base_solver.hpp"
#include "tinyspline_ros/tinysplinecpp.h"
#include "reference_path_smoother/tension_smoother.hpp"

namespace PathOptimizationNS
{

    PathOptimizer::PathOptimizer(const State &start_state,
                                 const State &end_state,
                                 const grid_map::GridMap &map) : grid_map_(std::make_shared<Map>(map)),
                                                                 collision_checker_(std::make_shared<CollisionChecker>(map)),
                                                                 reference_path_(std::make_shared<ReferencePath>()),
                                                                 vehicle_state_(std::make_shared<VehicleState>(start_state, end_state, 0.0, 0.0))
    {
    }

    void PathOptimizer::printObstacleDistance(std::vector<SlState> path)
    {
        for (int i = 0; i < path.size(); i++)
        {
            grid_map::Position position(path[i].x, path[i].y);
            PathOptimizationNS::Map map = *grid_map_;
            double map_distance = map.getObstacleDistance(position);
            std::cout << "result points distance: " << map_distance << std::endl;
        }
    }
    void PathOptimizer::printObstacleDistance2(std::vector<State> path)
    {
        for (int i = 0; i < path.size(); i++)
        {
            grid_map::Position position(path[i].x, path[i].y);
            PathOptimizationNS::Map map = *grid_map_;
            double map_distance = map.getObstacleDistance(position);
            std::cout << "smooth points distance: " << map_distance << std::endl;
        }
    }

    double PathOptimizer::calDisObs(double x,double y)
    {
        grid_map::Position position(x,y);
        PathOptimizationNS::Map map = *grid_map_;
        double distance  = map.getObstacleDistance(position);
        return distance;
    }

    bool PathOptimizer::solve(const std::vector<State> &reference_points, std::vector<SlState> *final_path)
    {
        // CHECK_NOTNULL(final_path);
        if (reference_points.empty())
        {
            std::cout << "Empty input, quit path optimization";
            return false;
        }
        // else
        // {
        //     for (int i = 0; i < reference_points.size(); i++)
        //     {
        //         grid_map::Position position(reference_points[i].x, reference_points[i].y);
        //         PathOptimizationNS::Map map = *grid_map_;
        //         double map_distance = map.getObstacleDistance(position);
        //         // std::cout << "reference points distance: " << map_distance << std::endl;
        //         // grid_map::Position ps(vehicle_state_->getStartState().x,vehicle_state_->getStartState().y);
        //         // double in_dis = map.getObstacleDistance(ps);
        //         // std::cout<<in_dis<<std::endl;
        //     }
        // }

        TimeRecorder time_recorder("Outer Solve Function");
        time_recorder.recordTime("reference path smoothing");
        // Smooth reference path.
        // TODO: refactor this part!
        reference_path_->clear();
        auto reference_path_smoother = ReferencePathSmoother::create(FLAGS_smoothing_method,
                                                                     reference_points,
                                                                     vehicle_state_->getStartState(),
                                                                     *grid_map_);
        if (!reference_path_smoother->solve(reference_path_))
        {
            std::cout << "Path optimization FAILED!";
            return false;
        }

        time_recorder.recordTime("reference path segmentation");
        // Divide reference path into segments;
        if (!processReferencePath())
        {
            std::cout << "Path optimization FAILED!";
            return false;
        }

        time_recorder.recordTime("path optimization");
        // Optimize.
        if (!optimizePath(final_path))
        {
            std::cout << "Path optimization FAILED!";
            return false;
        }
        time_recorder.recordTime("end");
        // time_recorder.printTime();
        return true;
    }

    void PathOptimizer::processInitState()
    {
        // Calculate the initial deviation and the angle difference.
        State init_point(reference_path_->getXS(0.0),
                         reference_path_->getYS(0.0),
                         getHeading(reference_path_->getXS(), reference_path_->getYS(), 0.0));
        auto first_point_local = global2Local(vehicle_state_->getStartState(), init_point);
        // In reference smoothing, the closest point to the vehicle is found and set as the
        // first point. So the distance here is simply the initial offset.
        double min_distance = distance(vehicle_state_->getStartState(), init_point);
        double initial_offset = first_point_local.y < 0.0 ? min_distance : -min_distance;
        double initial_heading_error = constrainAngle(vehicle_state_->getStartState().heading - init_point.heading);
        vehicle_state_->setInitError(initial_offset, initial_heading_error);
    }

    void PathOptimizer::setReferencePathLength()
    {
        State end_ref_state(reference_path_->getXS(reference_path_->getLength()),
                            reference_path_->getYS(reference_path_->getLength()),
                            getHeading(reference_path_->getXS(), reference_path_->getYS(), reference_path_->getLength()));
        auto vehicle_target_to_end_ref_state = global2Local(end_ref_state, vehicle_state_->getTargetState());
        // Target state is out of ref line.
        if (vehicle_target_to_end_ref_state.x > 0.0)
        {
            return;
        }
        auto target_projection = getProjection(reference_path_->getXS(),
                                               reference_path_->getYS(),
                                               vehicle_state_->getTargetState().x,
                                               vehicle_state_->getTargetState().y,
                                               reference_path_->getLength(),
                                               0.0);
        reference_path_->setLength(target_projection.s);
    }

    bool PathOptimizer::processReferencePath()
    {
        if (isEqual(reference_path_->getLength(), 0.0))
        {
            std::cout << "Smoothed path is empty!";
            return false;
        }

        processInitState();
        // If the start heading differs a lot with the ref path, quit.
        if (fabs(vehicle_state_->getInitError().back()) > 75 * M_PI / 180)
        {
            std::cout << "Initial psi error is larger than 75°, quit path optimization!";
            return false;
        }
        setReferencePathLength();

        reference_path_->buildReferenceFromSpline(FLAGS_output_spacing / 2.0, FLAGS_output_spacing);
        reference_path_->updateBounds(*grid_map_);
        return true;
    }

    bool PathOptimizer::optimizePath(std::vector<SlState> *final_path)
    {
        // CHECK_NOTNULL(final_path);
        if (!final_path->empty())
        {
            std::cout << "optimizePath final path is not empty" << std::endl;
        }
        final_path->clear();
        // TODO: remove arg: iter_num.
        std::vector<SlState> input_path;
        for (const auto &ref_state : reference_path_->getReferenceStates())
        {
            SlState input_state;
            input_state.x = ref_state.x;
            input_state.y = ref_state.y;
            input_state.heading = ref_state.heading;
            input_state.s = ref_state.s;
            input_state.k = ref_state.k;
            input_path.push_back(input_state);
        }
        BaseSolver solver(*reference_path_, *vehicle_state_, input_path);
        TimeRecorder time_recorder("Optimize Path Function");
        // Solve with soft collision constraints.
        time_recorder.recordTime("Pre solving");
        if (!solver.solve(final_path))
        {
            std::cout << "Pre solving failed!";
            reference_path_->logBoundsInfo();
            return false;
        }
        time_recorder.recordTime("Update ref");
        // reference_path_->updateBoundsOnInputStates(*grid_map_, *input_path);
        // Solve.
        time_recorder.recordTime("Solving");
        // BaseSolver post_solver(*reference_path_, *vehicle_state_, *final_path);
        // if (!post_solver.solve(final_path)) {
        if (!solver.updateProblemFormulationAndSolve(*final_path, final_path))
        {
            std::cout << "Solving failed!";
            reference_path_->logBoundsInfo();
            return false;
        }
        time_recorder.recordTime("end");
        // time_recorder.printTime();
        return true;
    }

    const ReferencePath &PathOptimizer::getReferencePath() const
    {
        return *reference_path_;
    }
}
