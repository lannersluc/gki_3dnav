/*
 * Params.cpp
 *
 *  Created on: 1 Mar 2013
 *      Author: andreas
 */

#include "gki_3dnav_planner/Params.h"
#include <math.h>
#include <gki_utils/misc.h>

namespace gki_3dnav_planner
{

namespace params
{

double robotLength = 0.5;
double robotWidth = 0.4;
double robotHeight = 0.4;
double minStepHeight = 0.085;
double maxStepHeight = 0.26;
double maxGapLength = 0.1;
double maxPitch = DEG2RAD(50.0);
double maxRoll = DEG2RAD(20.0);
double maxDeltaPitch = DEG2RAD(50.0);
double maxDeltaRoll = DEG2RAD(15.0);
double maxStepPitch = DEG2RAD(50.0);
double linearVelocity = 0.25; // m/s
double angularVelocity = 1.0; // rad/s

// terrain classification
double maxNormalAngleDifference = DEG2RAD(5);
double maxOriginDistanceDifference = 0.09;

// planning environment
double motionPrimitiveResolution = 0.025;
int motionPrimitiveScaleFactor = 2;
double cellSizeXY = 0.05;
double cellSizeZ = 0.25;
double cellSizeYaw = DEG2RAD(360/16.0);

// costs
int infiniteCosts = 100000000;
double rollFactor = 4.0;
double pitchFactor = 2.0;
double deltaRollFactor = 4.0;
double deltaPitchFactor = 2.0;
double stepPitchFactor = 10.0;

double edgeMeanLineDistanceThreshold = 0.01;
double edgeMaxDeltaYaw = cellSizeYaw * 0.5;

// planner
int planning_mode = 2;
bool return_first_solution = false;
double initial_eps = 10.0;
double final_eps = 1.0;
double dec_eps = 0.5;
double max_time = 30.0;
double repair_time = -10.0;

// visualization
bool show_non_ground_cells;

void updateParams(gki_3dnav_planner::OctomapServerConfig& config, uint32_t level)
{
    // robot
    robotLength = config.robot_length;
    robotWidth = config.robot_width;
    robotHeight = config.robot_height;
    minStepHeight = config.min_step_height;
    maxStepHeight = config.max_step_height;
    maxGapLength = config.max_gap_length;
    maxPitch = DEG2RAD(config.max_pitch);
    maxRoll = DEG2RAD(config.max_roll);
    maxDeltaPitch = DEG2RAD(config.max_delta_pitch);
    maxDeltaRoll = DEG2RAD(config.max_delta_roll);
    maxStepPitch = DEG2RAD(config.max_step_pitch);
    linearVelocity = config.linear_velocity;
    angularVelocity = DEG2RAD(config.angular_velocity);

    // terrain classification
    maxNormalAngleDifference = DEG2RAD(config.max_normal_angle_difference);
    maxOriginDistanceDifference = config.max_origin_distance_difference;

    // planning environment
    motionPrimitiveResolution = config.motion_primitive_resolution;
    motionPrimitiveScaleFactor = config.motion_primitive_scale_factor;
    cellSizeXY = config.discrete_cell_size_xy;
    cellSizeZ = config.discrete_cell_size_z;

    // costs
    rollFactor = config.cost_roll_factor;
    pitchFactor = config.cost_pitch_factor;
    deltaRollFactor = config.cost_delta_roll_factor;
    deltaPitchFactor = config.cost_delta_pitch_factor;
    stepPitchFactor = config.cost_step_pitch_factor;
    edgeMeanLineDistanceThreshold = config.edge_mean_line_distance_threshold;
    edgeMaxDeltaYaw = DEG2RAD(config.edge_max_delta_yaw);

    // planner
    planning_mode = config.planning_mode;
    return_first_solution = config.planner_return_first_solution;
    initial_eps = config.planner_initial_eps;
    final_eps = config.planner_final_eps;
    dec_eps = config.planner_eps_decrement;
    max_time = config.planner_max_time;
    repair_time = config.planner_repair_time;

    show_non_ground_cells = config.show_non_ground;
}

} /* namespace params */

} /* namespace gki_3dnav_planner */
