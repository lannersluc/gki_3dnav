/*
 * Params.h
 *
 *  Created on: 1 Mar 2013
 *      Author: andreas
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "gki_3dnav_planner/OctomapServerConfig.h"

namespace gki_3dnav_planner
{

namespace params
{
void updateParams(gki_3dnav_planner::OctomapServerConfig& config, uint32_t level);

// robot attributes and skills
extern double robotLength;
extern double robotWidth;
extern double robotHeight;
extern double minStepHeight;
extern double maxStepHeight;
extern double maxStepPitch;
extern double maxGapLength;
extern double maxPitch;
extern double maxRoll;
extern double maxDeltaPitch;
extern double maxDeltaRoll;
extern double linearVelocity;
extern double angularVelocity;

// terrain classification
extern double maxNormalAngleDifference;
extern double maxOriginDistanceDifference;

// planning environment
extern double motionPrimitiveResolution;
extern int motionPrimitiveScaleFactor;
extern double cellSizeXY;
extern double cellSizeZ;
extern double cellSizeYaw;

// costs
extern int infiniteCosts;
extern double rollFactor;
extern double pitchFactor;
extern double deltaRollFactor;
extern double deltaPitchFactor;
extern double stepPitchFactor;
extern double edgeMeanLineDistanceThreshold;
extern double edgeMaxDeltaYaw;

// planner
extern int planning_mode;
extern bool return_first_solution;
extern double initial_eps;
extern double final_eps;
extern double dec_eps;
extern double max_time;
extern double repair_time;

// visualization
extern bool show_non_ground_cells;


} /* namespace params */

} /* namespace gki_3dnav_planner */
#endif /* PARAMS_H_ */
