/*
 * PlanningEnvironment.cpp
 *
 *  Created on: 28 Feb 2013
 *      Author: andreas
 */

#include "gki_3dnav_planner/PlanningEnvironmentODSH.h"
#include "gki_3dnav_planner/Params.h"
#include <ros/assert.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/utils.h>
#include <gki_utils/misc.h>

#define NUMOFINDICES_STATEID2IND 2

namespace gki_3dnav_planner
{

PlanningEnvironmentODSH::PlanningEnvironmentODSH(
        const CollisionChecker* checker,
        SBPLPlanner* heuristicPlanner,
        PlanningEnvironment* heuristicEnvironment)
{
    ROS_INFO("%s begins", __PRETTY_FUNCTION__);
    this->checker = checker;
    startState = NULL;
    goalState = NULL;
    State::cellSize.xy = params::cellSizeXY; // m
    State::cellSize.z = params::cellSizeXY; // m
    State::cellSize.yaw_count = 16; // yaw angles
    State::cellSize.yaw_count_half = State::cellSize.yaw_count / 2;
    State::cellSize.yaw = DEG2RAD(360.0/(double)State::cellSize.yaw_count);
    State::cellSize.yaw_half = State::cellSize.yaw / 2.0;
    forEach (const tf::Point& point, checker->getBaseFootprintPolygon())
    {
        footprintPolygon.push_back(sbpl_2Dpt_t(point.x(), point.y()));
    }
    actionwidth = 0;

    this->heuristicPlanner = heuristicPlanner;
    this->heuristicEnvironment = heuristicEnvironment;

    ROS_INFO("%s ends", __PRETTY_FUNCTION__);
}

PlanningEnvironmentODSH::~PlanningEnvironmentODSH()
{
    startState = NULL;
    goalState = NULL;
    for (IdStateMap::iterator stateIt = idToState.begin(); stateIt != idToState.end(); stateIt++)
    {
        delete stateIt->second;
    }
}

void PlanningEnvironmentODSH::readMotionPrimitives(const std::string& file)
{
    FILE* fMotPrims = fopen(file.c_str(), "r");
    if(fMotPrims == NULL)
    {
        SBPL_ERROR("ERROR: unable to open %s\n", file.c_str());
        throw new SBPL_Exception();
    }

    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    SBPL_PRINTF("Reading in motion primitives...");

    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0)
        throw new SBPL_Exception();
    if (strcmp(sTemp, sExpected) != 0)
    {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0)
        throw new SBPL_Exception();
    double resolutionCorrectionFactor = params::motionPrimitiveScaleFactor;
    if (fabs(fTemp - params::motionPrimitiveResolution) > ERR_EPS)
    {
        resolutionCorrectionFactor = params::motionPrimitiveResolution*params::motionPrimitiveScaleFactor / fTemp;
        SBPL_INFO("resolution %f (instead of %f) in the dynamics file; scaling numerical values by %0.3f.\n", fTemp, State::cellSize.xy, resolutionCorrectionFactor);
    }

    //read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0)
        throw new SBPL_Exception();
    if (strcmp(sTemp, sExpected) != 0)
    {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0)
        throw new SBPL_Exception();
    if (dTemp != State::cellSize.yaw_count)
    {
        SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n", dTemp, State::cellSize.yaw_count);
        throw new SBPL_Exception();
    }

    //read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0)
        throw new SBPL_Exception();
    if (strcmp(sTemp, sExpected) != 0)
    {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0)
    {
        throw new SBPL_Exception();
    }

    for (int i = 0; i < totalNumofActions; i++)
    {
        SBPL_xytheta_mprimitive motprim;

        if (readinMotionPrimitive(&motprim, fMotPrims, resolutionCorrectionFactor) == false)
            throw new SBPL_Exception();

        motionPrimitives.push_back(motprim);

    }
    fclose(fMotPrims);
    precomputeActionswithCompleteMotionPrimitive(&motionPrimitives);
}

bool PlanningEnvironmentODSH::readinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim, FILE* fIn, double resolutionCorrectionFactor)
{
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;

    //read in actionID
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if (strcmp(sTemp, sExpected) != 0)
    {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1)
        return false;

    //read in start angle
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if (strcmp(sTemp, sExpected) != 0)
    {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0)
    {
        SBPL_ERROR("ERROR reading start angle\n");
        return false;
    }
    pMotPrim->starttheta_c = dTemp;

    //read in end pose
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if (strcmp(sTemp, sExpected) != 0)
    {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }

    if (readinCell(&pMotPrim->endcell, fIn) == false)
    {
        SBPL_ERROR("ERROR: failed to read in end search pose\n");
        return false;
    }

    //read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if (strcmp(sTemp, sExpected) != 0)
    {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1)
        return false;
    pMotPrim->additionalactioncostmult = dTemp;

    //read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    if (strcmp(sTemp, sExpected) != 0)
    {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1)
        return false;
    //all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done
    //after the action is rotated by initial orientation
    for (int i = 0; i < numofIntermPoses; i++)
    {
        sbpl_xy_theta_pt_t intermpose;
        if (readinPose(&intermpose, fIn, resolutionCorrectionFactor) == false)
        {
            SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
            return false;
        }
        pMotPrim->intermptV.push_back(intermpose);
    }

    //check that the last pose corresponds correctly to the last pose
    pMotPrim->endcell.x *= params::motionPrimitiveScaleFactor;
    pMotPrim->endcell.y *= params::motionPrimitiveScaleFactor;
    sbpl_xy_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, params::motionPrimitiveResolution);
    sourcepose.y = DISCXY2CONT(0, params::motionPrimitiveResolution);
    sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, State::cellSize.yaw_count);
    double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;
    int endx_c = CONTXY2DISC(mp_endx_m, params::motionPrimitiveResolution);
    int endy_c = CONTXY2DISC(mp_endy_m, params::motionPrimitiveResolution);
    int endtheta_c = ContTheta2Disc(mp_endtheta_rad, State::cellSize.yaw_count);
    if (endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta)
    {
        SBPL_ERROR("ERROR: incorrect primitive %d with startangle=%d last interm point %f %f %f does not match end pose %d %d %d\n", pMotPrim->motprimID, pMotPrim->starttheta_c, pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x, pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y,
                pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta, pMotPrim->endcell.x, pMotPrim->endcell.y, pMotPrim->endcell.theta);
        return false;
    }
    pMotPrim->endcell.x /= params::motionPrimitiveScaleFactor;
    pMotPrim->endcell.y /= params::motionPrimitiveScaleFactor;

    return true;
}
bool PlanningEnvironmentODSH::readinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    cell->x = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    cell->y = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    cell->theta = atoi(sTemp);

    //normalize the angle
    cell->theta = NORMALIZEDISCTHETA(cell->theta, State::cellSize.yaw_count);

    return true;
}

bool PlanningEnvironmentODSH::readinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn, double resolutionCorrectionFactor)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    pose->x = atof(sTemp) * resolutionCorrectionFactor;
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    pose->y = atof(sTemp) * resolutionCorrectionFactor;
    if (fscanf(fIn, "%s", sTemp) == 0)
        return false;
    pose->theta = atof(sTemp);
    pose->theta = normalizeAngle(pose->theta);
    return true;
}

//here motionprimitivevector contains actions for all angles
void PlanningEnvironmentODSH::precomputeActionswithCompleteMotionPrimitive(std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
    SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
    std::vector<sbpl_2Dcell_t> footprint;

    if (motionprimitiveV->size() % State::cellSize.yaw_count != 0) {
        SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
        throw new SBPL_Exception();
    }

    //iterate over source angles
    int maxnumofactions = 0;
    for (int counter = 0; counter < State::cellSize.yaw_count; counter++) {
        int yaw_index = State::normalizeDiscreteYaw(counter);
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", yaw_index, State::cellSize.yaw_count);

//        actions[yaw_index].resize(motionprimitiveV->size());

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, State::cellSize.xy);
        sourcepose.y = DISCXY2CONT(0, State::cellSize.xy);
        sourcepose.theta = State::discreteYawToRad(yaw_index);

        //iterate over motion primitives
        int numofactions = 0;
        int action_index = -1;
        for (int motion_index = 0; motion_index < (int)motionprimitiveV->size(); motion_index++) {
            //find a motion primitive for this angle
            if (State::normalizeDiscreteYaw(motionprimitiveV->at(motion_index).starttheta_c) != yaw_index)
                continue;

            actions[yaw_index].push_back(Action());
            Action& action = actions[yaw_index].back();
            EnvNAVXYTHETALATAction_t& discreteAction = action.discreteAction;
            action_index = actions[yaw_index].size() - 1;
            numofactions++;

            //action index
            discreteAction.aind = action_index;

            //start angle
            discreteAction.starttheta = yaw_index;

            //compute dislocation
            discreteAction.endtheta = motionprimitiveV->at(motion_index).endcell.theta;
            discreteAction.dX = motionprimitiveV->at(motion_index).endcell.x;
            discreteAction.dY = motionprimitiveV->at(motion_index).endcell.y;

            //compute and store interm points as well as intersecting cells
            discreteAction.intersectingcellsV.clear();
            discreteAction.intermptV.clear();
            discreteAction.interm3DcellsV.clear();

            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.x = 0;
            previnterm3Dcell.y = 0;

            // Compute all the intersected cells for this action (intermptV and interm3DcellsV)
            for (int pind = 0; pind < (int)motionprimitiveV->at(motion_index).intermptV.size(); pind++)
            {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(motion_index).intermptV[pind];
                discreteAction.intermptV.push_back(intermpt);

                // also compute the intermediate discrete cells if not there already
                sbpl_xy_theta_pt_t pose;
                pose.x = intermpt.x + sourcepose.x;
                pose.y = intermpt.y + sourcepose.y;
                pose.theta = intermpt.theta;

                sbpl_xy_theta_cell_t intermediate2dCell;
                intermediate2dCell.x = State::meterToDiscreteCell(pose.x, State::cellSize.xy);
                intermediate2dCell.y = State::meterToDiscreteCell(pose.y, State::cellSize.xy);

                // add unique cells to the list
                if(discreteAction.interm3DcellsV.size() == 0
                        || intermediate2dCell.x != previnterm3Dcell.x
                        || intermediate2dCell.y != previnterm3Dcell.y)
                {
                    discreteAction.interm3DcellsV.push_back(intermediate2dCell);
                }

                previnterm3Dcell = intermediate2dCell;
            }

            //compute linear and angular time
            double linear_distance = 0;
            for (unsigned int i = 1; i < discreteAction.intermptV.size(); i++) {
                double x0 = discreteAction.intermptV[i - 1].x;
                double y0 = discreteAction.intermptV[i - 1].y;
                double x1 = discreteAction.intermptV[i].x;
                double y1 = discreteAction.intermptV[i].y;
                double dx = x1 - x0;
                double dy = y1 - y0;
                linear_distance += sqrt(dx * dx + dy * dy);
            }
            double linear_time = linear_distance / params::linearVelocity;
            double angular_distance = State::discreteYawToRad(State::discreteYawDistance(discreteAction.endtheta, discreteAction.starttheta));
            double angular_time = angular_distance / params::angularVelocity;
            //make the cost the max of the two times
            discreteAction.cost = (int)(ceil(100 * fmax(linear_time, angular_time)));
            //use any additional cost multiplier
            discreteAction.cost *= motionprimitiveV->at(motion_index).additionalactioncostmult;

            //now compute the intersecting cells for this motion (including ignoring the source footprint)
            get_2d_motion_cells(footprintPolygon, motionprimitiveV->at(motion_index).intermptV, &discreteAction.intersectingcellsV, params::motionPrimitiveResolution);
            std::set<sbpl_2Dcell_t> initialCells;
            get_2d_footprint_cells(footprintPolygon, &initialCells, motionprimitiveV->at(motion_index).intermptV.front(), params::motionPrimitiveResolution);
            std::set<sbpl_2Dcell_t> finalCells;
            get_2d_footprint_cells(footprintPolygon, &finalCells, motionprimitiveV->at(motion_index).intermptV.back(), params::motionPrimitiveResolution);
            action.updateFromDiscretePoses();

#if DEBUG
            SBPL_FPRINTF(
                    fDeb,
                    "action tind=%2d aind=%2d: dX=%3d dY=%3d endtheta=%3d (%6.2f degs -> %6.2f degs) cost=%4d (mprimID %3d: %3d %3d %3d) numofintermcells = %d numofintercells=%d\n",
                    yaw_index,
                    action_index,
                    discreteAction.dX,
                    discreteAction.dY,
                    discreteAction.endtheta,
                    discreteAction.intermptV[0].theta * 180 / PI_CONST,
                    discreteAction.intermptV[discreteAction.intermptV.size() - 1].theta * 180 / PI_CONST, discreteAction.cost,
                    motionprimitiveV->at(motion_index).motprimID, motionprimitiveV->at(motion_index).endcell.xy,
                    motionprimitiveV->at(motion_index).endcell.y, motionprimitiveV->at(motion_index).endcell.theta,
                    (int)discreteAction.interm3DcellsV.size(),
                    (int)discreteAction.intersectingcellsV.size());
#endif

            //add to the list of backward actions
            int targettheta = discreteAction.endtheta;
            predecessorActions[targettheta].push_back(&action);

        }

        if (maxnumofactions < numofactions)
            maxnumofactions = numofactions;
    }

    //at this point we don't allow nonuniform number of actions
    if (motionprimitiveV->size() != (size_t)(State::cellSize.yaw_count * maxnumofactions)) {
        SBPL_ERROR("ERROR: nonuniform number of actions is not supported (maxnumofactions=%d while motprims=%d thetas=%d\n",
                maxnumofactions,
                (unsigned int)motionprimitiveV->size(),
                State::cellSize.yaw_count);
        throw new SBPL_Exception();
    }
    actionwidth = maxnumofactions;

    //now compute replanning data
//    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}

bool PlanningEnvironmentODSH::initializeSearch(tf::Pose start, tf::Pose goal)
{
    ROS_INFO("%s begins", __PRETTY_FUNCTION__);
    if (! checker->projectPoseOnDrivableSurface(start))
    {
        ROS_ERROR("%s: start not on drivable ground", __PRETTY_FUNCTION__);
        PrintPose(start);
        return false;
    }
    KeyCellDataMap startEdgeCells;
    Cost startCost = checker->initializeStartState(start, startEdgeCells);
    if (startCost.total >= params::infiniteCosts)
    {
        ROS_ERROR("%s: start pose invalid", __PRETTY_FUNCTION__);
        PrintPose(start);
        return false;
    }
    if (! checker->projectPoseOnDrivableSurface(goal))
    {
        ROS_ERROR("%s: goal not on drivable ground", __PRETTY_FUNCTION__);
        PrintPose(goal);
        return false;
    }
    KeyCellDataMap goalEdgeCells;
    Cost goalCost = checker->initializeStartState(goal, goalEdgeCells);
    if (goalCost.total >= params::infiniteCosts)
    {
        ROS_ERROR("%s: goal pose invalid", __PRETTY_FUNCTION__);
        PrintPose(goal);
        return false;
    }
    expandedIds.clear();
    startState = findOrCreateState(start);
    startState->footprint = startEdgeCells;
    goalState = findOrCreateState(goal);
    goalState->footprint = goalEdgeCells;
    heuristicEnvironment->createStartState(goal);
    heuristicPlanner->set_start(heuristicEnvironment->getStartId());
    ROS_INFO("%s ends", __PRETTY_FUNCTION__);
    return true;
}

bool PlanningEnvironmentODSH::convertPath(const std::vector<int>& pathIds, std::vector<tf::Pose>& pathPoses, std::vector<Cost>* pathCosts) const
{
    pathPoses.reserve(pathIds.size());
    std::vector<int>::const_iterator previousIdIt = pathIds.end();
    for (std::vector<int>::const_iterator idIt = pathIds.begin();
            idIt != pathIds.end(); idIt++)
    {
        pathPoses.push_back(tf::Pose());
        if (! convertState(*idIt, &pathPoses.back()))
            return false;
        if (previousIdIt != pathIds.end() && pathCosts != NULL)
        {
            TransitionCostMap::const_iterator costIt = costs.find(std::make_pair(*previousIdIt, *idIt));
            if (costIt != costs.end())
            {
                pathCosts->push_back(costIt->second);
            }
        }
        previousIdIt = idIt;
    }

    return true;
}
bool PlanningEnvironmentODSH::convertState(int stateId, tf::Pose* statePose) const
{
    IdStateMap::const_iterator stateIt = idToState.find(stateId);
    if (stateIt == idToState.end())
    {
        ROS_ERROR("%s: state information for id %d is missing.", __PRETTY_FUNCTION__, stateId);
        return false;
    }
    *statePose = stateIt->second->pose;
    // Hack: for visualization: put yaw on z coordinate
    tf::Point& point = statePose->getOrigin();
    point.setZ(point.z() + 0.01 * stateIt->second->yaw);
    return true;
}

bool PlanningEnvironmentODSH::InitializeEnv(const char* sEnvFile)
{
    ROS_FATAL("%s not supported.", __PRETTY_FUNCTION__);
    return false;
}

/**
 * \brief initialization of MDP data structure
 */
bool PlanningEnvironmentODSH::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    MDPCfg->goalstateid = goalState->id;
    MDPCfg->startstateid = startState->id;
    return true;
}

/**
 * \brief heuristic estimate from state FromStateID to state ToStateID
 */
int PlanningEnvironmentODSH::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    return getFromToHeuristic(idToState.find(FromStateID)->second, idToState.find(FromStateID)->second);
}

/**
 * \brief heuristic estimate from state with stateID to goal state
 */
int PlanningEnvironmentODSH::GetGoalHeuristic(int stateID)
{
    return getFromToHeuristic(idToState.find(stateID)->second, goalState);
}

/**
 * \brief heuristic estimate from start state to state with stateID
 */
int PlanningEnvironmentODSH::GetStartHeuristic(int stateID)
{
    return getFromToHeuristic(startState, idToState.find(stateID)->second);
}

int PlanningEnvironmentODSH::getFromToHeuristic(const State* fromState, const State* toState)
{
    int heuristicValue = params::infiniteCosts;
    static int heuristic_requested = 0;
    heuristic_requested++;
    tf::Pose pose = fromState->pose;
    if (checker->projectPoseOnDrivableSurface(pose))
    if (! heuristicEnvironment->getPathCostTo(pose, &heuristicValue))
    {
        static int heuristic_searched = 0;
        heuristic_searched++;
        heuristicEnvironment->createGoalState(pose);
//        if (! checker->projectPoseOnDrivableSurface(pose))
//        {
//            checker->visualizeCollision(pose, "heuristic collision");
//        }
        heuristicPlanner->set_goal(heuristicEnvironment->getGoalId());
        std::vector<int> solutionIds;
        int result = heuristicPlanner->replan(5.0, &solutionIds);
        if (heuristicEnvironment->getPathCostTo(pose, &heuristicValue))
        {
//            ROS_INFO("heuristic searched %d of %d. value: %d", heuristic_searched, heuristic_requested, heuristicValue);
        }
        else
        {
            checker->visualizeCollision(fromState->pose, "heuristic collision");
            ROS_WARN("heuristic computation fail! planner result: %d", result);
            PrintPose(pose);
        }
    }
    return heuristicValue / sqrt(2);
}

State* PlanningEnvironmentODSH::findOrCreateState(const tf::Pose& pose)
{
//    ROS_INFO("%s", __PRETTY_FUNCTION__);
    State* newState = new State(pose);
    StateIntMap::iterator oldStateIt = stateToId.find(newState);
    if (oldStateIt != stateToId.end())
    {
        // we have been here, get old state
        delete newState;
        newState = oldStateIt->first;
    }
    else
    {
        // new state, set id
        newState->id = stateToId.size();
        idToState.insert(std::make_pair(newState->id, newState));
        stateToId.insert(std::make_pair(newState, newState->id));

        // following lines copied from sbpl_3dnav_planner... no idea what they do, but we need them
        //insert into and initialize the mappings
        int* entry = new int [NUMOFINDICES_STATEID2IND];
        StateID2IndexMapping.push_back(entry);
        for(int i = 0; i < NUMOFINDICES_STATEID2IND; i++)
          StateID2IndexMapping[newState->id][i] = -1;
        if(newState->id != (int)StateID2IndexMapping.size()-1)
        {
          ROS_ERROR("ERROR in Env... function: last state has incorrect stateID");
          throw new SBPL_Exception();
        }
    }
//    if (goalState != NULL)
//    {
//        if (State::xyzDistance(newState->x - goalState->x, newState->y - goalState->y, 0) < 0.1)
//        {
//            ROS_INFO("close to goal:");
//            PrintState(newState);
//            PrintState(goalState);
//        }
//    }
    return newState;
}

/** \brief depending on the search used, it may call GetSuccs function
 *         (for forward search) or GetPreds function (for backward search)
 *         or both (for incremental search). At least one of this functions should
 *         be implemented (otherwise, there will be no search to run) Some searches
 *         may also use SetAllActionsandAllOutcomes or SetAllPreds functions if they
 *         keep the pointers to successors (predecessors) but most searches do not
 *         require this, so it is not necessary to support this
 */
void PlanningEnvironmentODSH::GetSuccs(int SourceStateID, std::vector<int>* successorIds, std::vector<int>* costs)
{
    successorIds->clear();
    costs->clear();

    expandedIds.push_back(SourceStateID);

    // goal state should be absorbing
    if(SourceStateID == goalState->id)
      return;

    // get current state
    State* current = idToState.find(SourceStateID)->second;

//    checker->visualizePose(current->pose);

    // for each motion primitive:
    forEach(const Action& action, actions[current->yaw])
    {
        const EnvNAVXYTHETALATAction_t& discreteAction = action.discreteAction;

        // expand
        int newX = current->x + discreteAction.dX;
        int newY = current->y + discreteAction.dY;
        int newTheta = State::normalizeDiscreteYaw(discreteAction.endtheta);

        // check collisions and surface segment transitions
        tf::Pose newPose = State::discreteToPose(newX, newY, current->z, newTheta);
//        PrintPose(current->pose);
//        PrintPose(newPose);
        KeyCellDataMap newFootprint;
//        if (action.discreteAction.aind == 1)
//        Cost motionCost = checker->debugMotionCosts(current->pose,
//                current->footprint,
//                &newPose, newFootprint,
//                action.intersectingCells,
//                action.discreteAction.cost, "action_"+boost::lexical_cast<std::string>(action.discreteAction.aind));

        Cost motionCost = checker->getMotionCosts(current->pose,
                current->footprint,
                &newPose, newFootprint,
                action.intersectingCells,
                action.discreteAction.cost);
        if (motionCost.total >= params::infiniteCosts)
        {
            continue;
        }

        // create new state
        State* newState = findOrCreateState(newPose);
        newState->footprint = newFootprint;
        successorIds->push_back(newState->id);
        // compute costs
        costs->push_back(motionCost.total);
        this->costs.insert(std::make_pair(std::make_pair(SourceStateID, newState->id), motionCost));
    }
}

/**
 * \brief see comments for GetSuccs functon
 */
void PlanningEnvironmentODSH::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV)
{
    ROS_FATAL("%s not supported.", __PRETTY_FUNCTION__);
}

/**
 * \brief see comments for GetSuccs functon
 */
void PlanningEnvironmentODSH::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    ROS_FATAL("%s not supported.", __PRETTY_FUNCTION__);
}

/**
 * \brief see comments for GetSuccs functon
 */
void PlanningEnvironmentODSH::SetAllPreds(CMDPSTATE* state)
{
    ROS_FATAL("%s not supported.", __PRETTY_FUNCTION__);
}

/**
 * \brief returns the number of states (hashentries) created
 */
int PlanningEnvironmentODSH::SizeofCreatedEnv()
{
    return idToState.size();
}

/**
 * \brief prints the state variables for a state with stateID
 */
void PlanningEnvironmentODSH::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    IdStateMap::const_iterator stateIt = idToState.find(stateID);
    if (stateIt != idToState.end())
        PrintState(stateIt->second, bVerbose, fOut);
}

void PlanningEnvironmentODSH::PrintState(const State* state, bool bVerbose, FILE* fOut)
{
    ROS_INFO("%d [%d %d %d %d]", state->id, state->x, state->y, state->z, state->yaw);
//    PrintPose(state->pose);
}
void PlanningEnvironmentODSH::PrintPose(const tf::Pose& pose)
{
    tf::Vector3 position = pose.getOrigin();
    ROS_INFO("  [%0.3f %0.3f %0.3f] %0.3f Deg", position.x(), position.y(), position.z(), RAD2DEG(tf::getYaw(pose.getRotation())));
}
/**
 * \brief prints environment config file
 */
void PlanningEnvironmentODSH::PrintEnv_Config(FILE* fOut)
{
    ROS_INFO("%s begins", __PRETTY_FUNCTION__);
    ROS_INFO("Planning Environment:");
    ROS_INFO("cell size: [%0.3f, %0.3f %0.3f] %0.3f Deg", State::cellSize.xy, State::cellSize.xy, State::cellSize.z, RAD2DEG(State::cellSize.yaw));
    ROS_INFO("start state:");
    PrintState(startState, false);
    ROS_INFO("goal state:");
    PrintState(goalState, false);
    ROS_INFO("state size: %d", SizeofCreatedEnv());
    ROS_INFO("%s ends", __PRETTY_FUNCTION__);
}

void PlanningEnvironmentODSH::testStateDiscretization(const tf::Pose& pose, std::vector<tf::Pose>& expanded)
{
    tf::Pose transformYaw(tf::createQuaternionFromYaw(State::cellSize.yaw), tf::Point(0,0,0));
//    tf::Pose transformX(tf::createIdentityQuaternion(), tf::Point(State::cellSize.xy,0,0));
//    tf::Pose transformY(tf::createIdentityQuaternion(), tf::Point(0, State::cellSize.xy,0));
//    tf::Pose newPoseX = pose;
//    for(int x = 0; x < 50; x++)
//    {
//        newPoseX = newPoseX * transformX;
//        tf::Pose newPoseY = newPoseX;
//        for(int y = 0; y < 50; y++)
//        {
//            newPoseY = newPoseY * transformY;
//            tf::Pose newPoseYaw = newPoseY;
//            for(int i = 0; i < State::cellSize.yaw_count; i++)
//            {
//                newPoseYaw = newPoseYaw * transformYaw;
//                State* state = findOrCreateState(newPoseYaw);
//                expanded.push_back(tf::Pose());
//                convertState(state->id, &expanded.back());
//                PrintState(state, true, NULL);
//            }
//        }
//    }
    std::vector<int> allSuccessors;
    allSuccessors.push_back(startState->id);
    std::vector<int> successors;
    std::vector<int> costs;
    tf::Pose newPoseYaw = pose;
    for(int i = 0; i <  State::cellSize.yaw_count; i++)
    {
        if (i == 0)
        {
            State* state = findOrCreateState(newPoseYaw);
            GetSuccs(state->id, &successors, &costs);
            allSuccessors.insert(allSuccessors.end(), successors.begin(), successors.end());
        }
        newPoseYaw = newPoseYaw * transformYaw;
    }
    convertPath(allSuccessors, expanded);
}

} /* namespace gki_3dnav_planner */
