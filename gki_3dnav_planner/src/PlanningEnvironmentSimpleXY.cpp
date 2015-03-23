/*
 * PlanningEnvironment.cpp
 *
 *  Created on: 28 Feb 2013
 *      Author: andreas
 */

#include "gki_3dnav_planner/PlanningEnvironmentSimpleXY.h"
#include "gki_3dnav_planner/Params.h"
#include <ros/assert.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/utils.h>
#include <gki_utils/misc.h>

#define NUMOFINDICES_STATEID2IND 2

namespace gki_3dnav_planner
{

PlanningEnvironmentSimpleXY::PlanningEnvironmentSimpleXY(const CollisionChecker* checker, bool trackPathCosts)
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
    actionwidth = 0;
    this->trackPathCosts = trackPathCosts;
    ROS_INFO("%s ends", __PRETTY_FUNCTION__);
}

PlanningEnvironmentSimpleXY::~PlanningEnvironmentSimpleXY()
{
    startState = NULL;
    goalState = NULL;
    for (IdStateMap::iterator stateIt = idToState.begin(); stateIt != idToState.end(); stateIt++)
    {
        delete stateIt->second;
    }
}

void PlanningEnvironmentSimpleXY::generateActions()
{
    std::vector<sbpl_2Dcell_t> expansions;
    expansions.push_back(sbpl_2Dcell_t(1, 0));
    expansions.push_back(sbpl_2Dcell_t(-1, 0));
    expansions.push_back(sbpl_2Dcell_t(0, 1));
    expansions.push_back(sbpl_2Dcell_t(0, -1));
    expansions.push_back(sbpl_2Dcell_t(1, 1));
    expansions.push_back(sbpl_2Dcell_t(-1, 1));
    expansions.push_back(sbpl_2Dcell_t(-1, -1));
    expansions.push_back(sbpl_2Dcell_t(1, -1));

    sbpl_xy_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, State::cellSize.xy);
    sourcepose.y = DISCXY2CONT(0, State::cellSize.xy);
    sourcepose.theta = State::discreteYawToRad(0);
    forEach(const sbpl_2Dcell_t& expansion, expansions)
    {
        actions.push_back(Action());
        Action& action = actions.back();
        EnvNAVXYTHETALATAction_t& discreteAction = action.discreteAction;
        sbpl_xy_theta_pt_t destinationPose;
        destinationPose.x = DISCXY2CONT(expansion.x, State::cellSize.xy);
        destinationPose.y = DISCXY2CONT(expansion.y, State::cellSize.xy);
        destinationPose.theta = State::discreteYawToRad(0);

        //compute dislocation
        discreteAction.endtheta = 0;
        discreteAction.dX = expansion.x;
        discreteAction.dY = expansion.y;

        double linear_distance = hypot(sourcepose.x - destinationPose.x, sourcepose.y - destinationPose.y);
        double linear_time = linear_distance / params::linearVelocity;
        discreteAction.cost = (int)(ceil(1000 * linear_time));

        std::vector<sbpl_xy_theta_pt_t> intermediatePoses;
        intermediatePoses.push_back(sourcepose);
        intermediatePoses.push_back(destinationPose);
        action.updateFromDiscretePoses();
    }
}

bool PlanningEnvironmentSimpleXY::initializeSearch(tf::Pose start, tf::Pose goal)
{
//    computingHeuristic = computeHeuristic;
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
    createStartState(start);
    startState->footprint = startEdgeCells;
    createGoalState(goal);
    goalState->footprint = goalEdgeCells;
    ROS_INFO("initial heuristic value: %d", getFromToHeuristic(startState, goalState));
    return true;
}

bool PlanningEnvironmentSimpleXY::convertPath(const std::vector<int>& pathIds, std::vector<tf::Pose>& pathPoses, std::vector<Cost>* pathCosts) const
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
bool PlanningEnvironmentSimpleXY::convertState(int stateId, tf::Pose* statePose) const
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

bool PlanningEnvironmentSimpleXY::InitializeEnv(const char* sEnvFile)
{
    ROS_FATAL("%s not supported.", __PRETTY_FUNCTION__);
    return false;
}

/**
 * \brief initialization of MDP data structure
 */
bool PlanningEnvironmentSimpleXY::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    MDPCfg->goalstateid = goalState->id;
    MDPCfg->startstateid = startState->id;
    return true;
}

/**
 * \brief heuristic estimate from state FromStateID to state ToStateID
 */
int PlanningEnvironmentSimpleXY::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    return getFromToHeuristic(idToState.find(FromStateID)->second, idToState.find(FromStateID)->second);
}

/**
 * \brief heuristic estimate from state with stateID to goal state
 */
int PlanningEnvironmentSimpleXY::GetGoalHeuristic(int stateID)
{
    return getFromToHeuristic(idToState.find(stateID)->second, goalState);
}

/**
 * \brief heuristic estimate from start state to state with stateID
 */
int PlanningEnvironmentSimpleXY::GetStartHeuristic(int stateID)
{
    return getFromToHeuristic(startState, idToState.find(stateID)->second);
}

int PlanningEnvironmentSimpleXY::getFromToHeuristic(const State* fromState, const State* toState)
{
    double distance = hypot(fromState->pose.getOrigin().x() - toState->pose.getOrigin().x(),
            fromState->pose.getOrigin().y() - toState->pose.getOrigin().y());
    return ceil(1000 * (distance / params::linearVelocity));
}

bool PlanningEnvironmentSimpleXY::getPathCostTo(const tf::Pose& pose, int* cost)
{
//    ROS_INFO("%s", __PRETTY_FUNCTION__);
    State* state = findOrCreateState(pose);
    StateIntMap::const_iterator it = pathCostTable.find(state);
    if (it != pathCostTable.end())
    {
        *cost = it->second;
        return true;
    }
    return false;
}

void PlanningEnvironmentSimpleXY::updatePathCost(State* state, int cost)
{
//    ROS_INFO("%s", __PRETTY_FUNCTION__);
    StateIntMap::const_iterator it = pathCostTable.find(state);
    if (it != pathCostTable.end())
    {
        // entry for state exists
        if (it->second < cost)
        {
            return;
        }
    }
    pathCostTable.insert(std::make_pair(state, cost));
}

State* PlanningEnvironmentSimpleXY::findOrCreateState(const tf::Pose& pose)
{
//    ROS_INFO("%s", __PRETTY_FUNCTION__);
    State* newState = new State(pose);
    newState->yaw = 0;
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
//        int heuristicValue = getFromToHeuristic(newState, goalState);
//        if (heuristicValue < bestHeuristicValue)
//        {
//            bestHeuristicValue = heuristicValue;
//    //            ROS_INFO("best heuristic value: %d", bestHeuristicValue);
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
void PlanningEnvironmentSimpleXY::GetSuccs(int SourceStateID, std::vector<int>* successorIds, std::vector<int>* costs)
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

    forEach(const Action& action, actions)
    {
        int newX = current->x + action.discreteAction.dX;
        int newY = current->y + action.discreteAction.dY;
        tf::Pose newPose = State::discreteToPose(newX, newY, current->z, 0);
        KeyCellDataMap newFootprint;
        Cost motionCost = checker->getHeuristicCosts(current->pose,
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

        this->costs.insert(std::make_pair(std::make_pair(SourceStateID, newState->id), motionCost));
        if (trackPathCosts)
        {
            costs->push_back(motionCost.total);
            StateIntMap::const_iterator it = pathCostTable.find(current);
            ROS_ASSERT_MSG(it != pathCostTable.end(), "heuristic lookup failed.");
            updatePathCost(newState, it->second + motionCost.total);
        }
        else
        {
            costs->push_back(motionCost.action);
        }
    }
}

/**
 * \brief see comments for GetSuccs functon
 */
void PlanningEnvironmentSimpleXY::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV)
{
    ROS_FATAL("%s not supported.", __PRETTY_FUNCTION__);
}

/**
 * \brief see comments for GetSuccs functon
 */
void PlanningEnvironmentSimpleXY::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    ROS_FATAL("%s not supported.", __PRETTY_FUNCTION__);
}

/**
 * \brief see comments for GetSuccs functon
 */
void PlanningEnvironmentSimpleXY::SetAllPreds(CMDPSTATE* state)
{
    ROS_FATAL("%s not supported.", __PRETTY_FUNCTION__);
}

/**
 * \brief returns the number of states (hashentries) created
 */
int PlanningEnvironmentSimpleXY::SizeofCreatedEnv()
{
    return idToState.size();
}

/**
 * \brief prints the state variables for a state with stateID
 */
void PlanningEnvironmentSimpleXY::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    return;
    IdStateMap::const_iterator stateIt = idToState.find(stateID);
    if (stateIt != idToState.end())
        PrintState(stateIt->second, bVerbose, fOut);
}

void PlanningEnvironmentSimpleXY::PrintState(const State* state, bool bVerbose, FILE* fOut)
{
    ROS_INFO("%d [%d %d %d %d]", state->id, state->x, state->y, state->z, state->yaw);
//    PrintPose(state->pose);
}
void PlanningEnvironmentSimpleXY::PrintPose(const tf::Pose& pose)
{
    tf::Vector3 position = pose.getOrigin();
    ROS_INFO("  [%0.3f %0.3f %0.3f] %0.3f Deg", position.x(), position.y(), position.z(), RAD2DEG(tf::getYaw(pose.getRotation())));
}
/**
 * \brief prints environment config file
 */
void PlanningEnvironmentSimpleXY::PrintEnv_Config(FILE* fOut)
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

} /* namespace gki_3dnav_planner */
