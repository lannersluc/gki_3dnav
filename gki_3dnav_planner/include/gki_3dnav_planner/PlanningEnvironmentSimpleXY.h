/*
 * PlanningEnvironment.h
 *
 *  Created on: 28 Feb 2013
 *      Author: andreas
 */

#ifndef TERRAINMAPPLANNINGENVIRONMENT_SIMPLEXY_H_
#define TERRAINMAPPLANNINGENVIRONMENT_SIMPLEXY_H_

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <map>
#include <tr1/unordered_set>
#include <list>
#include <tf/tf.h>
#include "gki_3dnav_planner/CollisionChecker.h"
#include "gki_3dnav_planner/Params.h"
#include "gki_3dnav_planner/PlanningEnvironmentCommon.h"

namespace gki_3dnav_planner
{

class PlanningEnvironmentSimpleXY : public PlanningEnvironment
{
public:
    PlanningEnvironmentSimpleXY(const CollisionChecker* checker, bool trackPathCosts = false);
    virtual ~PlanningEnvironmentSimpleXY();

    bool initializeSearch(tf::Pose start, tf::Pose goal);

    virtual bool InitializeEnv(const char* sEnvFile);

    /**
     * \brief initialization of MDP data structure
     */
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);

    /**
     * \brief heuristic estimate from state FromStateID to state ToStateID
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief heuristic estimate from state with stateID to goal state
     */
    virtual int GetGoalHeuristic(int stateID);

    /**
     * \brief heuristic estimate from start state to state with stateID
     */
    virtual int GetStartHeuristic(int stateID);

    /** \brief depending on the search used, it may call GetSuccs function
     *         (for forward search) or GetPreds function (for backward search)
     *         or both (for incremental search). At least one of this functions should
     *         be implemented (otherwise, there will be no search to run) Some searches
     *         may also use SetAllActionsandAllOutcomes or SetAllPreds functions if they
     *         keep the pointers to successors (predecessors) but most searches do not
     *         require this, so it is not necessary to support this
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void SetAllPreds(CMDPSTATE* state);

    /**
     * \brief returns the number of states (hashentries) created
     */
    virtual int SizeofCreatedEnv();

    /**
     * \brief prints the state variables for a state with stateID
     */
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);
    void PrintState(const State* state, bool bVerbose = true, FILE* fOut = NULL);
    void PrintPose(const tf::Pose& pose);
    /**
     * \brief prints environment config file
     */
    virtual void PrintEnv_Config(FILE* fOut);
    int getFromToHeuristic(const State* fromState, const State* toState);
    bool convertPath(const std::vector<int>& pathIds, std::vector<tf::Pose>& pathPoses, std::vector<Cost>* pathCosts = NULL) const;
    bool convertState(int pathId, tf::Pose* pathPose) const;
    int getStartId() const {if (startState != NULL) return startState->id; return 0;}
    int getGoalId() const {if (goalState != NULL) return goalState->id; return 0;}
    const std::vector<int> getExpandedIds() const {return expandedIds;}

    void generateActions();
    void createStartState(const tf::Pose& pose) {startState = findOrCreateState(pose); if(trackPathCosts)updatePathCost(startState, 0);}
    void createGoalState(const tf::Pose& pose) {goalState = findOrCreateState(pose);}
    bool getPathCostTo(const tf::Pose& pose, int* cost);
private:
    State* findOrCreateState(const tf::Pose& pose);
    void updatePathCost(State* state, int cost);

    State* goalState;
    State* startState;
    IdStateMap idToState;
    StateIntMap stateToId;

    int actionwidth;
    std::vector<Action> actions;

    const CollisionChecker* checker;

    std::vector<int> expandedIds;
    TransitionCostMap costs;

    StateIntMap pathCostTable;
    bool trackPathCosts;

};

} /* namespace gki_3dnav_planner */
#endif /* TERRAINMAPPLANNINGENVIRONMENT_SIMPLEXY_H_ */
