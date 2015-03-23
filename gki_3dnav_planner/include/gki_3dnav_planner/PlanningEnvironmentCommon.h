/*
 * PlanningEnvironmentCommon.h
 *
 *  Created on: 4 Apr 2013
 *      Author: andreas
 */

#ifndef PLANNINGENVIRONMENTCOMMON_H_
#define PLANNINGENVIRONMENTCOMMON_H_

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <map>
#include <tr1/unordered_set>
#include <list>
#include <tf/tf.h>
#include "gki_3dnav_planner/CollisionChecker.h"
#include "gki_3dnav_planner/Params.h"

namespace gki_3dnav_planner
{

struct CellSizeType
{
    double xy;
    double z;
    double yaw;
    double yaw_half;
    int yaw_count;
    int yaw_count_half;
};

typedef std::map<std::pair<int, int>, Cost> TransitionCostMap;

struct State
{
    State(const tf::Pose pose)
    {
        this->pose = pose;
        tf::Vector3 origin = pose.getOrigin();
        x = meterToDiscreteCell(origin.x(), cellSize.xy);
        y = meterToDiscreteCell(origin.y(), cellSize.xy);
        z = meterToDiscreteCell(origin.z(), cellSize.z);
        double roll, pitch, yaw;
        pose.getBasis().getRPY(roll, pitch, yaw);
        this->yaw = radToDiscreteYaw(yaw);
        id = -1;
//        this->pose.getOrigin().setX(discreteCellToMeter(x, cellSize.xy) - 0.25*State::cellSize.xy);
//        this->pose.getOrigin().setY(discreteCellToMeter(y, cellSize.xy) - 0.25*State::cellSize.xy);
        this->pose.getOrigin().setX(discreteCellToMeter(x, cellSize.xy));
        this->pose.getOrigin().setY(discreteCellToMeter(y, cellSize.xy));
//        this->pose.getOrigin().setZ(discreteCellToMeter(z, cellSize.z));
        this->pose.getBasis().setRPY(roll, pitch, discreteYawToRad(this->yaw));
    }

    /// Provides a hash function on Keys
    struct StateHash
    {
        size_t operator()(const State* state) const
        {
            // a hashing function
            size_t hash_value = state->x + 1013 * state->y + /*26453 * state->z +*/ 376291 * state->yaw;
//            ROS_INFO("%s: %zu",__PRETTY_FUNCTION__, hash_value);
            return hash_value;
        }
    };
    struct StatePred
    {
        size_t operator()(const State* state, const State* other) const
        {
            return state->x == other->x &&
                    state->y == other->y &&
                    fabs(state->z - other->z) < 3 &&
                    state->yaw == other->yaw;
        }
    };

    int id;
    tf::Pose pose;
    int x;
    int y;
    int z;
    int yaw;
    KeyCellDataMap footprint;

    static CellSizeType cellSize;

    static inline int radToDiscreteYaw(double angle)
    {
        return normalizeDiscreteYaw(lrint(floor((angle + State::cellSize.yaw_half) / State::cellSize.yaw)));
    }
    static inline double discreteYawToRad(int angle)
    {
        return angle * State::cellSize.yaw;
    }
    static inline int meterToDiscreteCell(double x, double xSize)
    {
        return CONTXY2DISC(x, xSize);
//        return lrint(floor((x + 0.5*xSize) / xSize));
    }
    static inline double discreteCellToMeter(int x, double xSize)
    {
//        return DISCXY2CONT(x, xSize);
        return x * xSize + 0.25 * xSize;
    }
    static inline double xyzDistance(int dx, int dy, int dz)
    {
        return sqrt(dx*dx+dy*dy+dz*dz);
    }
    static inline int normalizeDiscreteYaw(int yaw)
    {
        return (yaw + State::cellSize.yaw_count_half + State::cellSize.yaw_count) % State::cellSize.yaw_count - State::cellSize.yaw_count_half;
    }
    static inline int discreteYawDistance(int fromYaw, int toYaw)
    {
        return fabs(normalizeDiscreteYaw(toYaw - fromYaw));
    }
    static inline tf::Pose discreteToPose(int x, int y, int z, int yaw)
    {
        return tf::Pose(tf::createQuaternionFromYaw(discreteYawToRad(yaw)),
                tf::Vector3(discreteCellToMeter(x, State::cellSize.xy),
                discreteCellToMeter(y, State::cellSize.xy),
                discreteCellToMeter(z, State::cellSize.z)));

    }
} ;

struct Action
{
    EnvNAVXYTHETALATAction_t discreteAction;
    std::vector<tf::Point> intersectingCells;
    std::vector<tf::Pose> intermediateCellPoses;
    void updateFromDiscretePoses()
    {
        forEach (const sbpl_xy_theta_cell_t& theta_cell, discreteAction.interm3DcellsV)
        {
            double x = State::discreteCellToMeter(theta_cell.x, params::motionPrimitiveResolution);
            double y = State::discreteCellToMeter(theta_cell.y, params::motionPrimitiveResolution);
            tf::Pose pose(tf::createQuaternionFromYaw(State::discreteYawToRad(theta_cell.theta)),
                        tf::Vector3(x, y, 0));
            intermediateCellPoses.push_back(pose);
        }
        forEach (const sbpl_2Dcell_t& xy_cell, discreteAction.intersectingcellsV)
        {
            // convert from primitive resolution to state resolution
            double x = State::discreteCellToMeter(xy_cell.x, params::motionPrimitiveResolution);
            double y = State::discreteCellToMeter(xy_cell.y, params::motionPrimitiveResolution);
            intersectingCells.push_back(tf::Point(x, y, 0.0));
        }
    }
};

typedef std::tr1::unordered_map<State*, int, State::StateHash, State::StatePred> StateIntMap;
typedef std::tr1::unordered_map<int, State*> IdStateMap;

class PlanningEnvironment : public ::DiscreteSpaceInformation
{
public:
    virtual bool initializeSearch(tf::Pose start, tf::Pose goal) = 0;
    virtual void PrintState(const State* state, bool bVerbose = true, FILE* fOut = NULL) = 0;
    virtual void PrintPose(const tf::Pose& pose) = 0;
    virtual int getFromToHeuristic(const State* fromState, const State* toState) = 0;
    virtual bool convertPath(const std::vector<int>& pathIds, std::vector<tf::Pose>& pathPoses, std::vector<Cost>* pathCosts = NULL) const = 0;
    virtual bool convertState(int pathId, tf::Pose* pathPose) const = 0;
    virtual int getStartId() const = 0;
    virtual int getGoalId() const = 0;
    virtual const std::vector<int> getExpandedIds() const = 0;
    virtual void createStartState(const tf::Pose& pose) = 0;
    virtual void createGoalState(const tf::Pose& pose) = 0;
    virtual bool getPathCostTo(const tf::Pose& pose, int* cost) {ROS_WARN("%s not implemented.", __PRETTY_FUNCTION__); return false;}
protected:
    virtual State* findOrCreateState(const tf::Pose& pose) = 0;
};
}


#endif /* PLANNINGENVIRONMENTCOMMON_H_ */
