/*
 * VisualPathPlannerNode.cpp
 *
 *  Created on: 2 Mar 2013
 *      Author: andreas
 */

#include "VisualPathPlannerNode.h"
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <tf_conversions/tf_eigen.h>
//#include <tf_conversions/eigen_msg.h>
//#include <tf/transform_datatypes.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH
#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>

namespace gki_3dnav_planner
{

ros::Publisher visualization_publisher;

VisualPathPlannerNode::VisualPathPlannerNode()
{
    // create an interactive marker server on the topic namespace simple_marker
    collisioinTestServer.reset( new interactive_markers::InteractiveMarkerServer("gki_3dnav_planner/visual_path_planner") );

    // setup planning scene
    scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    //scene_monitor->startStateMonitor("/move_group/monitored_planning_scene");
    //scene_monitor->startWorldGeometryMonitor();
    //scene_monitor->startSceneMonitor();
    scene_monitor->requestPlanningSceneState("/get_planning_scene");
    scene = scene_monitor->getPlanningScene();
    planning_scene_publisher = ros::NodeHandle().advertise<moveit_msgs::PlanningScene>("planning_scene", 3);
    //scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "planning_scene");

    // create an interactive marker for our server
    start_control_marker.header.frame_id = scene_monitor->getPlanningScene()->getPlanningFrame();
    ROS_INFO_STREAM(start_control_marker.header.frame_id);
    start_control_marker.name = "start_control";
    start_control_marker.description = "start";
    std_msgs::ColorRGBA gray;
    gray.r = 0.2;
    gray.g = 0.2;
    gray.b = 0.2;
    gray.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    visualization_msgs::Marker arrow_marker;
    arrow_marker.pose.orientation.w = 1;
    arrow_marker.color = gray;
    arrow_marker.scale.x = 0.1;
    arrow_marker.scale.y = 0.05;
    arrow_marker.scale.z = 0.05;
//    box_control.markers.push_back(arrow_marker);

    // add the control to the interactive marker
    start_control_marker.controls.push_back(box_control);
    start_control_marker.scale = 0.5;
    start_control_marker.pose.position.z = 2;

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    start_control_marker.controls.push_back(control);
    control.name = "move_xy";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
    start_control_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.name = "plan path";
    control.always_visible = true;
    control.markers.push_back(arrow_marker);
    start_control_marker.controls.push_back(control);

    goal_control_marker = start_control_marker;
    goal_control_marker.name = "goal_control";
    goal_control_marker.pose.position.x = 1.0;
    goal_control_marker.description = "goal";
    goal_control_marker.controls.back().markers.front().color.r = 0.8;
    start_control_marker.controls.back().markers.front().color.b = 0.8;

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    collisioinTestServer->insert(start_control_marker, boost::bind( &VisualPathPlannerNode::processFeedback, this, _1 ));
    //collisioinTestServer->insert(goal_control_marker, boost::bind( &VisualPathPlannerNode::processFeedback, this, _1 ));

    // 'commit' changes and send to all clients
    collisioinTestServer->applyChanges();

    ros::NodeHandle nh;
    planPathClient = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan", false);
}

VisualPathPlannerNode::~VisualPathPlannerNode()
{
}

void VisualPathPlannerNode::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    geometry_msgs::Pose robot_pose;
    tf::Pose robot_pose_tf;
    Eigen::Affine3d robot_pose_eigen;
    //planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor);
    robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
    std::vector<std::string> link_names = robot_state.getRobotModel()->getLinkModelNames();
    std::vector<std::string> link_names2;
    link_names2.push_back("base_link");
    std_msgs::ColorRGBA yellow;
    yellow.a = 1;
    yellow.r = 1;
    yellow.g = 1;
    yellow.b = 0;
    std_msgs::ColorRGBA teal;
    teal.a = 1;
    teal.r = 0;
    teal.g = 1;
    teal.b = 1;
    std_msgs::ColorRGBA orange;
    orange.a = 1;
    orange.r = 1;
    orange.g = 0.5;
    orange.b = 0;

    visualization_msgs::MarkerArray visual_array;
//	robot_state.getRobotMarkers(visual_array, link_names, yellow, "initial", ros::Duration(10), true);
//    visualization_publisher.publish(visual_array);
//    visual_array = visualization_msgs::MarkerArray();
    moveit_msgs::PlanningScene scene_msg;

    switch (feedback->event_type)
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        //robot_state.printStateInfo(std::cout);
//        robot_pose_eigen = planning_scene->getTransforms().getTransform("/base_footprint");
//        tf::transformEigenToTF(robot_pose_eigen, robot_pose_tf);
//        ROS_INFO_STREAM("transform: "<<robot_pose_tf.getOrigin().x()<<" "<<robot_pose_tf.getOrigin().y()<<" "<<robot_pose_tf.getOrigin().z());
        //planning_scene->setCurrentState(robot_state);
//        for_each(const std::string& link_name, link_names2)
//        {
//        	ROS_INFO_STREAM(link_name);
//        }
        //planning_scene->
//        planning_scene->checkCollision(request, result);
//        ROS_INFO_STREAM("collision: "<< (result.collision ? "true": "false"));
//        ROS_INFO_STREAM("collision distance: "<<result.distance);
        //robot_state.printTransforms(std::cout);
        //robot_state.printDirtyInfo(std::cout);
//        planSrv.request.start = collisionSrv.request.poses[0];
//        planSrv.request.goal = collisionSrv.request.poses[1];
//        if (planPathClient.exists())
//        {
//            planPathClient.call(planSrv);
//        }
        break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        // read marker poses
        //ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z);
//        start_control_marker.pose = feedback->pose;
//        robot_pose = feedback->pose;
//        robot_pose.position.z = 0;
//        tf::poseMsgToTF(robot_pose, robot_pose_tf);
//        robot_pose_eigen = planning_scene->getTransforms().getTransform("/base_footprint");
//        tf::transformTFToEigen(robot_pose_tf, robot_pose_eigen);
//        planning_scene->getTransformsNonConst().setTransform(robot_pose_eigen, "/base_footprint");
//
//        result.clear();
//        planning_scene->checkCollision(request, result);
//        robot_pose_eigen = planning_scene->getTransforms().getTransform("/base_footprint");
//        tf::transformEigenToTF(robot_pose_eigen, robot_pose_tf);
//        ROS_INFO_STREAM("robot_pose: "<<robot_pose_tf.getOrigin().x()<<" "<<robot_pose_tf.getOrigin().y()<<" "<<robot_pose_tf.getOrigin().z());
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    	robot_state.getRobotMarkers(visual_array, link_names, yellow, "initial", ros::Duration(10), true);
        robot_pose = feedback->pose;
        robot_pose.position.z = 0;
        tf::poseMsgToTF(robot_pose, robot_pose_tf);
        tf::transformTFToEigen(robot_pose_tf, robot_pose_eigen);
        robot_state.updateStateWithLinkAt("base_footprint", robot_pose_eigen);
        robot_state.updateCollisionBodyTransforms();

        //robot_state.updateLinkTransforms();
    	scene->setCurrentState(robot_state);
        scene->checkCollision(request, result);
        if (result.collision)
        	robot_state.getRobotMarkers(visual_array, link_names, orange, "collision", ros::Duration(10), true);
        else
        	robot_state.getRobotMarkers(visual_array, link_names, teal, "moved", ros::Duration(10), true);
        visualization_publisher.publish(visual_array);
        scene->getPlanningSceneMsg(scene_msg);
        planning_scene_publisher.publish(scene_msg);
        break;
    default:
        break;
    }
    collisioinTestServer->applyChanges();
}

} /* namespace gki_3dnav_planner */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_path_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    gki_3dnav_planner::VisualPathPlannerNode visualPlanner;
    gki_3dnav_planner::visualization_publisher = ros::NodeHandle("~").advertise<visualization_msgs::MarkerArray>("vis_debug", 10);
    ROS_INFO("visual path planner initialized.");
    ros::spin();
    return 0;
}

