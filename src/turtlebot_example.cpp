/**
 * ME 597 Lab2
 */

#include "turtlebot_example.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "rrt.h"
#include "marker.h"
#include <iostream>
#include <ros/ros.h>

ros::Publisher marker_pub;
bool isMapReceived;
bool init;
geometry_msgs::Pose starting_position;
Map* current_map;


// Live
void pose_callback(const PoseWithCovarianceStamped msg) {
    if (init) {
      starting_position = msg.pose.pose;
      init = false;
    }
}

//Example of drawing a curve
void drawCurve(int k, vector<Milestone*> route) {
    // Curves are drawn as a series of stright lines
    // Simply sample your curves into a series of points


    visualization_msgs::Marker lines;
    lines.header.frame_id = "/map";
    lines.id = k; //each curve must have a unique id or you will overwrite an old ones
    lines.type = visualization_msgs::Marker::LINE_STRIP;
    lines.action = visualization_msgs::Marker::ADD;
    lines.ns = "curves";
    lines.scale.x = 0.1;
    lines.color.r = 1.0;
    lines.color.b = 0.2*k;
    lines.color.a = 1.0;

    //generate curve points
    for(int i = 0; i < route.size(); i++) {
        lines.points.push_back(route[i]->getDestination().position);
    }

    //publish new curve
    marker_pub.publish(lines);
}

/**
 * Callback function for the Map topic
 */
void map_callback(const nav_msgs::OccupancyGrid& msg) {
    current_map = new Map(msg);
    isMapReceived = true;
}

void refresh(ros::Rate& loop_rate) {
    loop_rate.sleep();
    ros::spinOnce();
}

int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    markerInit(n);
    ros::Rate loop_rate(UPDATES_PER_SEC);

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    isMapReceived = false;
    init = true;

    while (!isMapReceived) {
        refresh(loop_rate);
        loop_rate.sleep();
        ROS_INFO("Waiting for map");
    }
    ROS_INFO("Map received");

    Pose goal;
    goal.position.x = 7;
    goal.position.y = 4;
    goal.position.z = 0;
    vector<Milestone*> route = findPath(starting_position, goal, *current_map);
    //drawCurve(1, route);
    while (ros::ok()) {
        refresh(loop_rate);
    }

    return 0;
}
