#include "map.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <time.h>

#define UPDATES_PER_SEC 20

geometry_msgs::Twist getError(Pose& curPose, Pose mExpectedPose);

