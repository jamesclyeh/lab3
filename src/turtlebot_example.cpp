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
geometry_msgs::Pose ips_pose;
Map* current_map;


// Live
void pose_callback(const PoseWithCovarianceStamped msg) {
    if (init)
    {
      starting_position = msg.pose.pose;
      //std::cout<< "Starting Position" << tf::getYaw(msg.pose.pose.orientation) << " " << starting_position.position.y << " " << starting_position.position.z << std::endl;
      init = false;
    }
    ips_pose = msg.pose.pose;
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
    goal.position.x = 6;
    goal.position.y = -4;
    goal.position.z = 0;

    Pose goal2;
    goal2.position.x = 7;
    goal2.position.y = 0;
    goal2.position.z = 0;

    Pose goal3;
    goal3.position.x = 4;
    goal3.position.y = 0;
    goal3.position.z = 0;

    vector<Milestone*> route = findPath(starting_position, goal, *current_map);
    goal.orientation = route[route.size()-1]->getDestination().orientation;
    vector<Milestone*> route2 = findPath(goal, goal2, *current_map);

    goal2.orientation = route2[route2.size()-1]->getDestination().orientation;
    vector<Milestone*> route3 = findPath(goal2, goal3, *current_map);
    ROS_INFO("Finished palnning");

    route.insert(route.end(), route2.begin()+1, route2.end());
    route.insert(route.end(), route3.begin()+1, route3.end());
    //moveRobot(velocity_publisher, route, loop_rate);
    int i = 1;
    Milestone* start = route[i];
    int max = route.size();
    float threshold = 0.5;

    while (ros::ok())
    {
        if (i < max)
        {
            float startX = start->getDestination().position.x;
            float startY = start->getDestination().position.y;
            float ipsX = ips_pose.position.x;
            float ipsY = ips_pose.position.y;

            float distance = sqrt(pow((startX - ipsX),2.0) + pow((startY - ipsY),2.0));

            if (distance < threshold)
            {
                i++;
                start = route[i];
                std::cout << "Node " << i << " Size " << max << " Distance " << distance << std::endl;
            }
            move(velocity_publisher, start);
        }

        refresh(loop_rate);
    }

    return 0;
}

Pose propogateDynamics(Pose start, float speed, float turnRate)
{
  Pose result = start;
  double roll, pitch, yaw;

  tf::Quaternion bt_q;
  quaternionMsgToTF(start.orientation, bt_q);
  tf::Matrix3x3(bt_q).getRPY(roll, pitch, yaw);

  result.position.x += (1 / (float) UPDATES_PER_SEC) * speed * cos(yaw);
  result.position.y += (1 / (float) UPDATES_PER_SEC) * speed * sin(yaw);
  yaw += (1 / (float) UPDATES_PER_SEC) * turnRate;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(roll, pitch, yaw),
      result.orientation);
  return result;
}

void move (ros::Publisher velPub, Milestone* mCurMilestone)
{
  Pose curPose = ips_pose;
  float speed = (mCurMilestone)->getVelocityLinear();
  float turnRate = (mCurMilestone)->getVelocityAngular();
  mCurMilestone->draw(CARROT);

  //This propogates the Dynamics based using the starting point can instead use getDestination().
  //starting_position = propogateDynamics(starting_position, speed, turnRate);

  Twist vel;

  vector<Point> points;

  // plot /indoor_pos
  points.push_back(curPose.position);
  Pose closePose = curPose;

  closePose.position.x = curPose.position.x + 0.01;
  closePose.position.y = curPose.position.y + 0.01;
  //points.push_back(closePose.position);

  //drawLine(CARROT, points);

  // feed forward control
  // vel.linear.x = speed;
  // vel.angular.z = turnRate;

  // feed back control
  Twist error = getError(curPose, mCurMilestone->getDestination());
  cout<<"Error x: "<<error.linear.x <<", y: "<<error.linear.y <<", yaw: "<<error.angular.z<<endl;
  vel.linear.x += 0.5 * error.linear.x;
  vel.angular.z += 1 * error.linear.y;
  // vel.angular.z -= 0.1 * error.angular.z;

  velPub.publish(vel); // Publish the command velocity

/*
  mCurCycle++;
  if (mCurCycle > (*mCurMilestone)->getNumCycles()) {
    mCurCycle = 0;
    mExpectedPose = (*mCurMilestone)->getEndPose();
    mCurMilestone++;
    if (mCurMilestone == mMilestones.end()) {
      return false;
    }
  }
  return true;*/
}

geometry_msgs::Twist getError(Pose& curPose, Pose mExpectedPose)
{
  float xError = mExpectedPose.position.x - curPose.position.x;
  float yError = mExpectedPose.position.y - curPose.position.y;

  tf::Quaternion q;
  double unusedRoll, unusedPitch;
  double curYaw, expectedYaw;

  quaternionMsgToTF(curPose.orientation, q);
  tf::Matrix3x3(q).getRPY(unusedRoll, unusedPitch, curYaw);
  quaternionMsgToTF(mExpectedPose.orientation, q);
  tf::Matrix3x3(q).getRPY(unusedRoll, unusedPitch, expectedYaw);

  Twist error;

  float PI = 3.14;

  error.angular.z = fmod(expectedYaw - curYaw, 2*PI);
  if (error.angular.z > PI) {
    error.angular.z -= PI;
  }

  // put x/y error in terms of the robot's orientation
  error.linear.x = xError * cos(curYaw) + yError * sin(curYaw);
  error.linear.y = xError * (-sin(curYaw)) + yError * cos(curYaw);

  return error;
}

void moveRobot(ros::Publisher velPub, vector<Milestone*> route, ros::Rate loop_rate)
{
    time_t start;
    time(&start);
    time_t now;

    int curr = 1;
    int max = route.size();
    while (ros::ok())
    {
        if (curr <= max)
        {
            //std::cout << " Move Robot " << std::endl;
            Milestone* temp = route[curr];
            geometry_msgs::Twist msg;
            msg.linear.x = temp->getVelocityLinear();
            msg.angular.z = temp->getVelocityAngular();
            //std::cout << curr << " " <<  msg.linear.x << " " << msg.angular.z << std::endl;
            velPub.publish(msg);

            time(&now);

            if (difftime(now,start) >= temp->getDuration())
            {
                curr++;
                time(&start);
            }
        }
        else
        {
            std::cout << " Move Robot " << std::endl;
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = 0;
            velPub.publish(msg);
        }


        refresh(loop_rate);
    }
}

/*
void track()
{

    int max = route.size();
    int curr = 1;

    for (int i = 0; i < route.size() - 1; i++)
    {
        Pose start = route[0]->getDestination();
        Pose next = route[1]->getDestination();

        float startX = start.position.x;
        float startY = start.position.y;
        float curryaw = tf::getYaw(start.orientation);

        float nextX = next.position.x;
        float nextY = next.position.y;
        float nextyaw = tf::getYaw(next.orientation);

        float diff = nextyaw - curryaw;

        float error = sqrt(pow((startX- nextX),2) + pow((startY - nextY),2));

        while (error < threshold)
        {

        }
    }
}*/




