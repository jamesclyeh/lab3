/**
 * ME 597 Lab2
 */

#include "turtlebot_example.h"
#include <iostream>

ros::Publisher marker_pub;
bool isMapReceived;
geometry_msgs::Pose starting_position;
map* current_map;


/**
 * Callback function for the Position topic
 */
void pose_callback(const turtlebot_example::ips_msg& msg) {
    if(msg.tag_id != TAGID) {
      return;
    }

    double X = msg.X; // Robot X psotition
    double Y = msg.Y; // Robot Y psotition
    double Yaw = msg.Yaw; // Robot Yaw

    std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}

//Example of drawing a curve
void drawCurve(int k) {
    // Curves are drawn as a series of stright lines
    // Simply sample your curves into a series of points

    double x = 0;
    double y = 0;
    double steps = 50;

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
    for(int i = 0; i < steps; i++) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0; //not used
        lines.points.push_back(p); 

        //curve model
        x = x+0.1;
        y = sin(0.1*i*k);   
    }

    //publish new curve
    marker_pub.publish(lines);
}

/**
 * Callback function for the Map topic
 */
void map_callback(const nav_msgs::OccupancyGrid& msg) {
    starting_position = msg.info.origin;
    current_map = new map(msg);
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
    ros::Rate loop_rate(UPDATES_PER_SEC);

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    //Velocity control variable
    geometry_msgs::Twist vel;
    
    while (!isMapReceived) {
        refresh(loop_rate);
    }	

    std::cout << current_map->hasObstacle(starting_position.position.x, starting_position.position.y);    

    while (ros::ok()) {
        refresh(loop_rate);

        //Draw Curves
        drawCurve(1);
        drawCurve(2);
        drawCurve(4);
    
        //Main loop code goes here:
        vel.linear.x = 0.1; // set linear speed
        vel.angular.z = 0.3; // set angular speed

        velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
