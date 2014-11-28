#include "rrt.h"
#include "turtlebot_example.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <random>
#include <cmath>

static default_random_engine gen;

Milestone::Milestone(Milestone* origin, float velocityLinear,
    float velocityAngular, int duration, Map map) : mOrigin(origin),
    mIsValid(false), mVelocityLinear(velocityLinear),
    mVelocityAngular(velocityAngular), mDuration(duration) {
  getDestination(map);
}

Milestone::Milestone(Pose destination) : mOrigin(NULL),
  mDestination(destination), mIsValid(true) {
}

Pose Milestone::getDestination() {
  return mDestination;
}

Pose Milestone::getDestination(Map map) {
  if (mIsValid == false) {
    Pose destination = mOrigin->getDestination(map);
    tf::Quaternion quat;
    double r, p, yaw;

    tf::quaternionMsgToTF(destination.orientation, quat);
    tf::Matrix3x3(quat).getRPY(r, p, yaw);

    for (int i = 0; i < mDuration * UPDATES_PER_SEC; i++) {
      destination.position.x +=
        1 / float(UPDATES_PER_SEC) * mVelocityLinear * cos(yaw);
      destination.position.y +=
        1 / float(UPDATES_PER_SEC) * mVelocityLinear * sin(yaw);
      yaw += 1 / float(UPDATES_PER_SEC) * mVelocityAngular;

      tf::quaternionTFToMsg(
          tf::createQuaternionFromRPY(r, p, yaw),
          destination.orientation);
      //ROS_INFO("Check obstacle exist.");
      if (map.hasObstacle(destination) || map.getSurroundingCells(destination)) {
        //ROS_INFO("Fail node");
        return mDestination;
      }
    }

    mIsValid = true;
    mDestination = destination;
  }

  return mDestination;
}

Milestone* Milestone::makeRandomNode(Map map) {
  uniform_real_distribution<> velocityLinearGenerator(0.0, 0.5);
  
  uniform_int_distribution<> durationGenerator(1, 3);
  for (int i = 0; i < MAX_RETRY; i++) {
    float velocityLinear = velocityLinearGenerator(gen);
    uniform_real_distribution<> velocityAngularGenerator(-1.0, 1);
    float velocityAngular = velocityAngularGenerator(gen);
    int duration = durationGenerator(gen);
    Milestone* newNode = new Milestone(this, velocityLinear, velocityAngular,
        duration, map);
    //ROS_INFO("Check new node valid");
    if (newNode->isValid())
      return newNode;
    else
      delete newNode;
  }

  return NULL;
}

float getDistance(Pose position1, Pose position2) {
  float x = position1.position.x - position2.position.x;
  float y = position1.position.y - position2.position.y;

  return sqrt(x * x + y * y);
}

void insertOrdered(vector<Milestone*>& nodes, Milestone* node, Pose goal) {
  float newDist = getDistance(node->getDestination(), goal);
  vector<Milestone*>::iterator iter = nodes.begin();
  while (iter != nodes.end() && getDistance((*iter)->getDestination(), goal) < newDist) {
    iter++;
  }
  nodes.insert(iter, node);
}

Milestone* getRandomNode(vector<Milestone*> nodes) {
  uniform_real_distribution<> picker(0, 1);
  for(int i = 0; i < nodes.size(); i++) {
    bool pick = picker(gen) > 0.7 ? true:false;
    if (pick) {
      ROS_INFO("Picked node %d", i);
      return nodes[i];
    }
  }

  ROS_INFO("Default to node %d", 0);
  return nodes[0];
}

vector<Milestone*> findPath(Pose start, Pose goal, Map map) {
  vector<Milestone*> tree;
  Milestone* finalNode = NULL;
  ROS_INFO("Add origin to tree");
  tree.push_back(new Milestone(start));
  for (int i = 0; i < MAX_BRANCH; i++) {
    Milestone* branchRoot = getRandomNode(tree);
    //ROS_INFO("Try to create random Node");
    Milestone* newBranch = branchRoot->makeRandomNode(map);
    if (newBranch == NULL) {
      //ROS_INFO("No valid random node created");
      continue;
    } else {
      //ROS_INFO("Insert node");
      insertOrdered(tree, newBranch, goal);
      //ROS_INFO("Draw branch");
      newBranch->draw(RANDOM_TREE);
    }
    float distanceToGoal = getDistance(newBranch->getDestination(), goal);
    //ROS_INFO("Distance to goal: %f", distanceToGoal);
    if (distanceToGoal < 0.3) {
      finalNode = newBranch;
      //ROS_INFO("End position: x: %f, y: %f", finalNode->getDestination().position.x, finalNode->getDestination().position.y);
      break;
    }
  }

  vector<Milestone*> route;  
  if (finalNode == NULL) {
    ROS_INFO("Found no path");
    return route;
  }
  route.push_back(finalNode);
  while(finalNode->getOrigin() != NULL) {
    route.push_back(finalNode->getOrigin());
    finalNode = finalNode->getOrigin();
  }

  reverse(route.begin(), route.end());

  for (int i = 1; i < route.size(); i++)
    route[i]->draw(SELECTED_TREE);
  return route;
}

void Milestone::draw(MarkerType color) {
  Pose start = mOrigin->getDestination();
  vector<Point> points;
  points.push_back(start.position);

  tf::Quaternion quat;
  double r, p, yaw;

  tf::quaternionMsgToTF(start.orientation, quat);
  tf::Matrix3x3(quat).getRPY(r, p, yaw);

  for (int i = 0; i < mDuration * UPDATES_PER_SEC; i++) {
    start.position.x +=
        1 / float(UPDATES_PER_SEC) * mVelocityLinear * cos(yaw);
    start.position.y +=
        1 / float(UPDATES_PER_SEC) * mVelocityLinear * sin(yaw);
    yaw += 1 / float(UPDATES_PER_SEC) * mVelocityAngular;

    tf::quaternionTFToMsg(
        tf::createQuaternionFromRPY(r, p, yaw),
        start.orientation);
    points.push_back(start.position);
  }

  drawLine(color, points);
}
