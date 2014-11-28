#pragma once

#include <vector>
#include "map.h"
#include "marker.h"

using namespace std;
using namespace geometry_msgs;

#define MAX_RETRY 50
#define MAX_BRANCH 1000

class Milestone {
  public:
    Milestone(Milestone* origin, float velocityLinear,
        float velocityAngular, int duration, Map map);
    Milestone(Pose desitnation);
    float getVelocityLinear() {return mVelocityLinear;};
    float getVelocityAngular() {return mVelocityAngular;};
    int getDuration() {return mDuration;};
    Milestone* getOrigin() {return mOrigin;};
    Pose getDestination();
    Pose getDestination(Map map);
    Milestone* makeRandomNode(Map map);
    bool isValid() {return mIsValid;};
    void draw(MarkerType color);

  private:
    Milestone* mOrigin;
    float mVelocityLinear;
    float mVelocityAngular;
    int mDuration;
    bool mIsValid;
    Pose mDestination;
};

float getDistance(Pose position1, Pose position2);
vector<Milestone*> findPath(Pose start, Pose goal, Map map);
Milestone* getRandomNode(vector<Milestone*> nodes);
void insertOrdered(vector<Milestone*>& nodes, Milestone* node, Pose goal);
void moveRobot(ros::Publisher velPub, vector<Milestone*> route, ros::Rate looprate);
