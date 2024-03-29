#pragma once

#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

using namespace std;
using namespace geometry_msgs;

class Map {
    public:
        Map(const nav_msgs::OccupancyGrid& msg);
        bool hasObstacle(Pose location);
        bool getSurroundingCells (Pose location);

    private:
        int mGridWidth;
        int mGridHeight;
        int mXOffset;
        int mYOffset;
        std::vector<std::vector<int> > mMap;
        float mResolution;
        std::vector<int> xyToGrid(float x, float y);
};
