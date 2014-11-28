#include "map.h"
#include <ros/ros.h>

Map::Map(const nav_msgs::OccupancyGrid& msg) {
    mGridWidth = msg.info.width;
    mGridHeight = msg.info.height;
    mXOffset = msg.info.origin.position.x * -1;
    mYOffset = msg.info.origin.position.y * -1;
    mResolution = 0.1;

    int row = -1;
    for (int i = 0; i < mGridWidth * mGridHeight; i++) {
        int col = i % mGridWidth;
        if (col == 0) {
            std::vector<int> new_row;
            mMap.push_back(new_row);
            row++;
        }
        mMap[row].push_back(msg.data[i]);
    }
}

std::vector<int> Map::xyToGrid(float x, float y) {
    std::vector<int> coord;
    coord.push_back(floor((x + mXOffset) / mResolution));
    coord.push_back(floor((y + mYOffset) / mResolution));

    return coord;
}
bool Map::getSurroundingCells (Pose location)
{
    float x = location.position.x;
    float y = location.position.y;
    std::vector<int> coord = xyToGrid(x, y);
    int tol = 4;

    return (coord[1]+tol>100 ||
            coord[1]-tol<0 ||
            coord[0]+tol>100 ||
            coord[0]-tol<0 ||
            mMap[coord[1]-tol][coord[0]] == 100 ||
            mMap[coord[1]][coord[0]+tol] == 100 ||
            mMap[coord[1]+tol][coord[0]] == 100 ||
            mMap[coord[1]][coord[0]-tol] == 100);


}

bool Map::hasObstacle(Pose location) {
    float x = location.position.x;
    float y = location.position.y;
    std::vector<int> coord = xyToGrid(x, y);
    return mMap[coord[1]][coord[0]] == 100;
}

