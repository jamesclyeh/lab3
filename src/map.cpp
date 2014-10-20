#include "map.h"

map::map(const nav_msgs::OccupancyGrid& msg) {
    mGridWidth = msg.info.width;
    mGridHeight = msg.info.height;
    
    int row = -1;
    for (int i = 0; i < mGridWidth * mGridHeight; i++) {
        int col = i % mGridWidth;
        if (i == 0) {
            std::vector<int> new_row;
            mMap.push_back(new_row);
            row++;
        }
        mMap[row].push_back(msg.data[i]);
    }
}

std::vector<int> map::xyToGrid(float x, float y) {
    std::vector<int> coord;
    coord.push_back(floor(x / mResolution));
    coord.push_back(floor(y / mResolution));
  
    return coord;
}

bool map::hasObstacle(float x, float y) {
    std::vector<int> coord = xyToGrid(x, y);
    return mMap[coord[0]][coord[1]] == 100;
}

