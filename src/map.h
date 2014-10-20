#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

class map {
    public:
        map(const nav_msgs::OccupancyGrid& msg);
        bool hasObstacle(float x, float y);
    
    private:
        int mGridWidth;
        int mGridHeight;
        std::vector<std::vector<int> > mMap;
        float mResolution;
        std::vector<int> xyToGrid(float x, float y);
};
