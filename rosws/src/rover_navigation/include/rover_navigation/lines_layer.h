//
// Created by matthew on 5/2/18.
//

#ifndef PROJECT_LINES_LAYER_H
#define PROJECT_LINES_LAYER_H

#include <costmap_2d/costmap_layer.h>
#include <rover_navigation/Path.h>

namespace rover_navigation {
    struct Point {
        int x, y;
    };

    class LineLayer : public costmap_2d::CostmapLayer {
    public:

    private:
        void parseFromMsg(Path &p);
        void raytrace(int x0, int y0, int x1, int y1, std::vector<Point>& cells);

        std::vector<geometry_msgs::Point> waypoints; // tf-ed to the frame of the map
    };
}

#endif //PROJECT_LINES_LAYER_H
