//
// Created by matthew on 5/2/18.
//

#ifndef PROJECT_LINES_LAYER_H
#define PROJECT_LINES_LAYER_H

#include <costmap_2d/costmap_layer.h>
#include <rover_navigation/Path.h>
#include <ros/ros.h>

namespace rover_navigation {
    struct Point {
        int x, y;
    };

    class LineLayer : public costmap_2d::Layer {
    public:
        void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x,
                          double *max_y) override;

        void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;

        void deactivate() override;

        void activate() override;

        void reset() override;

    protected:
        void onInitialize() override;

    private:
        int _minX = 0;
        int _minY = 0;
        int _maxX = 0;
        int _maxY = 0;

        void computeBounds();

        void parseFromMsg(const PathConstPtr &p);
        void raytrace(int x0, int y0, int x1, int y1, std::vector<Point>& cells);

        std::vector<geometry_msgs::Point> waypoints; // tf-ed to the frame of the map
        ros::Subscriber subscriber;
    };
}

#endif //PROJECT_LINES_LAYER_H
