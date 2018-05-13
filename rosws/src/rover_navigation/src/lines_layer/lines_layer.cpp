//
// Created by matthew on 5/2/18.
//

#include <vector>
#include <rover_navigation/lines_layer/lines_layer.h>
#include <pluginlib/class_list_macros.h>

void rover_navigation::LineLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<Point> &cells) {
    // totally not stolen from another package because i couldn't be bothered to write out a line algorithm
    // todo: maybe use different precisions (bresenham is an integer math solution, but the current ints are not converted
    // todo: to costmap units)
    ROS_DEBUG_STREAM("raytracing");
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    Point pt = {};
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
        cells.push_back(pt);

        if (error > 0) {
            pt.x += x_inc;
            error -= dy;
        } else {
            pt.y += y_inc;
            error += dx;
        }
    }
}

void rover_navigation::LineLayer::parseFromMsg(const rover_navigation::PathConstPtr& p) {
    std::string globalFrameID = this->layered_costmap_->getGlobalFrameID();

    this->waypoints.clear();
    for (auto stampy_the_stamp : p->points) {
        geometry_msgs::PointStamped strampy_the_cramp;
        this->tf_->transformPoint(globalFrameID, stampy_the_stamp, strampy_the_cramp);
        this->waypoints.push_back(strampy_the_cramp.point);
    }

    computeBounds();
    ROS_INFO_STREAM("Set the path to " << this->waypoints.size() << " points.");
    this->current_ = false;
}

void rover_navigation::LineLayer::computeBounds() {
    if (this->waypoints.empty()) {
        _minX = _minY = _maxX = _maxY = 0;
        return;
    }
    ROS_DEBUG_STREAM("computing");

    _minX = _maxX = static_cast<int>(this->waypoints[0].x);
    _minY = _maxY = static_cast<int>(this->waypoints[0].z);
    _maxX++;
    _maxY++;
    
    
    for (int i = 1; i < this->waypoints.size(); i++) {
        _minX = std::min(_minX, static_cast<int>(this->waypoints[i].x));
        _maxX = std::max(_maxX, static_cast<int>(this->waypoints[i].x));
        _minY = std::min(_minY, static_cast<int>(this->waypoints[i].y));
        _maxY = std::max(_maxY, static_cast<int>(this->waypoints[i].y));
    }
}

void rover_navigation::LineLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                               double *min_y, double *max_x, double *max_y) {
    if (!this->enabled_) return;
    if (this->waypoints.empty()) return;

    ROS_INFO_STREAM("updating");

    *min_x = std::min((double)_minX, *min_x);
    *min_y = std::min((double)_minY, *min_y);
    *max_x = std::min((double)_maxX, *max_x);
    *max_y = std::min((double)_maxY, *max_y);
}

void rover_navigation::LineLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i,
                                              int max_j) {
    ROS_DEBUG_STREAM("UPDATING");
    // if (!enabled_) return; fixme: why doesn't this work?
    if (this->waypoints.empty()) return;

    ROS_DEBUG_STREAM("UPDATING_MAYBE");

    std::vector<Point> points;
    for (int i = 0; i < waypoints.size()-1; i++) {
        ROS_DEBUG_STREAM("Trying to do waitpoint " << i << " and " << i +1 << "[size=" << waypoints.size() << "]");
        raytrace(static_cast<int>(waypoints[i].x), static_cast<int>(waypoints[i].y),
                 static_cast<int>(waypoints[i + 1].x), static_cast<int>(waypoints[i + 1].y), points);
    }

    ROS_DEBUG_STREAM("UPDATING_FOR_REAL");

    for (auto e : points) {
        unsigned int mx, my;
        if (master_grid.worldToMap(e.x, e.y, mx, my)) {
            if (master_grid.getCost(mx, my) == costmap_2d::NO_INFORMATION) {
                master_grid.setCost(mx, my, costmap_2d::FREE_SPACE);
            }
        }
    }
}

void rover_navigation::LineLayer::deactivate() {
    this->subscriber.shutdown();
    this->enabled_ = false;
}

void rover_navigation::LineLayer::activate() {
    this->onInitialize();
    this->enabled_ = true;
}

void rover_navigation::LineLayer::reset() {
    this->onInitialize();
}

void rover_navigation::LineLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);
    this->subscriber = nh.subscribe("path", 10, &LineLayer::parseFromMsg, this);
    this->enabled_ = true;
}

PLUGINLIB_EXPORT_CLASS(rover_navigation::LineLayer, costmap_2d::Layer)
