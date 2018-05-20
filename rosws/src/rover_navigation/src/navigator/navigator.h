//
// Created by matthew on 5/20/18.
//

#ifndef PROJECT_NAVIGATOR_H
#define PROJECT_NAVIGATOR_H

#include <string>
#include <ros/duration.h>
#include <control_toolbox/pid.h>

namespace rover_navigation {

    /// it has a frame
    /// it has a goal
    /// it computes the stuff

    enum NavState {
        NOT_RUNNING,
        GOING,
        GOAL_OK
    };

    class Navigator {
    public:
        Navigator();

        void setGoalPosition(double x, double y);
        void setCurrentPosition(double x, double y, double ang);

        void update(ros::Duration dt);
        void acceptGoal();

        double getCurrentLinear();
        double getCurrentAngular();

        NavState getNavState();

    private:
        double goalX, goalY, curX, curY, curAng;
        double max_speed;
        double max_ang;

        double curLin;
        double curAngC;

        NavState navState;

        control_toolbox::Pid lin, ang;
        double tolerance;
    };

}


#endif //PROJECT_NAVIGATOR_H
