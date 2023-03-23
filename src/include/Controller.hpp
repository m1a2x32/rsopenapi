#ifndef _INCLUDE_RSOPEN_CONTROLLER_HPP_
#define _INCLUDE_RSOPEN_CONTROLLER_HPP_

#include <Robot.hpp>

using namespace rsopen;

class RobotController
{
    public:
        RobotController(int id, uint64_t hash);
        ~RobotController();
        bool moveToPoint(pos2d_t desCoords);
        float findOrientationToPoint(pos2d_t orientationCoords);
    private:
        Robot *_robot;
        pos2d_t desiredGoal = pos2d(0,0);
        interrobot_t data;
        float distTolerance = 0.5;
};

#endif