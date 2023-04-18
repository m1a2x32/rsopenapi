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
        bool rotateToPoint(pos2d_t desCoords);
        bool kick(int velocity, bool isLobShot);
        bool grabBall();
        // void GetData(interrobot_t& data);
        interrobot_t GetData();
    private:
        float findOrientationToPoint(pos2d_t orientationCoords);
        Robot *_robot;
        interrobot_t data;
        float distTolerance = 0.15;
};

#endif
