#include <Controller.hpp>
#include <boost/thread/thread.hpp>

using namespace rsopen;

RobotController::RobotController(int id, uint64_t hash)
{
    _robot = new Robot(id, hash);
}

RobotController::~RobotController()
{
    delete(_robot);
}
 
float RobotController::findOrientationToPoint(pos2d_t orientationCoords)
{
    float dy = orientationCoords.y-data.self.pose.y;
    float dx = orientationCoords.x-data.self.pose.x;
    // Calculate distance
    float targetTheta = atan2(dy, dx);
    float deltaTheta = 2*M_PI-(data.self.pose.rz-targetTheta+M_PI/2);
    if(deltaTheta > M_PI)
    {
        deltaTheta -= 2*M_PI;
    }
    else if(deltaTheta < -M_PI)
    {
        deltaTheta += 2*M_PI;
    }

    // Determine the direction to turn based on the sign of the angle difference and the right-hand rule
    float turn_direction = (deltaTheta > 0) ? 1 : -1;
    // Calaclate the absolute value of the angle difference
    float abs_angle_diff = fabs(deltaTheta);

    // Calculate the angle to rotate by applying the right-hand rule
    return (turn_direction * fmin(abs_angle_diff, M_PI/4));
}

bool RobotController::rotateToPoint(pos2d_t desCoords)
{
    if(!_robot->read(data))
    {
        return false;
    }
    float angle_to_rotate = findOrientationToPoint(desCoords);

    std::cout << "\nRobot missing radians to rotate: " << (round(angle_to_rotate*100)/100) << std::endl; // rounding up to 3 decimal points
    if((round(angle_to_rotate*100)/100) != 0)
    {
        _robot->writeVelocity(0, 0, angle_to_rotate);
        return false;
    }
    else
    {
        _robot->writeVelocity(0, 0, 0);
        return true;
    }
}

bool RobotController::moveToPoint(pos2d desCoords)
{
    if(!_robot->read(data))
    {
        return false;
    }
    usleep(250000);
    float dy = desCoords.y-data.self.pose.y;
    float dx = desCoords.x-data.self.pose.x;
    float targetTheta = atan2(dy, dx);
    float deltaTheta = targetTheta - data.self.pose.rz;

    // float angle_to_rotate = findOrientationToPoint(desCoords);
    // // get distance 
    float distance = std::hypotf(dx,dy);
    float linear_vel = distance > 5 ? 2 : 1;
    linear_vel = distance > 10 ? 3 : 2;

    std::cout << "\nRobot pose: " << data.self.pose.x << " , " << data.self.pose.y << ", " << data.self.pose.rz << std::endl;


    if(distance >= distTolerance)
    {
        _robot->writeVelocity(linear_vel*cos(deltaTheta), linear_vel*sin(deltaTheta), findOrientationToPoint(desCoords));
        // _robot->writeVelocity(0, linear_vel*distance, linear_vel*angle_to_rotate);
        return false;
    }
    else
    {
        _robot->writeVelocity(0, 0, 0);
        return true;
    }
}

bool RobotController::grabBall()
{
    if(!_robot->read(data))
    {
        return false;
    }
    usleep(250000);
    float dy = data.ball.pos.y-data.self.pose.y;
    float dx = data.ball.pos.x-data.self.pose.x;

    float targetTheta = atan2(dy, dx);
    float deltaTheta = targetTheta - data.self.pose.rz;

    // float angle_to_rotate = findOrientationToPoint(desCoords);
    // // get distance 
    float distance = std::hypotf(dx,dy);
    float linear_vel = distance > 2 ? 2 : 0.5;

    std::cout << "\nRobot pose: " << data.self.pose.x << " , " << data.self.pose.y << ", " << data.self.pose.rz << std::endl;


    if(!data.player_status.control_ball)
    {
        _robot->writeVelocity(linear_vel*cos(deltaTheta), linear_vel*sin(deltaTheta), findOrientationToPoint(pos2d(data.ball.pos.x,data.ball.pos.y)));
        return false;
    }
    else
    {
        _robot->writeVelocity(0, 0, 0);
        return true;
    }
}

bool RobotController::kick(int velocity, bool isLobShot)
{
    _robot->writeKickElevation(isLobShot);
    _robot->writeKick(velocity);
    return true;
}

interrobot_t RobotController::GetData()
{
    _robot->read(data);
    return data;
}
