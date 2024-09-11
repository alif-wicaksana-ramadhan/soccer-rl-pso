#include "sfm/sfm.hpp"

SFM::SFM(double robotMass, double robotRadiusResponse, double robotDesiredSpeed, double relaxationTime) : robotMass_(robotMass), radiusResponse_(robotRadiusResponse), desiredSpeed_(robotDesiredSpeed), relaxationTime_(relaxationTime)
{
    robotMass_ = robotMass;
    radiusResponse_ = robotRadiusResponse;
    desiredSpeed_ = robotDesiredSpeed;
    relaxationTime_ = relaxationTime;
}

void SFM::updateStaticParams(double gainForce, double effectiveForceRange)
{
    staticGainForce_ = gainForce;
    staticEffectiveForceRange_ = effectiveForceRange;
}

void SFM::updateDynamicParams(double gainForce, double effectiveForceRange)
{
    dynamicGainForce_ = gainForce;
    dynamicEffectiveForceRange_ = effectiveForceRange;
}

void SFM::addStaticObject(array<double, 2> position)
{
    staticObjects_.push_back(position);
}

void SFM::addDynamicObject(array<double, 2> position)
{
    dynamicObjects_.push_back(position);
}

void SFM::setTargetPosition(array<double, 2> position)
{
    targetPosition_ = position;
}

void SFM::clearStaticObjects()
{
    staticObjects_.clear();
}

void SFM::clearDynamicObjects()
{
    dynamicObjects_.clear();
}

array<double, 2> SFM::calculateForce(array<double, 2> robotPosition, array<double, 2> robotVelocity)
{
    array<double, 2> targetForce = calculateTargetForce(robotPosition, robotVelocity);
    array<double, 2> staticForce = calculateStaticForce(robotPosition);
    array<double, 2> dynamicForce = calculateDynamicForce(robotPosition);

    array<double, 2> resultantForce = {
        targetForce[0] + staticForce[0] + dynamicForce[0],
        targetForce[1] + staticForce[1] + dynamicForce[1]};

    return resultantForce;
}

array<double, 2> SFM::calculateTargetForce(array<double, 2> robotPosition, array<double, 2> robotVelocity)
{
    array<double, 2> targetVector = {
        targetPosition_[0] - robotPosition[0],
        targetPosition_[1] - robotPosition[1]};

    double dist = this->calculateDistance(targetVector);
    double vectorX = targetVector[0] / dist;
    double vectorY = targetVector[1] / dist;

    array<double, 2> targetForce = {
        vectorX * robotMass_ * ((desiredSpeed_)-robotVelocity[0]) / relaxationTime_,
        vectorY * robotMass_ * ((desiredSpeed_)-robotVelocity[1]) / relaxationTime_};

    return targetForce;
}

array<double, 2> SFM::calculateStaticForce(array<double, 2> robotPosition)
{
    array<double, 2> staticForce = {0.0, 0.0};
    array<double, 2> objectVector;
    double distance, vectorX, vectorY;
    double socialForce, physicalForce;

    for (int i = 0; i < staticObjects_.size(); i++)
    {
        objectVector = {
            staticObjects_[i][0] - robotPosition[0],
            staticObjects_[i][1] - robotPosition[1]};
        distance = calculateDistance(objectVector);
        vectorX = objectVector[0] / distance;
        vectorY = objectVector[1] / distance;

        if (distance <= radiusResponse_)
        {
            socialForce = pow(staticGainForce_, (radiusResponse_ - distance) / staticEffectiveForceRange_);
            physicalForce = staticGainForce_ * (radiusResponse_ - distance);

            if (physicalForce < 0)
                physicalForce = 0;
        }
        else
        {
            socialForce = 0;
            physicalForce = 0;
        }

        staticForce[0] -= (socialForce + physicalForce) * vectorX;
        staticForce[1] -= (socialForce + physicalForce) * vectorY;
    }

    staticObjects_.clear();

    return staticForce;
}

array<double, 2> SFM::calculateDynamicForce(array<double, 2> robotPosition)
{
    array<double, 2> dynamicForce = {0.0, 0.0};
    array<double, 2> objectVector;
    double distance, vectorX, vectorY;
    double socialForce, physicalForce;
    double totalRadius = radiusResponse_ + dynamicObjectRadius_;

    for (int i = 0; i < dynamicObjects_.size(); i++)
    {
        objectVector = {
            dynamicObjects_[i][0] - robotPosition[0],
            dynamicObjects_[i][1] - robotPosition[1]};
        distance = calculateDistance(objectVector);
        vectorX = objectVector[0] / distance;
        vectorY = objectVector[1] / distance;

        if (distance <= totalRadius)
        {
            socialForce = pow(dynamicGainForce_, (totalRadius - distance) / dynamicEffectiveForceRange_);
            physicalForce = dynamicGainForce_ * (totalRadius - distance);

            if (physicalForce < 0)
                physicalForce = 0;
        }
        else
        {
            socialForce = 0;
            physicalForce = 0;
        }

        dynamicForce[0] -= (socialForce + physicalForce) * vectorX;
        dynamicForce[1] -= (socialForce + physicalForce) * vectorY;
    }

    dynamicObjects_.clear();

    return dynamicForce;
}

double SFM::calculateDistance(array<double, 2> p1, array<double, 2> p2)
{
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p2[1], 2));
}

double SFM::calculateDistance(array<double, 2> vector)
{
    return sqrt(pow(vector[0], 2) + pow(vector[1], 2));
}

double SFM::getAngleFromVector(double xVector, double yVector)
{
    double angle = 0;
    if (xVector < 0)
        angle = radToDegree(atan(yVector / xVector)) + 180;
    else if (xVector > 0)
    {
        if (yVector < 0)
            angle = radToDegree(atan(yVector / xVector)) + 360;
        else
            angle = radToDegree(atan(yVector / xVector));
    }
    return angle;
}

double SFM::radToDegree(double radian)
{
    return 180 * radian / M_PI;
}

double SFM::degToRadian(double degree)
{
    return degree * M_PI / 180;
}

double SFM::normalizeAngle(double angle)
{
    while (angle < 0)
        angle += 360;
    while (angle > 360)
        angle -= 360;
    return angle;
}