#ifndef __SFM_HPP__
#define __SFM_HPP__

#include <array>
#include <vector>
#include <iostream>
#include <cmath>

#define M_PI 3.14159265358979323846 /* pi */

using namespace std;

class SFM
{
private:
    vector<array<double, 2>> dynamicObjects_, staticObjects_;
    array<double, 2> targetPosition_;
    double robotMass_, radiusResponse_, desiredSpeed_;
    double relaxationTime_ = 2;
    double staticGainForce_ = 1;
    double dynamicGainForce_ = 1;
    double staticEffectiveForceRange_ = 1;
    double dynamicEffectiveForceRange_ = 1;
    double dynamicObjectRadius_ = 5.0;

public:
    SFM(double robotMass, double robotRadiusResponse, double robotDesiredSpeed, double relaxationTime);
    void updateStaticParams(double staticGainForce, double staticEffectiveForce);
    void updateDynamicParams(double dynamicGainForce, double dynamicEffectiveForce);
    void addStaticObject(array<double, 2> position);
    void addDynamicObject(array<double, 2> position);
    void setTargetPosition(array<double, 2> position);
    void clearStaticObjects();
    void clearDynamicObjects();
    array<double, 2> calculateForce(array<double, 2> robotPosition, array<double, 2> robotVelocity);
    array<double, 2> calculateTargetForce(array<double, 2> robotPosition, array<double, 2> robotVelocity);
    array<double, 2> calculateStaticForce(array<double, 2> robotPosition);
    array<double, 2> calculateDynamicForce(array<double, 2> robotPosition);

    double calculateDistance(array<double, 2> p1, array<double, 2> p2);
    double calculateDistance(array<double, 2> vector);
    double getAngleFromVector(double xVector, double yVector);
    double radToDegree(double radian);
    double degToRadian(double degree);
    double normalizeAngle(double angle);
};

#endif