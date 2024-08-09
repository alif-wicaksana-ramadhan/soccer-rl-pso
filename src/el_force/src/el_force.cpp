#include <cmath>
#include <iostream>
#include "el_force/el_force.hpp"

ElForce::ElForce(double k, double ownCharge, double enemyCharge, double targetCharge) : k_(k), ownCharge_(ownCharge), enemyCharge_(enemyCharge), targetCharge_(targetCharge)
{
    target_ = {0.0, 0.0};
}

void ElForce::setTarget(std::vector<double> target_pos)
{
    target_ = target_pos;
}

void ElForce::addEnemy(std::vector<double> enemyPos)
{
    enemies_.push_back(enemyPos);
}

void ElForce::clearEnemies()
{
    enemies_.clear();
}

double ElForce::calculateNorm(std::vector<double> vec)
{
    double sum = 0.0;
    for (int i = 0; i < (int)vec.size(); ++i)
    {
        sum += std::pow(vec[i], 2);
    }
    return std::sqrt(sum);
}

std::vector<double> ElForce::calculateForce(std::vector<double> dronePos)
{
    std::vector<double> vec;
    std::vector<double> forceVector = {0.0, 0.0};

    if (dronePos.size() != target_.size())
    {
        std::cout << "NOT MATCH" << std::endl;
    }

    for (int i = 0; i < enemies_.size(); i++)
    {
        for (int j = 0; j < enemies_[i].size(); j++)
        {
            double step = enemies_[i][j] - dronePos[j];
            vec.push_back(step);
        }

        double norm = calculateNorm(vec);
        double force = k_ * ownCharge_ * enemyCharge_ / (norm * norm);

        for (int j = 0; j < enemies_[i].size(); j++)
        {
            forceVector[j] -= (vec[j] / norm) * force;
        }

        vec.clear();
    }

    for (int i = 0; i < dronePos.size(); i++)
    {
        double step = target_[i] - dronePos[i];
        vec.push_back(step);
    }

    double norm = calculateNorm(vec);
    double force = k_ * ownCharge_ * targetCharge_ / (norm * norm);

    for (int i = 0; i < dronePos.size(); i++)
    {
        forceVector[i] += (vec[i] / norm) * force;
    }
    vec.clear();
    enemies_.clear();

    return forceVector;
}