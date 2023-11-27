#include "targetPositionGenerator.h"


TargetPositionGenerator::TargetPositionGenerator(GenerationType generationType)
{
    currentGenerationType = generationType;

    // Initialize all the functions
    virtualPositionGenerators[generationTypeConstantAroundACircle] = new GenConstantAroundACircle(this);
}


TargetPositionGenerator::~TargetPositionGenerator()
{
}


// VIRTUAL POSITION GENERATORS:

VirtualPositionGenerators::VirtualPositionGenerators(TargetPositionGenerator *parent)
{
    this->parent = parent;
}

void VirtualPositionGenerators::setRobotNumber(int num)
{
    this->num_robots = num;
    Points.resize(num);
}

void VirtualPositionGenerators::calculateFormation()
{
}

std::vector<Point> VirtualPositionGenerators::returnFormation(int time)
{
    this->time = time;
    calculateFormation();
    return Points;
}


void GenConstantAroundACircle::calculateFormation() // Calculate the positions of the robots in round formation
{
    for(int i=0; i<num_robots; i++){
        //Vector2d getRobotFormationPosition(double center_x, double center_y, float radius, double start_angle, int i, int N)
        Points[i] = getRobotFormationPosition(parent->getParam(param_center_x), parent->getParam(param_center_y), 
            parent->getParam(param_formation_radius), parent->getParam(param_start_angle), i, num_robots);
    }
}

