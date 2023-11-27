#ifndef TARGETPOSITIONGENERATOR_H
#define TARGETPOSITIONGENERATOR_H

#pragma once

#include <vector>
#include "utilities.h"

typedef Vector2d Point;

class TargetPositionGenerator;

class VirtualPositionGenerators
{
public:
    VirtualPositionGenerators(TargetPositionGenerator * parent);
    void setRobotNumber(int num);
    std::vector<Point> returnFormation(int time); 
    virtual void calculateFormation();

protected:
    TargetPositionGenerator * parent;
    int num_robots;
    std::vector<Point> Points; // Keep all the points in the formation
    int time = 0;
};

class GenConstantAroundACircle : public VirtualPositionGenerators
{
public:
    GenConstantAroundACircle(TargetPositionGenerator * parent): VirtualPositionGenerators(parent) {}
    void calculateFormation();
};

// All the params names, CAUTION, depending on the generationtype some can be unused

enum IntParams:int{
    param_placeholder,
    totalIntParams
};

enum FloatParams:int{
    param_formation_radius,
    totalFloatParams
};

enum DoubleParams:int{
    param_center_x,
    param_center_y,
    param_start_angle,
    totalDoubleParams
};


class TargetPositionGenerator
{
public:
    enum GenerationType:int{ // All the type in how i can generate data
        generationTypeConstantAroundACircle,
        totalGenerationType // Needet to get the length
    };

    TargetPositionGenerator(GenerationType generationType);
    ~TargetPositionGenerator();

    std::vector<Point> getFormation(int current_time){return currentGenerator()->returnFormation(current_time);};

    // Parameters getters-setters
    void setParam(IntParams id, int val){intParams[id] = val;};
    int getParam(IntParams id){return intParams[id];};
    void setParam(FloatParams id, float val){floatParams[id] = val;};
    float getParam(FloatParams id){return floatParams[id];};
    void setParam(DoubleParams id, double val){doubleParams[id] = val;};
    double getParam(DoubleParams id){return doubleParams[id];};

    void setRobotNumbers(int robot_num){currentGenerator()->setRobotNumber(robot_num);};

private:
    GenerationType currentGenerationType;

    // Needed for a "State-Machine" structure code
    VirtualPositionGenerators * currentGenerator() { return virtualPositionGenerators[currentGenerationType]; }
    VirtualPositionGenerators * virtualPositionGenerators[totalGenerationType];

    float intParams[totalIntParams];    
    float floatParams[totalFloatParams];
    float doubleParams[totalDoubleParams];
};

#endif