#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>
#include <iostream>
#include <string>

class RegionGrowingConst{
private:
    int SmoothTresh = 1; //Degrees

public:
    bool fast = true;  //faster Calculation of segments. faster = not as good

    int MinClusterSize = 300;
    int NumberOfNeighbours = 20;
    float SmoothnessThreshold = (SmoothTresh * M_PI / 180);
    float CurvatureThreshold = 1.0;
};

class NormalEstConst{
public:
    int KSearch = 40;
};

class TriangulationConst{
private:
    int MaxSurfAng = 45; //Degrees
    int MinAngle = 10;   //Degrees
    int MaxAngle = 120;   //Degrees

public:
    int SearchRadius = 2;
    int Mu = 1;
    float MaximumSurfaceAngle = (MaxSurfAng * M_PI / 180);
    float MinimumAngle = (MinAngle * M_PI / 180);
    float MaximumAngle = (MaxAngle * M_PI / 180);
    bool NormalConsistency = true;

    std::string PLY_OBJ_VTK = "PLY";
};

class VoxGridConst{
public:
    float leafSizeX = 0.1f;
    float leafSizeY = 0.1f;
    float leafSizeZ = 0.1f;
};

class HullConst{
public:
    bool KeepInfo = true;
    float Alpha = 0.05;
};


class SACSegConst{
    public:
        float DistanceThreshold = 0.01;
        bool OptimizeCoefficients = true;
};


#endif // CONSTANTS_H
