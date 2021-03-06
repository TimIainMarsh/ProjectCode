#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>
#include <iostream>
#include <string>

class RegionGrowingConst{
private:
    int SmoothTresh = 1.0; //Degrees

public:
    bool fast = false;  //faster Calculation of segments. faster = not as good

    int MinClusterSize = 5000;
    int NumberOfNeighbours = 20;
    float SmoothnessThreshold = (SmoothTresh * M_PI / 180);
    float CurvatureThreshold = 1.0;

//    int Concave_or_Convex = 1; // 1 == Concave 2 == Convex
};

class NormalEstConst{
public:
    int KSearch = 40;
};

class TriangulationConst{
private:

    int MaxSurfAng = 60; //Degrees
    int MinAngle = 10;   //Degrees
    int MaxAngle = 150;   //Degrees

public:
    float SearchRadius = 0.5;
    int Mu = 2.5;
    float MaximumSurfaceAngle = (MaxSurfAng * M_PI / 180);
    float MinimumAngle = (MinAngle * M_PI / 180);
    float MaximumAngle = (MaxAngle * M_PI / 180);
    bool NormalConsistency = true;

    std::string PLY_OBJ = "PLY";
};

class VoxGridConst{
public:
    float leafSizeX = 0.05;
    float leafSizeY = 0.05;
    float leafSizeZ = 0.05;
};

class HullConst{
public:
    bool KeepInfo = true;
    float Alpha = 0.05;

    int setDimConcave = 2;
    int setDimConvex = 2;
};

class SACSegConst{
    public:
        float DistanceThreshold = 0.01;
        bool OptimizeCoefficients = true;
};

#endif // CONSTANTS_H
