#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <typeinfo>
#include <math.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/io.h>
#include <pcl/point_types.h>

//my includes
#include "timedate.h"
#include "Functions.h"
#include "cloudoperations.h"
#include "constants.h"
#include "segmentor.h"
#include "projection.h"

#include <pcl/visualization/cloud_viewer.h>


int
main()
{
    displayTime();
    cout<<"Start\n"<< endl;

    string filename = "../ptClouds/DeepSpace-Full";

    PointCloud<PointXYZRGB>::Ptr origCloud =  openCloud(filename + ".pcd");

    cout<<"Calculating Normals...\n"<< endl;
    PointCloud<Normal>::Ptr normals = normalCalc(origCloud);
    cout<<"Normals Calculated...\n"<< endl;

    displayTime();
    cout<<"Segmentation Starting..."<< endl;
    vector <PointIndices::Ptr> vector_of_segments;
    vector <PointIndices::Ptr> vector_of_roof_floor;
    PointCloud <PointXYZRGB>::Ptr segCloud(new PointCloud <PointXYZRGB>);

    std::tie(vector_of_roof_floor,vector_of_segments, segCloud) = segmentor(origCloud, normals);

    PointCloud <PointXYZRGB>::Ptr extentCloud = vectorToCloud(vector_of_segments, segCloud);
    cout<<"Segmentation Complete...\n"<< endl;
    displayTime();

    vector < PointCloud<PointXYZRGB>::Ptr> Boundries = getBoundriesOfSegments(vector_of_segments, segCloud);

    cout<<"Extracting corner points...\n"<<endl;
    ExtractCornerPoints(vector_of_segments,vector_of_roof_floor,Boundries, segCloud);
    cout<<"Corner points extracted...\n"<<endl;


    cout<<"Writing boundry to .obj file...\n"<<endl;
    CreateCornerFile(Boundries,filename);

    cout<<"Writing Cloud to File...\n"<<endl;
    string outputFileName = filename + "-Segmented";
    write(extentCloud,outputFileName + ".pcd");
    displayTime();


    cout<<"Displaying Cloud...\n"<< endl;
    Viewer(extentCloud);


    cout<<"End"<< endl;
    return 0;
}
