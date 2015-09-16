#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <typeinfo>

//generic
#include <pcl/common/io.h>
#include <pcl/point_types.h>

//Searching
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>
//my includes
#include "timedate.h"
#include "Functions.h"
#include "cloudoperations.h"
#include "constants.h"
#include "segmentor.h"

//region growing
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/intersections.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

using namespace pcl;
using namespace std;




float
RandomFloat(float min, float max)
{
    float r = (float)rand() / (float)RAND_MAX;
    return min + r * (max - min);
}

ModelCoefficients::Ptr
getModelCoeff(PointCloud<PointXYZRGB>::Ptr input_cloud)
{

    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    SACSegmentation<PointXYZRGB> segmentation;
    segmentation.setInputCloud(input_cloud);
    segmentation.setModelType(SACMODEL_PLANE);
    segmentation.setMethodType(SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    PointIndices::Ptr inlierIndices(new PointIndices);
    segmentation.segment(*inlierIndices, *coefficients);


    return coefficients;

}


void
ExpandSegmentsToExtents(vector <PointIndices::Ptr> indices, PointCloud <PointXYZRGB>::Ptr cloud)
{

    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0,0,0);

//    viewer.addCoordinateSystem (1.0);
    vector<ModelCoefficients> lines_ModCoeff;
    vector<Eigen::VectorXf> lines_Eigen;
    PointCloud <PointXYZRGB>::Ptr result(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (cloud);
    for (int out=0; out < indices.size(); out++){
        PointIndices::Ptr outerSegment = indices[out];
        PointCloud <PointXYZRGB>::Ptr outerCloud(new PointCloud <PointXYZRGB>);
        filtrerG.setIndices(outerSegment);
        filtrerG.filter(*outerCloud);

        ModelCoefficients::Ptr modelCoeff_outer;
        modelCoeff_outer = getModelCoeff(outerCloud);




        for (int in=0; in < indices.size(); in++){
            if(in == out){
                continue;
                cout<<"skipped"<<endl;
            }
            PointIndices::Ptr innerSegment = indices[in];
            PointCloud <PointXYZRGB>::Ptr innerCloud(new PointCloud <PointXYZRGB>);
            filtrerG.setIndices(innerSegment);
            filtrerG.filter(*innerCloud);


            ModelCoefficients::Ptr modelCoeff;
            modelCoeff = getModelCoeff(innerCloud);
            cout<<modelCoeff->values[0]<<"  "<<modelCoeff->values[1]<<"  "<<modelCoeff->values[2]<<"  "<<modelCoeff->values[3]<<endl;


            double angular_tolerance=0.0;
            Eigen::Vector4f plane_a;
            plane_a.x()=modelCoeff->values[0];
            plane_a.y()=modelCoeff->values[1];
            plane_a.z()=modelCoeff->values[2];
            plane_a.w()=modelCoeff->values[3];
            Eigen::Vector4f plane_b;
            plane_b.x()=modelCoeff_outer->values[0];
            plane_b.y()=modelCoeff_outer->values[1];
            plane_b.z()=modelCoeff_outer->values[2];
            plane_b.w()=modelCoeff_outer->values[3];

            Eigen::VectorXf line;
            pcl::planeWithPlaneIntersection(plane_a,plane_b,line,angular_tolerance);
            pcl::ModelCoefficients::Ptr l(new pcl::ModelCoefficients ());
            l->values.resize(6);
            for (int i=1;i<6;i++)
            {
                l->values[i]=line[i];
            }
            lines_Eigen.push_back(line);
            lines_ModCoeff.push_back(*l);
        }
        *result += *outerCloud;
    }

    for (int i=0; i < lines_ModCoeff.size(); i++){
        string R;
        ostringstream convert;
        convert << i;
        R = convert.str();


        viewer.addLine(lines_ModCoeff[i],R);
    }
//    viewer.addPointCloud(result);

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }

}



int
main()
{

    displayTime();
    cout<<"Start"<< endl;
    string filename = "../ptClouds/box";
    PointCloud<PointXYZRGB>::Ptr origCloud =  openCloud(filename + ".pcd");

    cout<<"Calculating Normals..."<< endl;
    PointCloud<Normal>::Ptr normals = normalCalc(origCloud);
    cout<<"Normals Calculated..."<< endl;

    cout<<"Segmentation Starting..."<< endl;

    vector <PointIndices::Ptr> vector_of_segments;

    PointCloud <PointXYZRGB>::Ptr segCloud(new PointCloud <PointXYZRGB>);

    std::tie(vector_of_segments, segCloud) = segmentor(origCloud, normals);

    displayTime();
    PointCloud <PointXYZRGB>::Ptr cloud = vectorToCloud(vector_of_segments, segCloud);
    displayTime();

    ExpandSegmentsToExtents(vector_of_segments,segCloud);

    cout<<"Writing Cloud to File..."<<endl;
    string outputFileName = filename + "-Segmented";
    write(cloud,outputFileName + ".pcd");
    displayTime();

    cout<<"Displaying Cloud..."<< endl;
//    CO.Viewer(cloud);

    cout<<"End"<< endl;
    return 0;
}






