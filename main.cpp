#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <typeinfo>
#include <math.h>

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

#include <pcl/common/distances.h>

using namespace pcl;
using namespace std;

vector <Eigen::Vector3f>
boundingBox(PointCloud <PointXYZRGB>::Ptr cloud){
    pcl::MomentOfInertiaEstimation <PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    PointXYZRGB min_point_OBB;
    PointXYZRGB max_point_OBB;
    PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);


    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);


    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat (rotational_matrix_OBB);
    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

//    p1 = rotational_matrix_OBB * p1 + position;
//    p2 = rotational_matrix_OBB * p2 + position;
//    p3 = rotational_matrix_OBB * p3 + position;
//    p4 = rotational_matrix_OBB * p4 + position;
//    p5 = rotational_matrix_OBB * p5 + position;
//    p6 = rotational_matrix_OBB * p6 + position;
//    p7 = rotational_matrix_OBB * p7 + position;
//    p8 = rotational_matrix_OBB * p8 + position;

//    pcl::PointXYZRGB pt1 (p1 (0), p1 (1), p1 (2));
//    pcl::PointXYZRGB pt2 (p2 (0), p2 (1), p2 (2));
//    pcl::PointXYZRGB pt3 (p3 (0), p3 (1), p3 (2));
//    pcl::PointXYZRGB pt4 (p4 (0), p4 (1), p4 (2));
//    pcl::PointXYZRGB pt5 (p5 (0), p5 (1), p5 (2));
//    pcl::PointXYZRGB pt6 (p6 (0), p6 (1), p6 (2));
//    pcl::PointXYZRGB pt7 (p7 (0), p7 (1), p7 (2));
//    pcl::PointXYZRGB pt8 (p8 (0), p8 (1), p8 (2));

    vector <Eigen::Vector3f> points;


    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
    points.push_back(p8);

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->addPointCloud(cloud);
//    viewer->setBackgroundColor(0,0,0);
//    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
//    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

//    viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
//    viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
//    viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
//    viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
//    viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
//    viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
//    viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
//    viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
//    viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
//    viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
//    viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
//    viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");

//    while(!viewer->wasStopped())
//    {
//      viewer->spinOnce (100);
//      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }

    return points;

}


ModelCoefficients::Ptr
FitPlane(PointCloud<PointXYZRGB>::Ptr input_cloud)
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

Eigen::Vector3f
projectOntoLine(Eigen::VectorXf line, Eigen::Vector3f currP){
    Eigen::Vector3f line_pt;
    line_pt.x() = line[0];
    line_pt.y() = line[1];
    line_pt.z() = line[2];

    Eigen::Vector3f line_dir;
    line_dir.x() = line[3];
    line_dir.y() = line[4];
    line_dir.z() = line[5];

    float A = (line_pt - currP).dot(line_pt - line_dir);
    float B = (line_pt - line_dir).dot(line_pt - line_dir);

    Eigen::Vector3f newP = line_pt + (A/B)*(line_pt-line_dir);

//    cout<<"----------------------------------"<<endl;
//    cout<<currP.x()<<" "<<currP.y()<<" "<<currP.z()<<endl;
//    cout<<newP.x()<<" "<<newP.y()<<" "<<newP.z()<<endl;

    return newP;
}

tuple<vector <Eigen::Vector3f> ,vector <Eigen::Vector3f> >
getPointsOnLine(vector <Eigen::Vector3f> OBBOuterPoints, vector <Eigen::Vector3f> OBBInnerPoints, Eigen::VectorXf line){

    int closestPointOuter = 0;\
    int secondClosestOuter = 0;
    float dist = INT8_MAX;
    float prevD = INT8_MAX;

    for (int i=0; i < OBBOuterPoints.size(); i++)
    {
        Eigen::Vector3f pt = OBBOuterPoints[i];
//        cout<<pt.x()<<" "<<pt.y()<<" "<<pt.z()<<" "<<endl;
        Eigen::Vector3f line_pt;
        line_pt.x() = line[0];
        line_pt.y() = line[1];
        line_pt.z() = line[2];

        Eigen::Vector3f line_dir;
        line_dir.x() = line[3];
        line_dir.y() = line[4];
        line_dir.z() = line[5];
        double d = (line_dir.cross(line_pt - pt)).squaredNorm () / line_dir.squaredNorm ();
//       cout<<d<<endl;
        if (d <= prevD ){
            prevD = dist;
            secondClosestOuter = i;
        }

        if (d < dist){
            prevD = dist;
            dist = d;
            closestPointOuter = i;
        }
    }

    int closestPointInner = 0;\
    int secondClosestInner = 0;
    dist = INT8_MAX;
    prevD = INT8_MAX;

    for (int i=0; i < OBBInnerPoints.size(); i++)
    {
        Eigen::Vector3f pt = OBBInnerPoints[i];
//        cout<<pt.x()<<" "<<pt.y()<<" "<<pt.z()<<" "<<endl;
        Eigen::Vector3f line_pt;
        line_pt.x() = line[0];
        line_pt.y() = line[1];
        line_pt.z() = line[2];

        Eigen::Vector3f line_dir;
        line_dir.x() = line[3];
        line_dir.y() = line[4];
        line_dir.z() = line[5];
        double d = (line_dir.cross(line_pt - pt)).squaredNorm () / line_dir.squaredNorm ();

        if (d <= prevD ){
            prevD = dist;
            secondClosestInner = i;
        }

        if (d < dist){
            prevD = dist;
            dist = d;
            closestPointInner = i;
        }
    }



    OBBOuterPoints[closestPointOuter] = projectOntoLine(line,OBBOuterPoints[closestPointOuter]);
    OBBOuterPoints[secondClosestOuter] = projectOntoLine(line,OBBOuterPoints[secondClosestOuter]);
    OBBInnerPoints[closestPointInner] = projectOntoLine(line,OBBInnerPoints[closestPointInner]);
    OBBInnerPoints[secondClosestInner] = projectOntoLine(line,OBBInnerPoints[secondClosestInner]);

    return make_tuple(OBBOuterPoints,OBBInnerPoints);
}



vector<ModelCoefficients>
ExtractPlaneIntersections(vector <PointIndices::Ptr> indices, PointCloud <PointXYZRGB>::Ptr cloud)
{

    vector<ModelCoefficients> lines_ModCoeff;

    for (int out=0; out < indices.size(); out++){
        PointIndices::Ptr outerSegment = indices[out];
        PointCloud <PointXYZRGB>::Ptr outerCloud(new PointCloud <PointXYZRGB>);
        ExtractIndices<PointXYZRGB> filtrerOuter (true);
        filtrerOuter.setInputCloud (cloud);
        filtrerOuter.setIndices(outerSegment);
        filtrerOuter.filter(*outerCloud);
//        Viewer(outerCloud);
        ModelCoefficients::Ptr OuterCoeff = FitPlane(outerCloud);
        vector <Eigen::Vector3f> OBBOuterPoints = boundingBox(outerCloud);

        for (int in=0; in < indices.size(); in++){
            if(in == out){
                cout<<"skipped"<<endl;
                continue;                
            }
            PointIndices::Ptr innerSegment = indices[in];
            PointCloud <PointXYZRGB>::Ptr innerCloud(new PointCloud <PointXYZRGB>);
            ExtractIndices<PointXYZRGB> filtrerInner (true);
            filtrerInner.setInputCloud (cloud);
            filtrerInner.setIndices(innerSegment);
            filtrerInner.filter(*innerCloud);
//            Viewer(innerCloud);


            ModelCoefficients::Ptr InnerCoeff = FitPlane(innerCloud);
            vector <Eigen::Vector3f> OBBInnerPoints = boundingBox(innerCloud);

            double angular_tolerance=0.0;
            Eigen::Vector4f plane_a;
            plane_a.x()=InnerCoeff->values[0];
            plane_a.y()=InnerCoeff->values[1];
            plane_a.z()=InnerCoeff->values[2];
            plane_a.w()=InnerCoeff->values[3];
            Eigen::Vector4f plane_b;
            plane_b.x()=OuterCoeff->values[0];
            plane_b.y()=OuterCoeff->values[1];
            plane_b.z()=OuterCoeff->values[2];
            plane_b.w()=OuterCoeff->values[3];

            Eigen::VectorXf line;
            pcl::planeWithPlaneIntersection(plane_a,plane_b,line,angular_tolerance);
            pcl::ModelCoefficients::Ptr l(new pcl::ModelCoefficients ());
            l->values.resize(6);
            for (int i=1;i<6;i++)
            {
                l->values[i]=line[i];
            }

            lines_ModCoeff.push_back(*l);

            //find which two points from each vector of OBB are closest to line
            //and project them on the line



            tie(OBBOuterPoints,OBBInnerPoints) = getPointsOnLine(OBBOuterPoints,OBBInnerPoints, line);



        }//end inner loop





    }//end outer loop



    return lines_ModCoeff;

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

    vector<ModelCoefficients> lines = ExtractPlaneIntersections(vector_of_segments,segCloud);

//    cout<<"Writing Cloud to File..."<<endl;
//    string outputFileName = filename + "-Segmented";
//    write(cloud,outputFileName + ".pcd");
//    displayTime();

//    cout<<"Displaying Cloud..."<< endl;
//    Viewer(cloud);

    cout<<"End"<< endl;
    return 0;
}






