#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <typeinfo>
#include <math.h>
#include <pcl/io/pcd_io.h>

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

#include <pcl/surface/concave_hull.h>

using namespace pcl;
using namespace std;

PointCloud<PointXYZRGB>::Ptr
BoundryDetection(PointCloud<PointXYZRGB>::Ptr cloud){

    /*http://pointclouds.org/documentation/tutorials/hull_2d.php*/

    HullConst HC;

    PointCloud<PointXYZRGB>::Ptr plane(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cloudHull(new PointCloud<PointXYZRGB>);

    // Get the plane model, if present.
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    SACSegmentation<PointXYZRGB> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(SACMODEL_PLANE);
    segmentation.setMethodType(SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    PointIndices::Ptr inlierIndices(new PointIndices);
    segmentation.segment(*inlierIndices, *coefficients);


    ExtractIndices<PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inlierIndices);
    extract.filter(*plane);


    ConvexHull<PointXYZRGB> hull;
    hull.setInputCloud(plane);
    hull.setDimension(HC.setDimConvex);
    hull.reconstruct(*cloudHull);


    return cloudHull;
}

PointCloud <PointXYZRGB>::Ptr
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



    PointCloud <PointXYZRGB> returnCloud;


    returnCloud.width=8;

    cout<<"HERE"<<endl;
    returnCloud.height   = 1;
    cout<<"HERE"<<endl;
    returnCloud.is_dense = false;

    returnCloud.points.resize (returnCloud.width * returnCloud.height);

    cout<<"HERE"<<endl;
    returnCloud.points[0].x = p1.x();returnCloud.points[0].y = p1.y();returnCloud.points[0].z = p1.z();
    returnCloud.points[1].x = p2.x();returnCloud.points[1].y = p2.y();returnCloud.points[1].z = p2.z();
    returnCloud.points[2].x = p3.x();returnCloud.points[2].y = p3.y();returnCloud.points[2].z = p3.z();
    returnCloud.points[3].x = p4.x();returnCloud.points[3].y = p4.y();returnCloud.points[3].z = p4.z();
    returnCloud.points[4].x = p5.x();returnCloud.points[4].y = p5.y();returnCloud.points[4].z = p5.z();
    returnCloud.points[5].x = p6.x();returnCloud.points[5].y = p6.y();returnCloud.points[5].z = p6.z();
    returnCloud.points[6].x = p7.x();returnCloud.points[6].y = p7.y();returnCloud.points[6].z = p7.z();
    returnCloud.points[7].x = p8.x();returnCloud.points[7].y = p8.y();returnCloud.points[7].z = p8.z();

    PointCloud<PointXYZRGB>::Ptr returnCloudPtr (new PointCloud<PointXYZRGB> (returnCloud));

    return returnCloudPtr;

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
//    vector <Eigen::Vector3f> OBBOuterPoints;
//    vector <Eigen::Vector3f> OBBInnerPoints;
//    pcl::PointCloud<pcl::PointXYZ> mainOutputPointCloud;


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
//        OBBOuterPoints = boundingBox(outerCloud);

        for (int in=0; in < indices.size(); in++){
            if(in == out){
//                cout<<"skipped"<<endl;
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
//            OBBInnerPoints = boundingBox(innerCloud);

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

        }//end inner loop
    }//end outer loop
    return lines_ModCoeff;

}

PointCloud <PointXYZRGB>::Ptr
cornersOfSegment(PointIndices::Ptr indices, PointCloud <PointXYZRGB>::Ptr cloud){
    PointCloud <PointXYZRGB>::Ptr segment(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filter (true);
    filter.setInputCloud (cloud);
    filter.setIndices(indices);
    filter.filter(*segment);

    PointCloud <PointXYZRGB>::Ptr segmentEDGE = boundingBox(segment);

    return segmentEDGE;
}

PointCloud <PointXYZRGB>::Ptr
ExtractCornerPoints(vector<ModelCoefficients> lines, vector <PointIndices::Ptr> vector_of_segments, PointCloud <PointXYZRGB>::Ptr cloud){
    PointCloud <PointXYZRGB>::Ptr mainCornerPoints;

    for(int i = 0; i < vector_of_segments.size();++i){
        PointCloud <PointXYZRGB>::Ptr corners;
        corners = cornersOfSegment(vector_of_segments[i], cloud);
//        for i in corners:
//            for j in lines:
//                find closest line.
//                project i onto line j.

    }


    return mainCornerPoints;
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

    PointCloud <PointXYZRGB>::Ptr extentCloud = vectorToCloud(vector_of_segments, segCloud);

    vector<ModelCoefficients> lines = ExtractPlaneIntersections(vector_of_segments,segCloud);
    cout<<"got lines..."<<endl;
    cout<<"Extracting corner points..."<<endl;
    ExtractCornerPoints(lines, vector_of_segments, segCloud);

    cout<<"Writing Cloud to File..."<<endl;
    string outputFileName = filename + "-Segmented";
    write(extentCloud,outputFileName + ".pcd");
    displayTime();

    cout<<"Displaying Cloud..."<< endl;
    Viewer(extentCloud);


    cout<<"End"<< endl;
    return 0;
}






