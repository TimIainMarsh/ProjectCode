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
boundingBox(PointCloud <PointXYZRGB>::Ptr cloud,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,string s){
    pcl::MomentOfInertiaEstimation <PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    PointXYZRGB min_point_OBB;
    PointXYZRGB max_point_OBB;
    PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);

    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

    p1 = rotational_matrix_OBB * p1 + position;
    p2 = rotational_matrix_OBB * p2 + position;
    p3 = rotational_matrix_OBB * p3 + position;
    p4 = rotational_matrix_OBB * p4 + position;
    p5 = rotational_matrix_OBB * p5 + position;
    p6 = rotational_matrix_OBB * p6 + position;
    p7 = rotational_matrix_OBB * p7 + position;
    p8 = rotational_matrix_OBB * p8 + position;

    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

     viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge"+s);
     viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge"+s);
     viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge"+s);
     viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge"+s);
     viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge"+s);
     viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge"+s);
     viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge"+s);
     viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge"+s);
     viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge"+s);
     viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge"+s);
     viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge"+s);
     viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge"+s);
     while(!viewer->wasStopped())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     }


    PointCloud <PointXYZRGB> returnCloud;


    returnCloud.width = 4;

    returnCloud.height = 1;

    returnCloud.is_dense = false;

    returnCloud.points.resize (returnCloud.width * returnCloud.height);

    returnCloud.points[0].x = p1.x();returnCloud.points[0].y = p1.y();returnCloud.points[0].z = p1.z();
//    returnCloud.points[1].x = p2.x();returnCloud.points[1].y = p2.y();returnCloud.points[1].z = p2.z();
    returnCloud.points[1].x = p3.x();returnCloud.points[1].y = p3.y();returnCloud.points[1].z = p3.z();
//    returnCloud.points[3].x = p4.x();returnCloud.points[3].y = p4.y();returnCloud.points[3].z = p4.z();
    returnCloud.points[2].x = p5.x();returnCloud.points[2].y = p5.y();returnCloud.points[2].z = p5.z();
//    returnCloud.points[5].x = p6.x();returnCloud.points[5].y = p6.y();returnCloud.points[5].z = p6.z();
    returnCloud.points[3].x = p7.x();returnCloud.points[3].y = p7.y();returnCloud.points[3].z = p7.z();
//    returnCloud.points[7].x = p8.x();returnCloud.points[7].y = p8.y();returnCloud.points[7].z = p8.z();

    PointCloud<PointXYZRGB>::Ptr returnCloudPtr (new PointCloud<PointXYZRGB> (returnCloud));

    for (int i = 0; i < returnCloudPtr->points.size(); ++i)
        cout<<returnCloudPtr->points[i].x<<"  "<<returnCloudPtr->points[i].y<<"  "<<returnCloudPtr->points[i].z<<endl;


//    Viewer(returnCloudPtr);

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
cornersOfSegment(PointIndices::Ptr indices, PointCloud <PointXYZRGB>::Ptr cloud,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,string s){
    PointCloud <PointXYZRGB>::Ptr segment(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filter (true);
    filter.setInputCloud (cloud);
    filter.setIndices(indices);
    filter.filter(*segment);

    PointCloud <PointXYZRGB>::Ptr segmentEDGE = boundingBox(segment,viewer,s);

    return segmentEDGE;
}


ModelCoefficients
findCloseLine(PointXYZRGB Point, vector<ModelCoefficients> lines){
//    cout<<"Start"<<endl;

//    cout<<Point.x<<"   "<<Point.y<<"   "<<Point.z<<endl;

    double closeDist = 100;
    int indexOfClose = -1;
    for(int i = 0; i < lines.size(); ++i){
        /// Currpoint in right format//////////////////////////////////////////
        Eigen::Vector4f PointEigen;
        PointEigen.x()= Point.x;
        PointEigen.y()= Point.y;
        PointEigen.z()= Point.z;
        PointEigen.w()= 0;
        ////////////////////////////////////////////

        //line point in right format/////////////////////////////////////////
        Eigen::Vector4f line_pt;
        line_pt.x()= lines[i].values[0];
        line_pt.y()= lines[i].values[1];
        line_pt.z()= lines[i].values[2];
        line_pt.w()= 0;
        /////////////////////////////////////////

        //line dirn in right format/////////////////////////////////////////
        Eigen::Vector4f line_dir;
        line_dir.x()= lines[i].values[3];
        line_dir.y()= lines[i].values[4];
        line_dir.z()= lines[i].values[5];
        line_dir.w()= 0;
        /////////////////////////////////////////

        double f = sqrPointToLineDistance(PointEigen, line_pt, line_dir);
        if (f < closeDist){
            closeDist = f;
            indexOfClose = i;
            }

        }

//    cout<<"-->"<<closeDist<<endl;
    return lines[indexOfClose];
}

PointCloud <PointXYZRGB>::Ptr
ExtractCornerPoints(vector<ModelCoefficients> lines, vector <PointIndices::Ptr> vector_of_segments, PointCloud <PointXYZRGB>::Ptr cloud){
    PointCloud <PointXYZRGB>::Ptr mainCornerPoints;


//    visualization::PCLVisualizer viewer("TEST");
         boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    for(int i = 0; i < vector_of_segments.size();++i){
        PointCloud <PointXYZRGB>::Ptr corners(new PointCloud <PointXYZRGB>);
        std::string s = std::to_string(i);
        corners = cornersOfSegment(vector_of_segments[i], cloud,viewer,s);
        cout<<i<<endl;

//        viewer.addPointCloud(corners,s);

//        for i in corners:
        for (int j = 0; j < corners->points.size();++j){
            PointXYZRGB currPoint = corners->points[j];

//            for j in lines:
            for(int k = 0; k < lines.size(); ++k){

//                find closest line.
                ModelCoefficients Closestline = findCloseLine(currPoint, lines);

//                project i onto line j.


            }

        }

    }
//    viewer.setBackgroundColor (1.0, 1.0, 1.0);
//    while (!viewer.wasStopped ())
//    {
//      viewer.spinOnce ();
//    }

    return mainCornerPoints;
}




int
main()
{
    displayTime();
    cout<<"Start"<< endl;
//    string filename = "../ptClouds/DeepSpace-CutDown";
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
    cout<<"Got lines..."<<endl;
    cout<<"Extracting corner points..."<<endl;

    PointCloud <PointXYZRGB>::Ptr cornersOfCloud = ExtractCornerPoints(lines, vector_of_segments, segCloud);

//    cout<<"Writing Cloud to File..."<<endl;
//    string outputFileName = filename + "-Segmented";
//    write(extentCloud,outputFileName + ".pcd");
    displayTime();

    cout<<"Displaying Cloud..."<< endl;
    Viewer(cornersOfCloud);


    cout<<"End"<< endl;
    return 0;
}






