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

void
boundingBox(PointCloud <PointXYZRGB>::Ptr cloud){
      pcl::MomentOfInertiaEstimation <PointXYZRGB> feature_extractor;
      feature_extractor.setInputCloud (cloud);
      feature_extractor.compute ();

      std::vector <float> moment_of_inertia;
      std::vector <float> eccentricity;
      PointXYZRGB min_point_AABB;
      PointXYZRGB max_point_AABB;
      PointXYZRGB min_point_OBB;
      PointXYZRGB max_point_OBB;
      PointXYZRGB position_OBB;
      Eigen::Matrix3f rotational_matrix_OBB;
      float major_value, middle_value, minor_value;
      Eigen::Vector3f major_vector, middle_vector, minor_vector;
      Eigen::Vector3f mass_center;

      feature_extractor.getMomentOfInertia (moment_of_inertia);
      feature_extractor.getEccentricity (eccentricity);
      feature_extractor.getAABB (min_point_AABB, max_point_AABB);
      feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
      feature_extractor.getEigenValues (major_value, middle_value, minor_value);
      feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
      feature_extractor.getMassCenter (mass_center);

      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      viewer->addPointCloud<PointXYZRGB> (cloud, "sample cloud");
      viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

      Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
      Eigen::Quaternionf quat (rotational_matrix_OBB);
      viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

      PointXYZRGB center (mass_center (0), mass_center (1), mass_center (2));
      PointXYZRGB x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
      PointXYZRGB y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
      PointXYZRGB z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
      viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
      viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
      viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");


      while(!viewer->wasStopped())
      {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
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

vector<ModelCoefficients>
ExtractPlaneIntersections(vector <PointIndices::Ptr> indices, PointCloud <PointXYZRGB>::Ptr cloud)
{

//    pcl::visualization::PCLVisualizer viewer;
//    viewer.setBackgroundColor(0,0,0);

//    viewer.addCoordinateSystem (1.0);
    vector<ModelCoefficients> lines_ModCoeff;
    vector<Eigen::VectorXf> lines_Eigen;
//    PointCloud <PointXYZRGB>::Ptr result(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (cloud);
    for (int out=0; out < indices.size(); out++){
        PointIndices::Ptr outerSegment = indices[out];
        PointCloud <PointXYZRGB>::Ptr outerCloud(new PointCloud <PointXYZRGB>);
        filtrerG.setIndices(outerSegment);
        filtrerG.filter(*outerCloud);

        ModelCoefficients::Ptr modelCoeff_outer;
        modelCoeff_outer = FitPlane(outerCloud);

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
            modelCoeff = FitPlane(innerCloud);
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
//        string R;
//        ostringstream convert;
//        convert << out;
//        R = convert.str();
//        *result += *outerCloud;
//        viewer.addPlane(*modelCoeff_outer,0.0,0.0,0.0,R+"P");
        cout<<"YES"<<endl;

    }

//    for (int i=0; i < lines_ModCoeff.size(); i++){
//        string R;
//        ostringstream convert;
//        convert << i;
//        R = convert.str();


//        viewer.addLine(lines_ModCoeff[i],R+"l");
//    }
//    viewer.addPointCloud(result);

//    while (!viewer.wasStopped ())
//    {
//      viewer.spinOnce ();
//    }
    return lines_ModCoeff;

}

void
ExtractPointsOnLines(vector<ModelCoefficients> lines, PointCloud <PointXYZRGB>::Ptr segCloud){


//    double sqr_distance = threshold * threshold;
//    Eigen::Vector3f line_point(l(0), l(1), l(2));
//    Eigen::Vector3f line_direction(l(3), l(4), l(5));

//    for(int i = 0; i < num_points; ++i)
//    {

//            double sqr_distance = (line_point - cloud->points[i].getVector3fMap ()).cross3 (line_direction).squaredNorm ();

//            if (sqr_distance < sqr_threshold)
//            {
//                    //save point at index i, cloud->points[i],  as inlier
//            }
//    }
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

    vector<ModelCoefficients> lines = ExtractPlaneIntersections(vector_of_segments,segCloud);

    ExtractPointsOnLines(lines, segCloud);


//    boundingBox(cloud);

    cout<<"Writing Cloud to File..."<<endl;
    string outputFileName = filename + "-Segmented";
    write(cloud,outputFileName + ".pcd");
    displayTime();

    cout<<"Displaying Cloud..."<< endl;
    Viewer(cloud);

    cout<<"End"<< endl;
    return 0;
}






