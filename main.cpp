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



    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
//    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
//    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
//    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
//    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);


    p1 = rotational_matrix_OBB * p1 + position;
//    p2 = rotational_matrix_OBB * p2 + position;
    p3 = rotational_matrix_OBB * p3 + position;
//    p4 = rotational_matrix_OBB * p4 + position;
    p5 = rotational_matrix_OBB * p5 + position;
//    p6 = rotational_matrix_OBB * p6 + position;
    p7 = rotational_matrix_OBB * p7 + position;
//    p8 = rotational_matrix_OBB * p8 + position;


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

    return returnCloudPtr;

}



ModelCoefficients::Ptr
FitPlane(PointCloud<PointXYZRGB>::Ptr input_cloud)
{


    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    SACSegmentation<PointXYZRGB> segmentation;
    segmentation.setInputCloud(input_cloud);
    segmentation.setModelType(SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    PointIndices::Ptr inlierIndices(new PointIndices);
    segmentation.segment(*inlierIndices, *coefficients);

    return coefficients;

}

PointCloud <PointXYZRGB>::Ptr
ExtractSegment(PointCloud <PointXYZRGB>::Ptr cloud, PointIndices::Ptr segInd){
    PointCloud <PointXYZRGB>::Ptr outCloud(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filtrerOuter (true);
    filtrerOuter.setInputCloud (cloud);
    filtrerOuter.setIndices(segInd);
    filtrerOuter.filter(*outCloud);

    return outCloud;
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


double
findDistToLine(PointXYZRGB Point, ModelCoefficients::Ptr  line){

    /// Currpoint in right format//////////////////////////////////////////
    Eigen::Vector4f PointEigen;
    PointEigen.x()= Point.x;
    PointEigen.y()= Point.y;
    PointEigen.z()= Point.z;
    PointEigen.w()= 0;
    ////////////////ExtractSegment////////////////////////////

    //line point in right format/////////////////////////////////////////
    Eigen::Vector4f line_pt;
    line_pt.x()= line->values[0];
    line_pt.y()= line->values[1];
    line_pt.z()= line->values[2];
    line_pt.w()= 0;
    /////////////////////////////////////////

    //line dirn in right format/////////////////////////////////////////
    Eigen::Vector4f line_dir;
    line_dir.x()= line->values[3];
    line_dir.y()= line->values[4];
    line_dir.z()= line->values[5];
    line_dir.w()= 0;
    /////////////////////////////////////////

    double f = sqrPointToLineDistance(PointEigen, line_pt, line_dir);
    return sqrt(f);
}

PointXYZRGB
projectOntoLine(PointXYZRGB Point,ModelCoefficients::Ptr  line){// CHANCES ARE ISSUE IS IN HERE

    /// Currpoint in right format////////////////////////////////////////
    Eigen::Vector3f PointEigen;
    PointEigen.x() = Point.x;
    PointEigen.y() = Point.y;
    PointEigen.z() = Point.z;
//    PointEigen.w()= 0;
    ////////////////////////////////////////////

    //line point in right format/////////////////////////////////////////
    Eigen::Vector3f line_pt;
    line_pt.x() = line->values[0];
    line_pt.y() = line->values[1];
    line_pt.z() = line->values[2];
//    line_pt.w()= 0;
    /////////////////////////////////////////

    //line dir in right format/////////////////////////////////////////
    Eigen::Vector3f line_dir;
    line_dir.x() = line->values[3];
    line_dir.y() = line->values[4];
    line_dir.z() = line->values[5];
//    line_dir.w()= 0;
    /////////////////////////////////////////

    float A = (line_pt - PointEigen).dot(line_pt - line_dir);
    float B = (line_pt - line_dir).dot(line_pt - line_dir);

    Eigen::Vector3f newP = line_pt + (A/B)*(line_pt-line_dir);

    PointXYZRGB newPoint;
    newPoint.x = newP.x();
    newPoint.y = newP.y();
    newPoint.z = newP.z();


    return newPoint;
}





void
Project( const PointCloud <PointXYZRGB>::Ptr& corner_cloud, const ModelCoefficients::Ptr& intersection){

    int min1 = 0;
    int min2 = 1;
    double min1D = INT8_MAX;
    double min2D = INT8_MAX;

    if(findDistToLine(corner_cloud->points[1], intersection) < findDistToLine(corner_cloud->points[0], intersection)){
        min1 = 1;
        min2 = 0;
    }
    for (int i = 0; i<corner_cloud->points.size();++i){
        double dist = findDistToLine(corner_cloud->points[i], intersection);
        if(dist< min1D){
            min2 = min1;
            min2D = min1D;
            min1 = i;
            min1D = dist;
        }
        else if(dist<min2D){
            min2 = i;
            min2D = dist;
        }
    }
//    cout<<min1<<" "<<min1D<<" "<<min2<<" "<<min2D<<endl;
    PointXYZRGB inter1;
    inter1 = projectOntoLine(corner_cloud->points[min1], intersection);
    corner_cloud->points[min1].x = inter1.x;
    corner_cloud->points[min1].y = inter1.y;
    corner_cloud->points[min1].z = inter1.z;

    PointXYZRGB inter2;
    inter2 = projectOntoLine(corner_cloud->points[min2], intersection);
    corner_cloud->points[min2].x = inter2.x;
    corner_cloud->points[min2].y = inter2.y;
    corner_cloud->points[min2].z = inter2.z;

}

void
pointsOnLine(const ModelCoefficients::Ptr& line){

    for(float i = -10; i<=10; i = i + 0.2){

            cout<< line->values[0] + (i * line->values[3])<<" "<<line->values[1] + (i * line->values[4])<<" "<<line->values[2] + (i * line->values[5])<<endl;

    }


}


PointCloud <PointXYZRGB>::Ptr
ExtractCornerPoints(const vector <PointIndices::Ptr>& vector_of_segments,const PointCloud <PointXYZRGB>::Ptr& cloud){
    PointCloud <PointXYZRGB>::Ptr mainCornerPoints;

//    vector <tuple<PointCloud <PointXYZRGB>::Ptr, PointCloud <PointXYZRGB>::Ptr> > SegmentAndBoundry;

//    for(int i = 0; i < vector_of_segments.size();++i){
//        PointCloud <PointXYZRGB>::Ptr cloud = ExtractSegment(cloud,vector_of_segments[i]);
//        PointCloud <PointXYZRGB>::Ptr cloud_Corners = boundingBox(cloud);
//        cout<<"HERE"<<endl;




////        SegmentAndBoundry.push_back(tuple<PointCloud <PointXYZRGB>::Ptr, PointCloud <PointXYZRGB>::Ptr>(cloud,cloud_Corners));
//    }


    for(int i = 0; i < vector_of_segments.size();++i){

        PointCloud <PointXYZRGB>::Ptr cloud1 = ExtractSegment(cloud,vector_of_segments[i]);
        PointCloud <PointXYZRGB>::Ptr cloud_1_Corners = boundingBox(cloud1);
        ModelCoefficients::Ptr cloud1Coeff = FitPlane(cloud1);


        for(int j = 0; j < vector_of_segments.size();++j){
            if(i == j){continue;}

            PointCloud <PointXYZRGB>::Ptr cloud2 = ExtractSegment(cloud,vector_of_segments[j]);
//            PointCloud <PointXYZRGB>::Ptr cloud_2_Corners = boundingBox(cloud2);
            ModelCoefficients::Ptr cloud2Coeff = FitPlane(cloud2);

            double angular_tolerance=0.0;
            Eigen::Vector4f plane_a;
            plane_a.x()=cloud1Coeff->values[0];
            plane_a.y()=cloud1Coeff->values[1];
            plane_a.z()=cloud1Coeff->values[2];
            plane_a.w()=cloud1Coeff->values[3];
            Eigen::Vector4f plane_b;
            plane_b.x()=cloud2Coeff->values[0];
            plane_b.y()=cloud2Coeff->values[1];
            plane_b.z()=cloud2Coeff->values[2];
            plane_b.w()=cloud2Coeff->values[3];

            Eigen::VectorXf line;

            //parrallel check//////////////////////////////////////////////////////////
            if(floor(plane_a.x() * 3)/3 == floor(plane_b.x() * 3)/3){
                if(floor(plane_a.y() * 3)/3 == floor(plane_b.y() * 3)/3){
                    if(floor(plane_a.z() * 3)/3 == floor(plane_b.z() * 3)/3){
//                        cout<<"skip"<<endl;
                        continue;
                    }
                }
            }
            //////////////////////////////////////////////////////////////////////////

            pcl::planeWithPlaneIntersection(plane_a,plane_b,line,angular_tolerance);
            pcl::ModelCoefficients::Ptr intersection(new pcl::ModelCoefficients ());
            intersection->values.resize(6);
            for (int i=0;i<6;i++){intersection->values[i]=line[i];}
//            cout<<"PROJECT"<<endl;
            Project(cloud_1_Corners, intersection);


//            pointsOnLine(intersection);
//            for (int i = 0; i < cloud_2_Corners->points.size(); ++i)
//                cout<<cloud_2_Corners->points[i].x<<"  "<<cloud_2_Corners->points[i].y<<"  "<<cloud_2_Corners->points[i].z<<endl;

        }
        for (int i = 0; i < cloud_1_Corners->points.size(); ++i)
            cout<<cloud_1_Corners->points[i].x<<"  "<<cloud_1_Corners->points[i].y<<"  "<<cloud_1_Corners->points[i].z<<endl;

    }
    return mainCornerPoints;
}

int
main()
{
    displayTime();
    cout<<"Start"<< endl;
    string filename = "../ptClouds/DeepSpace-CutDown";
//    string filename = "../ptClouds/box";

    PointCloud<PointXYZRGB>::Ptr origCloud =  openCloud(filename + ".pcd");

    cout<<"Calculating Normals..."<< endl;
    PointCloud<Normal>::Ptr normals = normalCalc(origCloud);
    cout<<"Normals Calculated..."<< endl;


    cout<<"Segmentation Starting..."<< endl;
    displayTime();
    vector <PointIndices::Ptr> vector_of_segments;
    PointCloud <PointXYZRGB>::Ptr segCloud(new PointCloud <PointXYZRGB>);
    std::tie(vector_of_segments, segCloud) = segmentor(origCloud, normals);
    PointCloud <PointXYZRGB>::Ptr extentCloud = vectorToCloud(vector_of_segments, segCloud);
    displayTime();
    cout<<"Segmentation Complete..."<< endl;



//    vector<ModelCoefficients> lines = ExtractPlaneIntersections(vector_of_segments,segCloud);
    cout<<"Extracting corner points..."<<endl;


    PointCloud <PointXYZRGB>::Ptr cornersOfCloud = ExtractCornerPoints(vector_of_segments, segCloud);

    cout<<"Writing Cloud to File..."<<endl;
    string outputFileName = filename + "-Segmented";
    write(extentCloud,outputFileName + ".pcd");
    displayTime();

    cout<<"Displaying Cloud..."<< endl;
    Viewer(extentCloud);


    cout<<"End"<< endl;
    return 0;
}






