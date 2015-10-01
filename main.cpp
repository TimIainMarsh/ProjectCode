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

void
shadowTest(const vector <PointIndices::Ptr> &indices, const PointCloud <PointXYZRGB>::Ptr &cloud){

    cout<<cloud->sensor_origin_[0]<<" "<<cloud->sensor_origin_[1]<<" "<<cloud->sensor_origin_[2]<<endl;

//    for( int i = 0;1<indices.size();i++){
//        PointCloud <PointXYZRGB>::Ptr corners = boundingBox(cloud);


//    }

}

void
pointsOnLine(const ModelCoefficients::Ptr& line){
    for(float i = -10; i<=10; i = i + 0.2){
            cout<< line->values[0] + (i * line->values[3])<<" "<<line->values[1] + (i * line->values[4])<<" "<<line->values[2] + (i * line->values[5])<<endl;
    }
}

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

PointCloud <PointXYZRGB>
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

    return returnCloud;

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


double
findDistToLine(PointXYZRGB Point, ModelCoefficients::Ptr  line){//checked -- correct.

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

    float A = (PointEigen - line_pt).dot(line_dir - line_pt);
    float B = (line_dir - line_pt).dot(line_dir - line_pt);

    Eigen::Vector3f newP = line_pt + (A/B)*(line_dir - line_pt);


//    A + dot(AP,AB) / dot(AB,AB) * AB

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
ExtractCornerPoints(const vector <PointIndices::Ptr>& vector_of_segments, const vector < PointCloud<PointXYZRGB>::Ptr> &Boundries,const PointCloud <PointXYZRGB>::Ptr& cloud){

    vector<ModelCoefficients::Ptr> lines;

    for(int i = 0; i < vector_of_segments.size();++i){

        PointCloud <PointXYZRGB>::Ptr cloud1 = ExtractSegment(cloud,vector_of_segments[i]);
        ModelCoefficients::Ptr cloud1Coeff = FitPlane(cloud1);


        for(int j = 1; j < vector_of_segments.size();++j){
            if(i == j){continue;}

            PointCloud <PointXYZRGB>::Ptr cloud2 = ExtractSegment(cloud,vector_of_segments[j]);
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
            if(round(plane_a.x()) == round(plane_b.x())){
                if(round(plane_a.y()) == round(plane_b.y())){
                    if(round(plane_a.z()) == round(plane_b.z())){
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
//            lines.push_back(intersection);
//            cout<<"PROJECT"<<endl;

            Project(Boundries[i], intersection);
            Project(Boundries[j], intersection);


//            pointsOnLine(intersection);
        }

    }




}


vector < PointCloud<PointXYZRGB>::Ptr>
getBoundriesOfSegments(const vector <PointIndices::Ptr> &vector_of_segments, const PointCloud <PointXYZRGB>::Ptr &segCloud){
    vector < PointCloud<PointXYZRGB>::Ptr> Boundries;
    for(int i = 0; i < vector_of_segments.size();++i){

        PointCloud <PointXYZRGB>::Ptr c = ExtractSegment(segCloud,vector_of_segments[i]);
        PointCloud <PointXYZRGB> c_Corners = boundingBox(c);
        Boundries.emplace_back(PointCloud<PointXYZRGB>::Ptr (new PointCloud<PointXYZRGB> (c_Corners)));

    }
    return Boundries;
}

//void intersectLines(vector<ModelCoefficients::Ptr> lines){


//    for(int i = 0; i < lines.size();++i){

//        Eigen::VectorXf line_a;
//        line_a.resize(6);

//        for (int k=0;k<6;k++){

//            line_a[k]=lines[i]->values[k];

//        }
//        for (int j =1; j < lines.size();++j){
//            if(i == j){continue;}

//            Eigen::VectorXf line_b;
//            line_b.resize(6);
//            for (int k=0;k<6;k++){
//                line_b[k]=lines[j]->values[k];
//            }

//            Eigen::Vector4f point;
//            lineWithLineIntersection(line_a,line_b,point);
//            if((point[0] != 0) && (point[1] != 0) && (point[2] != 0) ){
//                cout<<point[0]<<" "<<point[1]<<" "<<point[2]<<endl;
//            }

//        }
//    }
//}

int
main()
{
    displayTime();
    cout<<"Start"<< endl;
    string filename = "../ptClouds/GTL-CutDown";


    PointCloud<PointXYZRGB>::Ptr origCloud =  openCloud(filename + ".pcd");

    cout<<"Calculating Normals..."<< endl;
    PointCloud<Normal>::Ptr normals = normalCalc(origCloud);
    cout<<"Normals Calculated...\n"<< endl;

    displayTime();
    cout<<"Segmentation Starting..."<< endl;
    vector <PointIndices::Ptr> vector_of_segments;
    PointCloud <PointXYZRGB>::Ptr segCloud(new PointCloud <PointXYZRGB>);
    std::tie(vector_of_segments, segCloud) = segmentor(origCloud, normals);
    PointCloud <PointXYZRGB>::Ptr extentCloud = vectorToCloud(vector_of_segments, segCloud);
    cout<<"Segmentation Complete..."<< endl;
    displayTime();

//    cout<<  vector_of_segments.size()   <<endl;


    vector < PointCloud<PointXYZRGB>::Ptr> Boundries = getBoundriesOfSegments(vector_of_segments, segCloud);


    cout<<"Extracting corner points..."<<endl;


    ExtractCornerPoints(vector_of_segments,Boundries, segCloud);


    cout<<"HERE"<<endl;
    for(int j = 0; j < Boundries.size();++j){
        for (int i = 0; i < Boundries[j]->points.size(); ++i)
            cout<<Boundries[j]->points[i].x<<"  "<<Boundries[j]->points[i].y<<"  "<<Boundries[j]->points[i].z<<endl;
    }


//    intersectLines(lines);

    cout<<"Writing Cloud to File..."<<endl;
    string outputFileName = filename + "-Segmented";
    write(extentCloud,outputFileName + ".pcd");
    displayTime();

    cout<<"Displaying Cloud..."<< endl;
    Viewer(extentCloud);


    cout<<"End"<< endl;
    return 0;
}


//    string filename = "../ptClouds/box";
//    PointXYZRGB pt;
//    pt.x = 20;
//    pt.y = 20;
//    pt.z = 0;
//    ModelCoefficients::Ptr line(new pcl::ModelCoefficients ());
//    line->values.resize(6);
//    line->values[0] = -1.0;
//    line->values[1] = 0.0;
//    line->values[2] = 0.0;
//    line->values[3] = 1.0;
//    line->values[4] = 0.0;
//    line->values[5] = 0.0;
//    PointXYZRGB s = projectOntoLine(pt,line);
//    cout<<"----->   "<<s.x<<" "<<s.y<<" "<<s.z<<endl;



