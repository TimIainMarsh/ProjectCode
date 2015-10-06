#include "projection.h"

#include "timedate.h"
#include "Functions.h"
#include "cloudoperations.h"
#include "constants.h"
#include "segmentor.h"

#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <typeinfo>
#include <math.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/io.h>
#include <pcl/point_types.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>

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
pointsOnLine(const ModelCoefficients::Ptr& line){
    for(float i = -10; i<=10; i = i + 0.2){
            cout<< line->values[0] + (i * line->values[3])<<" "<<line->values[1] + (i * line->values[4])<<" "<<line->values[2] + (i * line->values[5])<<endl;
    }
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

    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);

    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);



    p1 = rotational_matrix_OBB * p1 + position;

    p3 = rotational_matrix_OBB * p3 + position;

    p5 = rotational_matrix_OBB * p5 + position;

    p7 = rotational_matrix_OBB * p7 + position;



    PointCloud <PointXYZRGB> returnCloud;

    returnCloud.width = 4;

    returnCloud.height = 1;

    returnCloud.is_dense = false;

    returnCloud.points.resize (returnCloud.width * returnCloud.height);

    returnCloud.points[0].x = p1.x();returnCloud.points[0].y = p1.y();returnCloud.points[0].z = p1.z();

    returnCloud.points[1].x = p3.x();returnCloud.points[1].y = p3.y();returnCloud.points[1].z = p3.z();

    returnCloud.points[2].x = p5.x();returnCloud.points[2].y = p5.y();returnCloud.points[2].z = p5.z();

    returnCloud.points[3].x = p7.x();returnCloud.points[3].y = p7.y();returnCloud.points[3].z = p7.z();


//    PointCloud<PointXYZRGB>::Ptr returnCloudPtr (new PointCloud<PointXYZRGB> (returnCloud));

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

findDistToLine(PointXYZRGB Point, ModelCoefficients::Ptr  line){//correct

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
projectOntoLine(PointXYZRGB Point,ModelCoefficients::Ptr  line){

    /// Currpoint in right format////////////////////////////////////////
    Eigen::Vector3f PointEigen;
    PointEigen.x() = Point.x;
    PointEigen.y() = Point.y;
    PointEigen.z() = Point.z;

    ////////////////////////////////////////////

    //line point in right format/////////////////////////////////////////
    Eigen::Vector3f line_pt;
    line_pt.x() = line->values[0];
    line_pt.y() = line->values[1];
    line_pt.z() = line->values[2];

    /////////////////////////////////////////

    //line dir in right format/////////////////////////////////////////
    Eigen::Vector3f line_dir;
    line_dir.x() = line->values[3];
    line_dir.y() = line->values[4];
    line_dir.z() = line->values[5];

    /////////////////////////////////////////


    float A = (PointEigen).dot(line_dir);
    float B = (line_dir).dot(line_dir);

    Eigen::Vector3f newP =(A/B) * (line_dir);


    PointXYZRGB newPoint;
    newPoint.x = newP.x() + line->values[0];
    newPoint.y = newP.y() + line->values[1];
    newPoint.z = newP.z() + line->values[2];


    return newPoint;
}


void
Project( const PointCloud <PointXYZRGB>::Ptr& corner_cloud, const ModelCoefficients::Ptr& intersection){

    int min1 = -1;
    int min2 = -1;
    double min1D = 1000;
    double min2D = 1000;


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

double
angleBetweenPlanes(Eigen::Vector4f plane_a,Eigen::Vector4f plane_b){
    Eigen::Vector3f norm_a;
    Eigen::Vector3f norm_b;
    for (int i =0; i< 3;i++){
        norm_a[i] = plane_a[i];
        norm_b[i] = plane_b[i];
    }

    double A = norm_a.dot(norm_b) / (norm_a.norm() * norm_a.norm());

    return acos(A) * 180/M_PI;
}

void
ExtractCornerPoints(const vector <PointIndices::Ptr> &vector_of_segments,  const vector <PointIndices::Ptr> &vector_of_roof_floor, const vector < PointCloud<PointXYZRGB>::Ptr> &Boundries,const PointCloud <PointXYZRGB>::Ptr &cloud){

    //////////////////////////////////////////////////////////////////////////////////////
    /// iterrates over all the segments and projects the closest two point of the outer
    /// loop to the line of itersection
    ///
    ///
    /// modifies boundries
    //////////////////////////////////////////////////////////////////////////////////////

    for(int i = 0; i < vector_of_roof_floor.size();i++){
        PointCloud <PointXYZRGB>::Ptr cloud1 = ExtractSegment(cloud,vector_of_roof_floor[i]);
        ModelCoefficients::Ptr cloud1Coeff = FitPlane(cloud1);
        cout<<i<<endl;
        for(int j = 0; j < vector_of_segments.size();j++){


            PointCloud <PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);
            cloud2 = ExtractSegment(cloud,vector_of_segments[j]);
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



            if(angleBetweenPlanes(plane_a,plane_b) < 15.0){
                //making sure planes are not parallel or too close to it
                continue;
             }


            Eigen::VectorXf line;
            pcl::planeWithPlaneIntersection(plane_a,plane_b,line,angular_tolerance);
            pcl::ModelCoefficients::Ptr intersection(new pcl::ModelCoefficients ());
            intersection->values.resize(6);


            for (int k=0;k<6;k++){intersection->values[k]=line[k];}


//            pcl::ModelCoefficients thing;
//            thing.values.resize(6);
//            for (int k=0;k<6;k++){
//                thing.values[k] = intersection->values[k];
//            }


//            pcl::visualization::PCLVisualizer viewer("Cloud");
//            viewer.addLine(thing);


//            viewer.addPointCloud<pcl::PointXYZRGB> (cloud1,"1");
//            viewer.addPointCloud<pcl::PointXYZRGB> (cloud2,"1.5");

////            viewer.addPointCloud<pcl::PointXYZRGB> (Boundries[i],"2");
////            cout<<i<<" "<<j<<endl;

//            for (int i=0;i<6;i++){intersection->values[i]=line[i];}


            Project(Boundries[j], intersection);

//            viewer.addPointCloud<pcl::PointXYZRGB> (Boundries[i],"3");

//            while (!viewer.wasStopped ())
//            {
//              viewer.spinOnce ();
//            }
//            viewer.close();
//            Project(Boundries[j], intersection);


//            pointsOnLine(intersection);

        }//end of first loop

    }//end of second loop


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
