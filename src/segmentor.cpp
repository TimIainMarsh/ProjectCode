#include "segmentor.h"

#include <iostream>
#include <vector>
#include <string>
#include <tuple>


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

using namespace pcl;
using namespace std;


int
removeClusterOnVerticality(const PointCloud<PointXYZRGB>::Ptr& input_cloud, const PointIndices::Ptr& cluster)
{

    if (cluster->indices.size() == 0)
        return 0;

    //////////////////////////////////////////////////////////////////////////
    /// Fitting a plae and taking the a,b,c values as normal to the plane
    /// should probaby be done with PCA for speed but for now it works
    ///
    /// if angle from vert greater than set value segment rejected
    ///
    //////////////////////////////////////////////////////////////////////////

    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    SACSegmentation<PointXYZRGB> segmentation;
    segmentation.setInputCloud(input_cloud);
    segmentation.setIndices(cluster);
    segmentation.setModelType(SACMODEL_PLANE);
    segmentation.setMethodType(SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    PointIndices::Ptr inlierIndices(new PointIndices);
    segmentation.segment(*inlierIndices, *coefficients);

    //Model Coeff and vert axis creating angle from vertical

    Eigen::Vector3f plane_normal(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
    Eigen::Vector3f vert_axis(0.0,0.0,1.0);

    float angleFromVert = acos(plane_normal.dot(vert_axis))* 180.0/M_PI;
//    cout<<angleFromVert<<endl;
    if (angleFromVert >= 30.0) {
        return 1;
    }
    else {
        return 0;
    }

}


int
removeClusterOnSize(const PointCloud<PointXYZRGB>::Ptr& input_cloud, const PointIndices::Ptr& cluster)
{

    if (cluster->indices.size() == 0)
        return 0;

    ///////////////////////////////////////////////////////////////////////////
    ///
    /// detemining the heght of the segment
    /// if it is bigger than set height then
    /// reject them if the are too big.
    ///
    /// done by finiding max Z and min Z then differencing
    ///
    //////////////////////////////////////////////////////////////////////////


    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (input_cloud);
    PointCloud <PointXYZRGB>::Ptr cloud(new PointCloud <PointXYZRGB>);
    filtrerG.setIndices(cluster);
    filtrerG.filter(*cloud);


    float max_Z = cloud->points[0].z;
    float min_Z = cloud->points[0].z;

    for(int i = 0; i < cloud->points.size(); i++ ) {
        if(cloud->points[i].z >= max_Z) {
            max_Z = cloud->points[i].z;
        }
    }

    for(int j = 0; j < cloud->points.size(); j++ ) {
        if(cloud->points[j].z <= min_Z) {
            min_Z = cloud->points[j].z;
        }
    }

    float gap = max_Z - min_Z;

    gap = sqrt(gap*gap);

//    cout<<min_Z<<" - "<<max_Z<<" - "<<gap<<endl;

    if(gap > 1.00) {
        return 1;
    }
    else {
        return 0;
    }
}


int
GetHORclusters(const PointCloud<PointXYZRGB>::Ptr& input_cloud, const PointIndices::Ptr& cluster) {

    if (cluster->indices.size() == 0)
        return 0;

    //////////////////////////////////////////////////////////////////////////
    /// Fitting a plae and taking the a,b,c values as normal to the plane
    /// should probaby be done with PCA for speed but for now it works
    ///
    /// if angle from vert greater than set value segment rejected
    ///
    //////////////////////////////////////////////////////////////////////////

    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    SACSegmentation<PointXYZRGB> segmentation;
    segmentation.setInputCloud(input_cloud);
    segmentation.setIndices(cluster);
    segmentation.setModelType(SACMODEL_PLANE);
    segmentation.setMethodType(SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    PointIndices::Ptr inlierIndices(new PointIndices);
    segmentation.segment(*inlierIndices, *coefficients);

    //Model Coeff and vert axis creating angle from vertical

    Eigen::Vector3f plane_normal(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
    Eigen::Vector3f vert_axis(0.0,0.0,1.0);

    float angleFromVert = acos(plane_normal.dot(vert_axis))* 180.0/M_PI;
//    cout<<angleFromVert<<endl;
    if (angleFromVert <= 60.0) {
        return 1;
    }
    else {
        return 0;
    }

}

float
GetMaxOfSeg(const PointCloud<PointXYZRGB>::Ptr& input_cloud, const PointIndices::Ptr& cluster)
{


    ///////////////////////////////////////////////////////////////////////////
    ///
    /// getting max value in cluster
    ///
    //////////////////////////////////////////////////////////////////////////


    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (input_cloud);
    PointCloud <PointXYZRGB>::Ptr cloud(new PointCloud <PointXYZRGB>);
    filtrerG.setIndices(cluster);
    filtrerG.filter(*cloud);

    float max_Z = cloud->points[0].z;

    for(int i = 0; i < cloud->points.size(); i++ ) {
        if(cloud->points[i].z >= max_Z) {
            max_Z = cloud->points[i].z;
        }
    }
    return max_Z;
}

float
GetMinOfSeg(const PointCloud<PointXYZRGB>::Ptr& input_cloud, const PointIndices::Ptr& cluster)
{


    ///////////////////////////////////////////////////////////////////////////
    ///
    /// getting min value in cluster
    ///
    //////////////////////////////////////////////////////////////////////////


    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (input_cloud);
    PointCloud <PointXYZRGB>::Ptr cloud(new PointCloud <PointXYZRGB>);
    filtrerG.setIndices(cluster);
    filtrerG.filter(*cloud);

    float min_Z = cloud->points[0].z;

    for(int i = 0; i < cloud->points.size(); i++ ) {
        if(cloud->points[i].z <= min_Z) {
            min_Z = cloud->points[i].z;
        }
    }
    return min_Z;
}

tuple<  vector <PointIndices::Ptr> , PointCloud<PointXYZRGB>::Ptr  >
segmentor(const PointCloud<PointXYZRGB>::Ptr& input_cloud, const PointCloud<Normal>::Ptr& normals) {
    //////////////////////////////////////////////////////////////////////////////////
    ///
    /// most of the region growing section coppied from here:
    ///http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php
    ///
    //////////////////////////////////////////////////////////////////////////////////
    RegionGrowingConst RGC;

    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);

    IndicesPtr indices (new vector <int>);
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud (input_cloud);

    pass.filter (*indices);

    RegionGrowing<PointXYZRGB, Normal> reg;

    reg.setMinClusterSize (RGC.MinClusterSize);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (RGC.NumberOfNeighbours);
    reg.setInputCloud (input_cloud);
    reg.setIndices (indices);
    reg.setInputNormals (normals);

    if (!RGC.fast) {
        reg.setSmoothnessThreshold (RGC.SmoothnessThreshold);
        reg.setCurvatureThreshold (RGC.CurvatureThreshold);
    }
    vector <PointIndices> clusters;
//    cout<<"Cluster Extracton Starting..."<< endl;
    reg.extract (clusters);
//    cout<<"Cluster Extracton Sucessfull..."<< endl;

    vector <PointIndices::Ptr> my_clusters;

    for (int i=0; i < clusters.size(); i++)
    {
        PointIndices::Ptr tmp_clusterR(new PointIndices(clusters[i]));
        my_clusters.push_back(tmp_clusterR);
    }


    PointCloud <PointXYZRGB>::Ptr segCloud = reg.getColoredCloud(); //replaces the input cloud with one coloured accoring to segments

    //////////////////////////////////////////////////////////////////////////
    ///
    /// Cluster rejecting section for VERTICAL
    ///
    /// checks for size then for verticality.
    /// the size and verticality return a 1 if the cluster succeeds
    /// if it fails it sets the cluster to 0 and removes the cluster later
    ///
    /////////////////////////////////////////////////////////////////////////

    cout<<" "<<endl;
//    cout<<" Staring Cluster Removal .. "<<endl;

    vector <PointIndices::Ptr> my_VERT_clusters;
//    vector <PointIndices::Ptr> my_HOR_clusters;

    for (int i=0; i < clusters.size(); i++)
    {
//        cout<<"----------------------Start Cluster"<<endl;
//        cout<<"-checking VERT"<<endl;
        if (removeClusterOnVerticality(segCloud,my_clusters[i]) != 1) {
//            cout<<"Removed cluster after VERT check..."<<endl;
            continue;
        }
//        cout<<"-checking SIZE"<<endl;
        if (removeClusterOnSize(segCloud,my_clusters[i]) != 1) {
//            cout<<"Removed cluster after SIZE check..."<<endl;
            continue;
        }
        else {
            my_VERT_clusters.push_back(my_clusters[i]);
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    ///
    /// Cluster rejecting section for HORIZONTAL
    ///
    ///////////////////////////////////////////////////////////////////////////


    vector <PointIndices::Ptr> my_HOR_clusters;

    for (int i=0; i < clusters.size(); i++)
    {
        if (GetHORclusters(segCloud,my_clusters[i]) == 1) {
            my_HOR_clusters.push_back(my_clusters[i]);
            continue;
        }
    }
    int maxSeg = -1;
    float maxSegHeight = -100;
    int minSeg = -1;
    float minSegHeight = 100;
    for (int i=0; i < my_HOR_clusters.size(); i++)
    {
        float HighsegHeight = GetMaxOfSeg(segCloud,my_HOR_clusters[i]);
        if (HighsegHeight >= maxSegHeight) {
            maxSegHeight = HighsegHeight;
            maxSeg = i;
        }
        float LowsegHeight = GetMinOfSeg(segCloud,my_HOR_clusters[i]);
        if (LowsegHeight <= minSegHeight) {
            minSegHeight = LowsegHeight;
            minSeg = i;
        }
    }

    vector <PointIndices::Ptr> extent_clusters;

    extent_clusters = my_VERT_clusters;

    extent_clusters.push_back(my_HOR_clusters[maxSeg]);
    extent_clusters.push_back(my_HOR_clusters[minSeg]);

//    cout<<"-----------------------------------------------"<<endl;
//    cout<<"Returning Segments..."<<endl;
    return make_tuple(extent_clusters, segCloud);
}
