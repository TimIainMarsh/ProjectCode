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
#include <timedate.h>
#include <displayptcloud.h>
#include <cloudoperations.h>
#include <constants.h>

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
removeClusterOnVerticality(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster)
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
    cout<<angleFromVert<<endl;
    if (angleFromVert >= 30.0){
            return 1;
    }
    else{
        return 0;
    }

}


int
removeClusterOnSize(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster)
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

    for(int i = 0; i < cloud->points.size(); i++ ){
        if(cloud->points[i].z >= max_Z){
            max_Z = cloud->points[i].z;
        }
    }

    for(int j = 0; j < cloud->points.size(); j++ ){
        if(cloud->points[j].z <= min_Z){
            min_Z = cloud->points[j].z;
        }
    }

    float gap = max_Z - min_Z;

    gap = sqrt(gap*gap);

//    cout<<min_Z<<" - "<<max_Z<<" - "<<gap<<endl;

    if(gap > 0.75){
        return 1;
    }
    else{
        return 0;
    }
}


int
GetHORclusters(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster){

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
    if (angleFromVert <= 60.0){
            return 1;
    }
    else{
        return 0;
    }

}

float
GetMaxOfSeg(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster)
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

    for(int i = 0; i < cloud->points.size(); i++ ){
        if(cloud->points[i].z >= max_Z){
            max_Z = cloud->points[i].z;
        }
    }
    return max_Z;
}

float
GetMinOfSeg(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster)
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

    for(int i = 0; i < cloud->points.size(); i++ ){
        if(cloud->points[i].z <= min_Z){
            min_Z = cloud->points[i].z;
        }
    }
    return min_Z;
}

tuple<  pcl::PointXYZRGB , pcl::PointXYZRGB  >
getBounding(PointCloud <PointXYZRGB>::Ptr cloud)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    pcl::PointXYZRGB min_point_OBB;
    pcl::PointXYZRGB max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    return make_tuple(min_point_OBB, max_point_OBB);
}
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


tuple<  vector <PointIndices::Ptr> , PointCloud<PointXYZRGB>::Ptr  >
segmentor(PointCloud<PointXYZRGB>::Ptr input_cloud, PointCloud<Normal>::Ptr normals){
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

    if (!RGC.fast){
        reg.setSmoothnessThreshold (RGC.SmoothnessThreshold);
        reg.setCurvatureThreshold (RGC.CurvatureThreshold);
    }
    vector <PointIndices> clusters;
    cout<<"Cluster Extracton Starting..."<< endl;
    reg.extract (clusters);
    cout<<"Cluster Extracton Sucessfull..."<< endl;

    vector <PointIndices::Ptr> my_clusters;

    for (int i=0; i < clusters.size(); i++)
    {
        PointIndices::Ptr tmp_clusterR(new PointIndices(clusters[i]));
        my_clusters.push_back(tmp_clusterR);
    }

    PointCloud <PointXYZRGB>::Ptr cloud = reg.getColoredCloud(); //replaces the input cloud with one coloured accoring to segments

//    CloudOperations CO;
//    CO.Viewer(cloud);
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
    cout<<" Staring Cluster Removal .. "<<endl;

    vector <PointIndices::Ptr> my_VERT_clusters;
//    vector <PointIndices::Ptr> my_HOR_clusters;

    for (int i=0; i < clusters.size(); i++)
    {
        cout<<"----------------------Start Cluster"<<endl;
        cout<<"-checking HOR"<<endl;

        cout<<"-checking SIZE"<<endl;
        if (removeClusterOnSize(cloud,my_clusters[i]) != 1){
//            cout<<"Removed cluster after SIZE check..."<<endl;
            continue;
        }
        cout<<"-checking VERT"<<endl;
        if (removeClusterOnVerticality(cloud,my_clusters[i]) != 1){
//            cout<<"Removed cluster after VERT check..."<<endl;
            continue;
        }
        else{
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
        if (GetHORclusters(cloud,my_clusters[i]) == 1){
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
        float HighsegHeight = GetMaxOfSeg(cloud,my_HOR_clusters[i]);
        if (HighsegHeight >= maxSegHeight){
            maxSegHeight = HighsegHeight;
            maxSeg = i;
        }
        float LowsegHeight = GetMinOfSeg(cloud,my_HOR_clusters[i]);
        if (LowsegHeight <= minSegHeight){
            minSegHeight = LowsegHeight;
            minSeg = i;
        }
    }

    cout<<"Max: "<<maxSeg<<" -   "<<maxSegHeight<<endl;
    cout<<"Min: "<<minSeg<<" -   "<<minSegHeight<<endl;

    ///////////////////////////////////////////////////////////////////////////
    ///
    /// Concatinating the clusters into the full cloud
    ///
    ///////////////////////////////////////////////////////////////////////////

    PointCloud <PointXYZRGB>::Ptr result(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (cloud);
    for (int i=0; i < my_VERT_clusters.size(); i++){

        PointCloud <PointXYZRGB>::Ptr clusterCloud(new PointCloud <PointXYZRGB>);
        PointCloud <PointXYZRGB>::Ptr inter2(new PointCloud <PointXYZRGB>);
        inter2 = result;
        filtrerG.setIndices(my_VERT_clusters[i]);
        filtrerG.filter(*clusterCloud);

        PointCloud <PointXYZRGB>::Ptr inter(new PointCloud <PointXYZRGB>);
        inter = clusterCloud;

        *result = *inter2 + *inter;
    }


    //////////////////////////////////////////////////////////////////////////////
    ///
    /// after all the vertical clusters added the two extent horizontal
    /// clusters are added
    ///
    /////////////////////////////////////////////////////////////////////////////


    PointCloud <PointXYZRGB>::Ptr _1(new PointCloud <PointXYZRGB>);
    PointCloud <PointXYZRGB>::Ptr _inter_1(new PointCloud <PointXYZRGB>);
    _inter_1 = result;
    filtrerG.setIndices(my_HOR_clusters[maxSeg]);
    filtrerG.filter(*_1);
    PointCloud <PointXYZRGB>::Ptr inter_1(new PointCloud <PointXYZRGB>);
    inter_1 = _1;
    *result = *_inter_1 + *inter_1;

    PointCloud <PointXYZRGB>::Ptr _2(new PointCloud <PointXYZRGB>);
    PointCloud <PointXYZRGB>::Ptr _inter_2(new PointCloud <PointXYZRGB>);
    _inter_2 = result;
    filtrerG.setIndices(my_HOR_clusters[minSeg]);
    filtrerG.filter(*_2);
    PointCloud <PointXYZRGB>::Ptr inter_2(new PointCloud <PointXYZRGB>);
    inter_2 = _2;
    *result = *_inter_2 + *inter_2;



    vector <PointIndices::Ptr> extent_clusters;

    extent_clusters = my_VERT_clusters;

    extent_clusters.push_back(my_HOR_clusters[maxSeg]);
    extent_clusters.push_back(my_HOR_clusters[minSeg]);



    return make_tuple(extent_clusters, cloud);
}

PointCloud<PointXYZRGB>::Ptr
vectorToCloud(vector <PointIndices::Ptr> indices, PointCloud <PointXYZRGB>::Ptr cloud){
    cout<<"In function"<<endl;

    PointCloud <PointXYZRGB>::Ptr result(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (cloud);
    for (int i=0; i < indices.size(); i++){

        PointCloud <PointXYZRGB>::Ptr clusterCloud(new PointCloud <PointXYZRGB>);
        PointCloud <PointXYZRGB>::Ptr inter2(new PointCloud <PointXYZRGB>);
        inter2 = result;
        filtrerG.setIndices(indices[i]);
        filtrerG.filter(*clusterCloud);

        PointCloud <PointXYZRGB>::Ptr inter(new PointCloud <PointXYZRGB>);
        inter = clusterCloud;

        *result = *inter2 + *inter;
    }
    return result;
}

void
newFunction(vector <PointIndices::Ptr> indices, PointCloud <PointXYZRGB>::Ptr cloud)
{
    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (cloud);
    for (int i=0; i < indices.size(); i++){
        PointIndices::Ptr segment = indices[i];
        PointCloud <PointXYZRGB>::Ptr clusterCloud(new PointCloud <PointXYZRGB>);
        filtrerG.setIndices(segment);
        filtrerG.filter(*clusterCloud);
        pcl::PointXYZRGB min_point_OBB;
        pcl::PointXYZRGB max_point_OBB;

        tie(min_point_OBB ,max_point_OBB) = getBounding(clusterCloud);
//        cout<<min_point_OBB<<"   "<< max_point_OBB<<endl;

        ModelCoefficients::Ptr modelCoeff;
        modelCoeff = getModelCoeff(clusterCloud);
        cout<<modelCoeff->values[0]<<"  "<<modelCoeff->values[1]<<"  "<<modelCoeff->values[2]<<"  "<<modelCoeff->values[3]<<endl;

    }
}

int
main()
{
    timeDate tmd;
    CloudOperations CO;
    displayPTcloud DPT;
    tmd.print(1);
    cout<<"Start"<< endl;
    string filename = "../ptClouds/box";
    PointCloud<PointXYZRGB>::Ptr cloud =  CO.openCloud(filename + ".pcd");

    cout<<"Calculating Normals..."<< endl;
    PointCloud<Normal>::Ptr normals = normalCalc(cloud);
    cout<<"Normals Calculated..."<< endl;

    cout<<"Segmentation Starting..."<< endl;

    vector <PointIndices::Ptr> vector_of_segments;

    std::tie(vector_of_segments, cloud) = segmentor(cloud, normals);

    newFunction(vector_of_segments,cloud);

    tmd.print(1);

    cloud = vectorToCloud(vector_of_segments, cloud);
    tmd.print(1);


    cout<<"Writing Cloud to File..."<<endl;
    string outputFileName = filename + "-Segmented";
    DPT.write(cloud,outputFileName + ".pcd");
    tmd.print(1);

    cout<<"Displaying Cloud..."<< endl;
    CO.Viewer(cloud);

    cout<<"End"<< endl;
    return 0;
}






