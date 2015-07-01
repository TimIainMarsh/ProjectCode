#include <iostream>
#include <vector>

//generic
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>
//visulisation
#include <pcl/visualization/cloud_viewer.h>

//normals
#include <pcl/features/normal_3d.h>

//my includes
#include <timedate.h>
//#include <displayptcloud.h>
#include <cloudoperations.h>
//region growing

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

//triangulation
#include <pcl/surface/gp3.h>


using namespace pcl;


pcl::PointCloud<pcl::Normal>::Ptr
normalCalc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

    return normals;
}


//PointCloud <PointXYZRGB>::Ptr
//simpleColourSeg(PointCloud<PointXYZRGB>::Ptr cloud){

//    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);


//    IndicesPtr indices (new std::vector <int>);
//    PassThrough<PointXYZRGB> pass;
//    pass.setInputCloud (cloud);
////    pass.setFilterFieldName ("z");
////    pass.setFilterLimits (-10.0, 10.0);
//    pass.filter (*indices);

//    RegionGrowing<PointXYZ, Normal> reg;
//    reg.setInputCloud (cloud);
//    reg.setIndices (indices);
//    reg.setSearchMethod (tree);
//    reg.setDistanceThreshold (10);
//    reg.setPointColorThreshold (6);
//    reg.setRegionColorThreshold (5);
//    reg.setMinClusterSize (600);

//    std::vector <PointIndices> clusters;
//    reg.extract (clusters);
//    PointCloud <PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    return colored_cloud;
//}


PointCloud<PointXYZRGB>::Ptr
segmentor(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals){

//    int min_cluster = 50;
//    int max_cluster = 10000000;


    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);


    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    PointCloud<PointXYZRGB>::Ptr segCloud = reg.getColoredCloud();

    return segCloud;
}



int
main(int argc, char** argv)
{
    std::cout<<"START"<< std::endl;

    timeDate tmd;
    CloudOperations CO;

    tmd.print(1);


    char filename[] = "../ptClouds/GTL_3 - Cloud.pcd";

    PointCloud<PointXYZRGB>::Ptr cloud =  CO.openCloud(filename);

    PointCloud<Normal>::Ptr normals = normalCalc(cloud);

    PointCloud<PointXYZRGB>::Ptr segCloud = segmentor(cloud, normals);

    CO.Viewer(segCloud);
    tmd.print(1);
    return 0;
}




















