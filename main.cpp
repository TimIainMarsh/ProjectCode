#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>


#include <timedate.h>
#include <displayptcloud.h>
#include <cloudoperations.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

using namespace pcl;

pcl::PointCloud<pcl::Normal>::Ptr
normalComp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){


    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    NormalEstimation<PointXYZRGB, Normal> ne;
    ne.setInputCloud(cloud);

    search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
    ne.setSearchMethod (tree);
    ne.setKSearch(15);

    ne.compute(*normals);

    return normals;
}


PointCloud <PointXYZRGB>::Ptr
simpleColourSeg(PointCloud<PointXYZRGB>::Ptr cloud){

    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);


    IndicesPtr indices (new std::vector <int>);
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (-10.0, 10.0);
    pass.filter (*indices);

    RegionGrowingRGB<PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);

    std::vector <PointIndices> clusters;
    reg.extract (clusters);
    PointCloud <PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    return colored_cloud;
}


PointCloud<PointXYZRGB>::Ptr
segmentor(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals){

    int min_cluster = 600;
    int max_cluster = 1000000;


    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);

    IndicesPtr indices (new std::vector <int>);
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.filter (*indices);

    RegionGrowingRGB<PointXYZRGB> reg;

    reg.setMinClusterSize (min_cluster);
    reg.setMaxClusterSize (max_cluster);

    reg.setInputCloud (cloud);
    reg.setInputNormals (normals);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);

    reg.setNumberOfNeighbours(20);

    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);


//    reg.setSmoothnessThreshold (5);
//    reg.setCurvatureThreshold (5);

    std::vector <PointIndices> clusters;
    reg.extract (clusters);

    PointCloud<PointXYZRGB>::Ptr segCloud = reg.getColoredCloud();

    return segCloud;
}


int
main(int argc, char** argv)
{
    std::cout<<"START"<< std::endl;

    displayPTcloud display;
    timeDate tmd;
    CloudOperations CO;

    tmd.print(1);

    /*Begining*/


    char filename[] = "../ptClouds/GTL_3 - Cloud.pcd";
    PointCloud<PointXYZRGB>::Ptr start_cloud = CO.openCloud(filename);
    std::cout << start_cloud->size() << " points" <<std::endl;
    /*Finished Reading Cloud*/


    PointCloud<Normal>::Ptr normals = normalComp(start_cloud);

    PointCloud<PointXYZRGB>::Ptr segCloud = segmentor(start_cloud, normals);

    CO.simpleViewer(segCloud);


  /*end*/
  return 0;
}




















