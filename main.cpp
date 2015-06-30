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
#include <pcl/segmentation/region_growing_rgb.h>

//triangulation
#include <pcl/surface/gp3.h>


using namespace pcl;




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
    int max_cluster = 100000;


    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);

    IndicesPtr indices (new std::vector <int>);
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.filter (*indices);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (-5.0, 0.0);
    pass.filter (*indices);

    RegionGrowingRGB<PointXYZRGB> reg;

    reg.setMinClusterSize (min_cluster);
    reg.setMaxClusterSize (max_cluster);

    reg.setInputCloud (cloud);
    reg.setInputNormals (normals);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);

    reg.setNumberOfNeighbours(30);

//    reg.setDistanceThreshold (10);
//    reg.setPointColorThreshold (6);
//    reg.setRegionColorThreshold (5);


    reg.setSmoothnessThreshold (3.0 / 180.0 * 3.14);
    reg.setCurvatureThreshold (1.0);


    std::vector <PointIndices> clusters;
    reg.extract (clusters);

    PointCloud<PointXYZRGB>::Ptr segCloud = reg.getColoredCloud();

    return segCloud;
}

void
Triangulation(PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr normals){
    std::cout<<"Start"<<std::endl;


    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (150);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);



//    CO.Viewer(triangles);


}


int
main(int argc, char** argv)
{
    std::cout<<"START"<< std::endl;

    timeDate tmd;
    CloudOperations CO;

    tmd.print(1);


    char filename[] = "../ptClouds/GTL_3 - Cloud.pcd";

    PointCloud<PointXYZ>::Ptr cloud =  CO.openCloud(filename);

//    PointCloud<Normal>::Ptr normals = CO.normalCalc(cloud);

    Triangulation(cloud,normals);

    tmd.print(1);
    return 0;
}




















