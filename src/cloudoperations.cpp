#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include "cloudoperations.h"


using namespace pcl;

CloudOperations::CloudOperations()
{
    //nothing
}

pcl::PointCloud <pcl::PointXYZRGB>::Ptr
CloudOperations::openCloud(char filename[]){


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile (filename, *cloud);

    return cloud;
}


void
CloudOperations::simpleViewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud){

    pcl::visualization::CloudViewer viewer ("The Cloud");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
}

void
CloudOperations::simpleViewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){

//    std::cout<< "the knn value used is: "<< m_knn<<std::endl;

    pcl::visualization::PCLVisualizer viewer("Cloud and Normals");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,400, 0.5, "normals");

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
}

