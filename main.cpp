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
/*use to write or save cloud*/

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std;

void
simpleViewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud){

    pcl::visualization::CloudViewer viewer ("The Cloud");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
openCloud(char filename[]){


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile (filename, *cloud);

    return cloud;
}

void
normalComp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){


    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    ne.setKSearch(30);

    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,400, 0.5, "normals");

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
}



pcl::PointCloud <pcl::PointXYZRGB>::Ptr
simpleColourSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);


    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-10.0, 10.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    simpleViewer(colored_cloud);
    return colored_cloud;
}




int
main(int argc, char** argv)
{
    cout<<"START"<< endl;

    displayPTcloud display;
    timeDate tmd;
    tmd.print(1);

    /*Begining*/
    /*Reading The Cloud*/
    char filename[] = "../ptClouds/GTL_3 - Cloud.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = openCloud(filename);
    /*Finished Reading Cloud*/



    simpleColourSeg(cloud);

  /*end*/
  return 0;
}




















