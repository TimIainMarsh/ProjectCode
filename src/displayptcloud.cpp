#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "displayptcloud.h"

displayPTcloud::displayPTcloud()
{
    //nothing
}


void
displayPTcloud::Print(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

}

void
displayPTcloud::write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::io::savePCDFileASCII("/home/tim/git/ResearchProjectCode/ptClouds/test_new.pcd", *cloud);
    std::cout << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;

}
