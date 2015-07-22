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
using namespace std;

CloudOperations::CloudOperations()
{
    //nothing
}


pcl::PointCloud <pcl::PointXYZRGB>::Ptr
CloudOperations::openCloud(char filename[]){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile (filename, cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

    return cloud;

}


void
CloudOperations::Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud, std::vector <pcl::ModelCoefficients::Ptr> coeff){

    pcl::visualization::PCLVisualizer viewer("Cloud and Normals");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

//    pcl::ModelCoefficients coeffs;
//    coeffs.values.push_back (-0.00137426 );
//    coeffs.values.push_back ( 0.00015838 );
//    coeffs.values.push_back ( 0.999999 );
//    coeffs.values.push_back ( 1.74278);
//    viewer.addPlane (coeffs, "plane");

    for (int i=0; i < coeff.size(); i++){
//        cout<< typeid(coeff[i]).name() <<endl;
        string Result;
        ostringstream convert;
        convert << i;
        Result = convert.str();

        viewer.addPlane (*coeff[i],1,1,1,Result,0);

    }

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
}

void
CloudOperations::Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){

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



