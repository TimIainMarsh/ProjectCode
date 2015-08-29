                                                                                                                                                                                         #include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d_omp.h>
#include <boost/thread/thread.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <cloudoperations.h>
#include <constants.h>


using namespace pcl;
using namespace std;

CloudOperations::CloudOperations()
{
    //nothing
}


pcl::PointCloud <pcl::PointXYZRGB>::Ptr
CloudOperations::openCloud(std::string filename){

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
CloudOperations::Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud){

    pcl::visualization::PCLVisualizer viewer("Cloud");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

    viewer.setBackgroundColor (0.0, 0.0, 0.0);

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

PointCloud <PointNormal>::Ptr
XYZRGBtoPointNormal(PointCloud <PointXYZRGB>::Ptr cloud,PointCloud<Normal>::Ptr normals){

    PointCloud<PointNormal>::Ptr cloud_with_Normals (new PointCloud<PointNormal>);

    // Fill in the cloud data
    cloud_with_Normals->width    = cloud->points.size();
    cloud_with_Normals->height   = 1;
    cloud_with_Normals->is_dense = false;
    cloud_with_Normals->points.resize (cloud_with_Normals->width * cloud_with_Normals->height);

    for (size_t i = 0; i < cloud_with_Normals->points.size (); ++i)
    {
      cloud_with_Normals->points[i].x = float(cloud->points[i].x);
      cloud_with_Normals->points[i].y = float(cloud->points[i].y);
      cloud_with_Normals->points[i].z = float(cloud->points[i].z);

      cloud_with_Normals->points[i].normal[0] = float(normals->points[i].normal[0]);
      cloud_with_Normals->points[i].normal[1] = float(normals->points[i].normal[1]);
      cloud_with_Normals->points[i].normal[2] = float(normals->points[i].normal[2]);
    }

    return cloud_with_Normals;
}


PointCloud<Normal>::Ptr
normalCalc(PointCloud<PointXYZRGB>::Ptr cloud){
    /*http://pointclouds.org/documentation/tutorials/normal_estimation.php*/

    NormalEstConst NE;


    NormalEstimationOMP<PointXYZRGB, Normal> n;
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (NE.KSearch);
    n.compute (*normals);

    return normals;
}

ModelCoefficients::Ptr
planeFitting(PointCloud <PointXYZRGB>::Ptr cloud){

    //not being used
    cout<<"not used"<<endl;

    /*http://pointclouds.org/documentation/tutorials/planar_segmentation.php*/

//    cout<<cloud->size()<<endl;

    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    // Create the segmentation object
    SACSegmentation<PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setInputCloud (cloud);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setDistanceThreshold (0.01);


    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      exit(-1);
    }
    return coefficients;

}
