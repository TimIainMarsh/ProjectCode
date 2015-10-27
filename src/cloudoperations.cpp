#include <iostream>
#include <vector>
#include <string>

#include <pcl/common/io.h>

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

#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

//region growing
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/ModelCoefficients.h>

//Writing files
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

using namespace pcl;
using namespace std;


pcl::PointCloud <pcl::PointXYZRGB>::Ptr
openCloud(std::string filename){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile (filename, cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

    return cloud;
}


//void
//Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud, std::vector <pcl::ModelCoefficients::Ptr> coeff){

//    pcl::visualization::PCLVisualizer viewer("Cloud and Normals");

//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

////    pcl::ModelCoefficients coeffs;
////    coeffs.values.push_back (-0.00137426 );
////    coeffs.values.push_back ( 0.00015838 );
////    coeffs.values.push_back ( 0.999999 );
////    coeffs.values.push_back ( 1.74278);
////    viewer.addPlane (coeffs, "plane");

//    for (int i=0; i < coeff.size(); i++){
////        cout<< typeid(coeff[i]).name() <<endl;
//        string Result;
//        ostringstream convert;
//        convert << i;
//        Result = convert.str();

//        viewer.addPlane (*coeff[i],1,1,1,Result,0);

//    }

//    while (!viewer.wasStopped ())
//    {
//      viewer.spinOnce ();
//    }
//}

void
Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud){

    pcl::visualization::PCLVisualizer viewer("Cloud");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

    viewer.setBackgroundColor (0.0, 0.0, 0.0);

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }

    viewer.close();
}

void
Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){

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

void
SubSampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){


    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);

    pcl::VoxelGrid<PointXYZRGB> Vox;
    Vox.setInputCloud (cloud);
    Vox.setLeafSize (1,1,1);
    Vox.filter (*cloud);
}


void
saveTriangles(PointCloud <PointXYZRGB>::Ptr pre_Filtered_cloud,PointCloud <PointXYZRGB>::Ptr hull,int i){

    TriangulationConst TC;
    VoxGridConst VG;

    PointCloud <PointXYZRGB>::Ptr cloud(new PointCloud <PointXYZRGB>);
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);

    pcl::VoxelGrid<PointXYZRGB> Vox;
    Vox.setInputCloud (pre_Filtered_cloud);
    Vox.setLeafSize (VG.leafSizeX,VG.leafSizeY, VG.leafSizeZ);
    Vox.filter (*cloud);

    *cloud = *hull;

    normals = normalCalc(cloud);

//    CloudOperations CO;

    PointCloud<PointNormal>::Ptr cloud_with_Normals = XYZRGBtoPointNormal(cloud,normals);

    search::KdTree<PointNormal>::Ptr tree2 (new search::KdTree<PointNormal>);
    tree2->setInputCloud (cloud_with_Normals);

    // Initialize objects
    GreedyProjectionTriangulation<PointNormal> gp3;
    PolygonMesh triangles;


    gp3.setSearchRadius (TC.SearchRadius);
    // Set typical values for the parameters
    gp3.setMu (TC.Mu);
    gp3.setMaximumSurfaceAngle(TC.MaximumSurfaceAngle);
    gp3.setMinimumAngle(TC.MaximumAngle);
    gp3.setMaximumAngle(TC.MaximumAngle);
    gp3.setNormalConsistency(TC.NormalConsistency);

    // Get result
    gp3.setInputCloud (cloud_with_Normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

//    string s = to_string(i);

    string s = boost::to_string(i);

    if (TC.PLY_OBJ == "PLY"){
        string name = "../mesh/PLY/mesh" + s + ".ply";
//        cout<<"Writing .. " + name<< endl;
        io::savePLYFile (name, triangles);
    }
    else if (TC.PLY_OBJ == "OBJ"){
        string name = "../mesh/OBJ/mesh" + s + ".obj";
//        cout<<"Writing .. " + name<< endl;
        io::saveOBJFile (name, triangles);

    }
    else{
        PCL_ERROR ("Please select a valid file format of either VTK, PLY of OBJ");
    }

}\



PointCloud<PointXYZRGB>::Ptr
vectorToCloud(vector <PointIndices::Ptr> indices, PointCloud <PointXYZRGB>::Ptr cloud){

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



