#include <iostream>
#include <vector>
#include <tuple>
#include <string>

//generic
#include <pcl/common/io.h>
#include <pcl/point_types.h>

//Writing files
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
//Searching
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <boost/thread/thread.hpp>
//visulisation
#include <pcl/visualization/cloud_viewer.h>
//normals
#include <pcl/features/normal_3d_omp.h>

//my includes
#include <timedate.h>
#include <displayptcloud.h>
#include <cloudoperations.h>
//region growing
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/surface/concave_hull.h>

//Triangulation
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/vtk_io.h>

#include <pcl/filters/voxel_grid.h>

#include <constants.h>

using namespace pcl;


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
    std::cout<<"not used"<<std::endl;

    /*http://pointclouds.org/documentation/tutorials/planar_segmentation.php*/

//    std::cout<<cloud->size()<<std::endl;

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

PointCloud<PointXYZRGB>::Ptr
BoundryDetection(PointCloud<PointXYZRGB>::Ptr cloud, int decide){

    /*http://pointclouds.org/documentation/tutorials/hull_2d.php*/

    HullConst HC;

    PointCloud<PointXYZRGB>::Ptr plane(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cloudHull(new PointCloud<PointXYZRGB>);

    // Get the plane model, if present.
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    SACSegmentation<PointXYZRGB> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(SACMODEL_PLANE);
    segmentation.setMethodType(SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    PointIndices::Ptr inlierIndices(new PointIndices);
    segmentation.segment(*inlierIndices, *coefficients);



    if (decide == 1){   
        //CONCAVE HULL

        // Copy the points of the plane to a new cloud.
        ExtractIndices<PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inlierIndices);
        extract.filter(*plane);

        // Object for retrieving the concave hull.
        ConcaveHull<PointXYZRGB> hull;
        hull.setInputCloud(plane);
        hull.setKeepInformation(HC.KeepInfo);
        // Set alpha, which is the maximum length from a vertex to the center of the voronoi cell
        // (the smaller, the greater the resolution of the hull).
        hull.setAlpha(HC.Alpha);   //--->0.1
        hull.reconstruct(*cloudHull);

        return cloudHull;
    }

    if(decide == 2){

        //CONVEX HULL


        ExtractIndices<PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inlierIndices);
        extract.filter(*plane);


        ConvexHull<PointXYZRGB> hull;
        hull.setInputCloud(plane);
        hull.reconstruct(*cloudHull);
        return cloudHull;

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


void
saveTriangles(PointCloud <PointXYZRGB>::Ptr pre_Filtered_cloud,PointCloud <PointXYZRGB>::Ptr hull,int i){

    TriangulationConst TC;
    VoxGridConst VG;

    // Normal estimation*
    PointCloud <PointXYZRGB>::Ptr cloud(new PointCloud <PointXYZRGB>);;

    pcl::VoxelGrid<PointXYZRGB> sor;
    sor.setInputCloud (pre_Filtered_cloud);



    sor.setLeafSize (VG.leafSizeX,VG.leafSizeY, VG.leafSizeZ);
    sor.filter (*cloud);

    *cloud = *cloud + *hull;


    PointCloud<Normal>::Ptr normals = normalCalc(cloud);

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

    std::string s = std::to_string(i);

    if (TC.PLY_OBJ_VTK == "PLY"){
        std::string name = "mesh/PLY/mesh" + s + ".ply";
        std::cout<<"Writing .. " + name<< std::endl;
        io::savePLYFile (name, triangles);
    }
    else if (TC.PLY_OBJ_VTK == "OBJ"){
        std::string name = "mesh/OBJ/mesh" + s + ".obj";
        std::cout<<"Writing .. " + name<< std::endl;
        io::saveOBJFile (name, triangles);
    }
    else if (TC.PLY_OBJ_VTK == "VTK"){
        std::string name = "mesh/VTK/mesh" + s + ".vtk";
        std::cout<<"Writing .. " + name<< std::endl;
        io::saveVTKFile (name, triangles);
    }
    else{
        PCL_ERROR ("Please select a valid file format of either VTK, PLY of OBJ");
    }

}


PointCloud <PointXYZRGB>::Ptr
segmentor(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals){

    /*http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php*/

    RegionGrowingConst RGC;

    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);

    IndicesPtr indices (new std::vector <int>);
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud (cloud);

    pass.filter (*indices);

    RegionGrowing<PointXYZRGB, Normal> reg;

    reg.setMinClusterSize (RGC.MinClusterSize);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (RGC.NumberOfNeighbours);
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setInputNormals (normals);

    if (!RGC.fast){
        reg.setSmoothnessThreshold (RGC.SmoothnessThreshold);
        reg.setCurvatureThreshold (RGC.CurvatureThreshold);
    }
    std::vector <PointIndices> clusters;
    std::cout<<"Cluster Extracton Starting..."<< std::endl;
    reg.extract (clusters);
    std::cout<<"Cluster Extracton Sucessfull..."<< std::endl;

    std::vector <PointIndices::Ptr> my_clusters;
    my_clusters.resize(clusters.size());
    for (int i=0; i < clusters.size(); i++)
    {
        PointIndices::Ptr tmp_clusterR(new PointIndices(clusters[i]));
        my_clusters[i] = tmp_clusterR;
    }
    PointCloud<PointXYZRGB>::Ptr segCloud = reg.getColoredCloud();





    PointCloud <PointXYZRGB>::Ptr result(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (segCloud);

    for (int i=0; i < clusters.size(); i++){
        PointCloud <PointXYZRGB>::Ptr clusterCloud(new PointCloud <PointXYZRGB>);
        PointCloud <PointXYZRGB>::Ptr inter2(new PointCloud <PointXYZRGB>);
        inter2 = result;
        filtrerG.setIndices(my_clusters[i]);
        filtrerG.filter(*clusterCloud);

        PointCloud <PointXYZRGB>::Ptr inter(new PointCloud <PointXYZRGB>);

        //   1 --> Concave hull  2 --> Convex hull
        inter = BoundryDetection(clusterCloud,1);
//        inter = clusterCloud;
        saveTriangles(clusterCloud,inter,i);

        *result = *inter + *inter2;
    }
    std::cout<<" "<<std::endl;
    return  result;
}



int
main(int argc, char** argv)
{


    timeDate tmd;
    tmd.print(1);
    std::cout<<"Start"<< std::endl;

    CloudOperations CO;
    displayPTcloud DPT;

    std::string filename = "../ptClouds/GTL-CutDown";
    PointCloud<PointXYZRGB>::Ptr cloud =  CO.openCloud(filename + ".pcd");



    std::cout<<"Calculating Normals..."<< std::endl;

    PointCloud<Normal>::Ptr normals = normalCalc(cloud);

    std::cout<<"Normals Calculated..."<< std::endl;



    std::cout<<"Segmentation Starting..."<< std::endl;

    PointCloud<PointXYZRGB>::Ptr segmentedCloud = segmentor(cloud, normals);

    std::cout<<"Writing Cloud to File..."<<std::endl;
    std::string outputFileName = filename + "-Segmented";
    DPT.write(segmentedCloud,outputFileName + ".pcd");


    std::cout<<"Displaying Cloud..."<< std::endl;
    CO.Viewer(segmentedCloud);




    std::cout<<"End"<< std::endl;
    tmd.print(1);

    return 0;
}






