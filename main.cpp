#include <iostream>
#include <vector>
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
#include <constants.h>

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
//OBB
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/pca.h>




using namespace pcl;
using namespace std;


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
        hull.setDimension(HC.setDimConcave);

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
        hull.setDimension(HC.setDimConvex);
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

    PointCloud <PointXYZRGB>::Ptr cloud(new PointCloud <PointXYZRGB>);
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);

    pcl::VoxelGrid<PointXYZRGB> Vox;
    Vox.setInputCloud (pre_Filtered_cloud);
    Vox.setLeafSize (VG.leafSizeX,VG.leafSizeY, VG.leafSizeZ);
    Vox.filter (*cloud);

    *cloud = *hull;

//    CloudOperations CO;
//    CO.Viewer(cloud);

    normals = normalCalc(cloud);

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

    string s = to_string(i);

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

}

int
removeClusterOnVerticality(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster)
{

    if (cluster->indices.size() == 0)
        return 0;

    ///////////////////////////////////////////
    /// Fitting a plae and taking the a,b,c values as normal to the plane
    /// should probaby be done with PCA for speed but for now it works
    ///
    /// if angle from vert greater than set value segment rejected
    ///
    //////////////////////////////////////////

    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    SACSegmentation<PointXYZRGB> segmentation;
    segmentation.setInputCloud(input_cloud);
    segmentation.setIndices(cluster);
    segmentation.setModelType(SACMODEL_PLANE);
    segmentation.setMethodType(SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    PointIndices::Ptr inlierIndices(new PointIndices);
    segmentation.segment(*inlierIndices, *coefficients);

    //Model Coeff and vert axis creating angle from vertical

    Eigen::Vector3f plane_normal(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
    Eigen::Vector3f vert_axis(0.0,0.0,1.0);

    float angleFromVert = acos(plane_normal.dot(vert_axis))* 180.0/M_PI;

    if (angleFromVert >= 45.0){
            return 1;
    }
    else{
        return 0;
    }

}


int
removeClusterOnSize(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster)
{

    if (cluster->indices.size() == 0)
        return 0;
//    cout<< cluster->indices.size() <<endl;
    /////////////////////////////////////////////////////////////
    ///
    /// detemining the heght of the segment
    /// if it is bigger than set height then
    /// reject them if the are too big.
    ///
    /// done by finiding max Z and min Z then differencing
    ///
    /////////////////////////////////////////////////////////////


    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (input_cloud);
    PointCloud <PointXYZRGB>::Ptr cloud(new PointCloud <PointXYZRGB>);
    filtrerG.setIndices(cluster);
    filtrerG.filter(*cloud);

    float max_Z = cloud->points[0].z;
    float min_Z = cloud->points[0].z;

    for(int i = 0; i < cloud->points.size(); i++ ){
        if(cloud->points[i].z >= max_Z){
            max_Z = cloud->points[i].z;
        }
    }

    for(int j = 0; j < cloud->points.size(); j++ ){
        if(cloud->points[j].z <= min_Z){
            min_Z = cloud->points[j].z;
        }
    }

    float gap = max_Z - min_Z;

    gap = sqrt(gap*gap);

//    cout<<min_Z<<" - "<<max_Z<<" - "<<gap<<endl;

    if(gap > 0.75){
        return 1;
    }
    else{
        return 0;
    }
}


PointCloud <PointXYZRGB>::Ptr
segmentor(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals){
    ///////////////////////////////////////////////
    ///http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php
    ///////////////////////////////////////////////
    RegionGrowingConst RGC;

    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);

    IndicesPtr indices (new vector <int>);
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
    vector <PointIndices> clusters;
    cout<<"Cluster Extracton Starting..."<< endl;
    reg.extract (clusters);
    cout<<"Cluster Extracton Sucessfull..."<< endl;

    cloud = reg.getColoredCloud();


    //////////////////////////////////////////////////////////////////////////
    /// Cluster sejecting section
    ///
    /// checks for size then for verticality.
    /// the size and verticality return a 1 if the cluster succeeds
    /// if it fails it sets the cluster to 0 and removes the cluster later
    ///
    /////////////////////////////////////////////////////////////////////////
    cout<<" "<<endl;
    cout<<" Staring Cluster Removal .. "<<endl;
    vector <PointIndices::Ptr> my_VERT_clusters;
    vector <PointIndices::Ptr> my_HOR_clusters;
//    my_VERT_clusters.resize(clusters.size());
    for (int i=0; i < clusters.size(); i++)
    {
        cout<<"----------------------Start Cluster"<<endl;
        PointIndices::Ptr tmp_clusterR(new PointIndices(clusters[i]));
        cout<<"-checking SIZE"<<endl;
        if (removeClusterOnSize(cloud,tmp_clusterR) != 1){
            my_HOR_clusters.push_back(tmp_clusterR);
            cout<<"Removed cluster after SIZE check..."<<endl;
            continue;
        }
        cout<<"-checking VERT"<<endl;
        if (removeClusterOnVerticality(cloud,tmp_clusterR) != 1){
            my_HOR_clusters.push_back(tmp_clusterR);
            cout<<"Removed cluster after VERT check..."<<endl;
            continue;
        }
        else{
            my_VERT_clusters.push_back(tmp_clusterR);
//            my_VERT_clusters[i] = tmp_clusterR;
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    ///
    /// Concatinating the clusters into the full cloud
    /// making sure to remove clusters that have been removed and made = to 0
    ///
    ///////////////////////////////////////////////////////////////////////////

    PointCloud <PointXYZRGB>::Ptr result(new PointCloud <PointXYZRGB>);
    ExtractIndices<PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (cloud);
    for (int i=0; i < my_VERT_clusters.size(); i++){
        if (my_VERT_clusters[i] == 0){
            continue;
        }
        PointCloud <PointXYZRGB>::Ptr clusterCloud(new PointCloud <PointXYZRGB>);
        PointCloud <PointXYZRGB>::Ptr inter2(new PointCloud <PointXYZRGB>);
        inter2 = result;
        filtrerG.setIndices(my_VERT_clusters[i]);
        filtrerG.filter(*clusterCloud);

        PointCloud <PointXYZRGB>::Ptr inter(new PointCloud <PointXYZRGB>);
        inter = clusterCloud;

        *result = *inter2 + *inter;

        if (RGC.Triangulation_Y_N){
            saveTriangles(clusterCloud,inter,i);
        }

    }
    return  result;
}

int
main()
{


    timeDate tmd;
    tmd.print(1);
    cout<<"Start"<< endl;

    CloudOperations CO;
    displayPTcloud DPT;

    string filename = "../ptClouds/DeepSpace-Full";
    PointCloud<PointXYZRGB>::Ptr cloud =  CO.openCloud(filename + ".pcd");


    cout<<"Calculating Normals..."<< endl;

    PointCloud<Normal>::Ptr normals = normalCalc(cloud);

    cout<<"Normals Calculated..."<< endl;


    cout<<"Segmentation Starting..."<< endl;

    cloud = segmentor(cloud, normals);

    cout<<"Writing Cloud to File..."<<endl;
    string outputFileName = filename + "-Segmented";
    DPT.write(cloud,outputFileName + ".pcd");

    tmd.print(1);
    cout<<"Displaying Cloud..."<< endl;
    CO.Viewer(cloud);




    cout<<"End"<< endl;


    return 0;
}






