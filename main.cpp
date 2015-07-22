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
#include <displayptcloud.h>
#include <cloudoperations.h>
//region growing

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/projection_matrix.h>


using namespace pcl;


pcl::PointCloud<pcl::Normal>::Ptr
normalCalc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (40);
    n.compute (*normals);

    return normals;
}



std::vector <pcl::PointIndices::Ptr>
segmentor(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals){



    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);


    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;

    reg.setMinClusterSize (50);

    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (50); //20
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
//    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
//    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);


    std::vector <pcl::PointIndices::Ptr> my_clusters;
    my_clusters.resize(clusters.size());
    for (int i=0; i < clusters.size(); i++)
    {
         pcl::PointIndices::Ptr tmp_clusterR(new  pcl::PointIndices(clusters[i]));
         my_clusters[i] = tmp_clusterR;
    }

    return my_clusters;

//    PointCloud<PointXYZRGB>::Ptr segCloud = reg.getColoredCloud();
//    return segCloud;
}





int
main(int argc, char** argv)
{
    std::cout<<"START"<< std::endl;

    timeDate tmd;
    CloudOperations CO;
//    displayPTcloud dPT;

    tmd.print(1);


    char filename[] = "../ptClouds/GTL-CutDown.pcd";

    PointCloud<PointXYZRGB>::Ptr cloud =  CO.openCloud(filename);

    PointCloud<Normal>::Ptr normals = normalCalc(cloud);

    std::vector <pcl::PointIndices::Ptr> segmented_clusters = segmentor(cloud, normals);



    pcl::PointCloud <pcl::PointXYZRGB>::Ptr fourth(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (cloud);
    for (int i=0; i < segmented_clusters.size(); i++){

        filtrerG.setIndices(segmented_clusters[i]);
        filtrerG.filter(*fourth);
        CO.Viewer(fourth);
        std::cout<< i << std::endl;
//        fourth.reset();
    }
//    CO.Viewer(segCloud);
    tmd.print(1);

    return 0;
}












//std::vector <pcl::PointIndices::Ptr> my_clusters;
//my_clusters.resize(clusters.size());
//for (int i=0; i < clusters.size(); i++)
//{
//     pcl::PointIndices::Ptr tmp_clusterR(new  pcl::PointIndices(clusters[i]));
//     my_clusters[i] = tmp_clusterR;
//}


//pcl::PointCloud <pcl::PointXYZRGB>::Ptr fourth(new pcl::PointCloud <pcl::PointXYZRGB>);
//pcl::ExtractIndices<pcl::PointXYZRGB> filtrerG (true);
//filtrerG.setInputCloud (cloud);
//filtrerG.setIndices(my_clusters[0]); //indices of the fouth cluster.
//filtrerG.filter(*fourth);







