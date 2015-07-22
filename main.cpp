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

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


using namespace pcl;


pcl::PointCloud<pcl::Normal>::Ptr
normalCalc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    /*http://pointclouds.org/documentation/tutorials/normal_estimation.php*/

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

void
planeFitting(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud){
    /*http://pointclouds.org/documentation/tutorials/planar_segmentation.php*/

    std::cout<<cloud->size()<<std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setInputCloud (cloud);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);


    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      exit(-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl; // 0=a, 1=b,2=c,3=d


}

PointCloud<PointXYZRGB>::Ptr
segmentor(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals){

    /*http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php*/

    search::Search <PointXYZRGB>::Ptr tree = boost::shared_ptr<search::Search<PointXYZRGB> > (new search::KdTree<PointXYZRGB>);


    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;

    reg.setMinClusterSize (100);

    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (20); //20
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);


    std::vector <pcl::PointIndices::Ptr> my_clusters;
    my_clusters.resize(clusters.size());
    for (int i=0; i < clusters.size(); i++)
    {
         pcl::PointIndices::Ptr tmp_clusterR(new  pcl::PointIndices(clusters[i]));
         my_clusters[i] = tmp_clusterR;
    }

    PointCloud<PointXYZRGB>::Ptr segCloud = reg.getColoredCloud();

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> filtrerG (true);
    filtrerG.setInputCloud (segCloud);
    for (int i=0; i < clusters.size(); i++){
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr im(new pcl::PointCloud <pcl::PointXYZRGB>);
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr im2(new pcl::PointCloud <pcl::PointXYZRGB>);
        im2 = result;
        filtrerG.setIndices(my_clusters[i]);
        filtrerG.filter(*im);
        planeFitting(im);
        *result = (*im) + (*im2);
        }

    return result;
}

//planeFitting(fourth);

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

    PointCloud<PointXYZRGB>::Ptr  segmented_cloud = segmentor(cloud, normals);




    CO.Viewer(segmented_cloud);
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







