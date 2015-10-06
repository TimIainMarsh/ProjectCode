#ifndef SEGMENTOR_H
#define SEGMENTOR_H

#include <tuple>
#include <vector>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

//using namespace std;

//int removeClusterOnVerticality(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster);\

//int removeClusterOnSize(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster);

//int GetHORclusters(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster);

//float GetMaxOfSeg(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster);

//float GetMinOfSeg(PointCloud<PointXYZRGB>::Ptr input_cloud, PointIndices::Ptr cluster);

std::tuple<  std::vector <pcl::PointIndices::Ptr>, std::vector <pcl::PointIndices::Ptr> , pcl::PointCloud<pcl::PointXYZRGB>::Ptr  >
segmentor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normals);

#endif // SEGMENTOR_H
