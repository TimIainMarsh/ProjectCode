#ifndef CLOUDOPERATIONS_H
#define CLOUDOPERATIONS_H

#include <tuple>
#include <vector>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace std;

PointCloud <PointXYZRGB>::Ptr openCloud(string filename);

void Viewer(PointCloud <PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals);

void Viewer(PointCloud <PointXYZRGB>::Ptr cloud);

//    void Viewer(PointCloud <PointXYZRGB>::Ptr cloud, vector <ModelCoefficients::Ptr> coeff);

PointCloud<Normal>::Ptr normalCalc(PointCloud<PointXYZRGB>::Ptr cloud);

PointCloud <PointNormal>::Ptr XYZRGBtoPointNormal(PointCloud <PointXYZRGB>::Ptr cloud,PointCloud<Normal>::Ptr normals);

//ModelCoefficients::Ptr planeFitting(PointCloud <PointXYZRGB>::Ptr cloud);

void SubSampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

void saveTriangles(PointCloud <PointXYZRGB>::Ptr pre_Filtered_cloud,PointCloud <PointXYZRGB>::Ptr hull,int i);

PointCloud<PointXYZRGB>::Ptr vectorToCloud(vector <PointIndices::Ptr> indices, PointCloud <PointXYZRGB>::Ptr cloud);

#endif // CLOUDOPERATIONS_H
