#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <typeinfo>
#include <math.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/io.h>
#include <pcl/point_types.h>

#ifndef PROJECTION_H
#define PROJECTION_H



class Projection
{
public:
    Projection();
};

void
ExtractCornerPoints(const std::vector <pcl::PointIndices::Ptr> &vector_of_segments,  const std::vector <pcl::PointIndices::Ptr> &vector_of_roof_floor, const std::vector < pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &Boundries,const pcl::PointCloud <pcl::PointXYZRGB>::Ptr &cloud);

std::vector < pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
getBoundriesOfSegments(const std::vector <pcl::PointIndices::Ptr> &vector_of_segments, const pcl::PointCloud <pcl::PointXYZRGB>::Ptr &segCloud);

#endif // PROJECTION_H
