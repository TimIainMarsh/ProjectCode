#ifndef Functions_H
#define Functions_H

#include <tuple>
#include <vector>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

void Print(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

void write(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::string filename);

float RandomFloat(float min, float max);

#endif // Functions_H
