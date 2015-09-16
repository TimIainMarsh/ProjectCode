#ifndef Functions_H
#define Functions_H

#include <tuple>
#include <vector>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

void Print(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename);


#endif // Functions_H
