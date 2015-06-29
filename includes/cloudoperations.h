#ifndef CLOUDOPERATIONS_H
#define CLOUDOPERATIONS_H

class CloudOperations
{
public:
    CloudOperations();

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr openCloud(char filename[]);

    void simpleViewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    void simpleViewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);

//    pcl::PointCloud <pcl::Normal>::Ptr normalComp(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);

};

#endif // CLOUDOPERATIONS_H
