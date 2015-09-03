#ifndef CLOUDOPERATIONS_H
#define CLOUDOPERATIONS_H

class CloudOperations
{
public:
    CloudOperations();

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr openCloud(std::string filename);

    void Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    void Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);

    void Viewer(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud, std::vector <pcl::ModelCoefficients::Ptr> coeff);
};

pcl::PointCloud<pcl::Normal>::Ptr normalCalc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

pcl::PointCloud <pcl::PointNormal>::Ptr XYZRGBtoPointNormal(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals);

pcl::ModelCoefficients::Ptr planeFitting(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);

#endif // CLOUDOPERATIONS_H
