#ifndef DISPLAYPTCLOUD_H
#define DISPLAYPTCLOUD_H

class displayPTcloud
{
public:
    displayPTcloud();

    void Print(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    void write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename);
};

#endif // DISPLAYPTCLOUD_H
